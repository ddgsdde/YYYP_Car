#include <Arduino.h>
#include <ArduinoJson.h>
#include "config.h"
#include "LineSensor.h"
#include "MotorControl.h"
#include "Sensors.h"
#include "Display.h"
#include "PIDController.h"
#include "ParameterManager.h"
#include "WebServerManager.h"
#include "ObjectDetector.h"
#include "TaskManager.h"

// 全局对象
LineSensor lineSensor;
MotorControl motor;
Sensors sensors;
Display display;
PIDController pidController(KP_LINE, KI_LINE, KD_LINE);
PIDController encoderPid(1.0, 0, 0); // 编码器直线保持PID
ParameterManager params;
WebServerManager webServer(&params);
ObjectDetector objectDetector(&sensors, &motor);
TaskManager taskManager;

// 状态变量
SystemState currentState = STATE_IDLE;
bool systemRunning = false;  // 系统运行标志

// 跨任务通信标志 (解决并发崩溃问题)
volatile bool pendingTestTurn = false;
volatile bool pendingTestStraight = false;
volatile bool pendingTestAvoid = false;
volatile bool pendingTestParking = false;
enum ManualCommand { CMD_NONE, CMD_STOP, CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, CMD_RIGHT, CMD_TURN_180 };
volatile ManualCommand pendingManualCmd = CMD_NONE;
volatile float pendingManualValue = 0;

// 障碍物检测计数器
int obstacleDetectCount = 0;  // 检测到的障碍物次数
bool obstacleDetectionEnabled = false;  // 是否启用障碍物检测

// 避障子状态
enum AvoidanceSubState {
    AVOID_NONE,
    AVOID_TURN_LEFT,      // 1. 左转离开赛道
    AVOID_FORWARD_OUT,    // 2. 直行离开赛道 (距离可调)
    AVOID_TURN_RIGHT_1,   // 3. 右转 (平行于赛道)
    AVOID_FORWARD_PARALLEL, // 4. 直行 (平行移动)
    AVOID_TURN_RIGHT_2,   // 5. 右转 (面向赛道)
    AVOID_FORWARD_IN,     // 6. 直行寻找黑线
    AVOID_TURN_LEFT_ALIGN // 7. 左转对齐赛道
};
AvoidanceSubState avoidSubState = AVOID_NONE;
unsigned long avoidStateStartTime = 0;
float avoidStateStartDistance = 0;
float avoidStartLeftDist = 0;
float avoidStartRightDist = 0;

// 避障后状态变量
unsigned long avoidanceFinishTime = 0; // 避障完成时间
bool postAvoidanceStable = false;      // 避障后是否已稳定(持续1秒识线)

// 停车子状态
enum ParkingSubState {
    PARK_APPROACH,      // 接近 (减速)
    PARK_VERY_SLOW,     // 极慢速
    PARK_STOP,          // 停止
    PARK_ALARM          // 报警
};
ParkingSubState parkingSubState = PARK_APPROACH;
unsigned long parkingStateStartTime = 0;

// 测试模式状态
enum TestSubState {
    TEST_NONE,
    TEST_TURN_90,
    TEST_STRAIGHT_1M
};
TestSubState currentTestState = TEST_NONE;
unsigned long testStartTime = 0;

// 按键状态机变量
unsigned long buttonPressStart = 0;
bool buttonWasPressed = false;
bool buttonProcessed = false;  // 防止重复触发

// 手动控制变量
bool manualControlActive = false;
unsigned long manualControlEndTime = 0;

// 循迹统计变量
unsigned long lineFollowStartTime = 0;
unsigned long totalLineFollowTime = 0;
uint32_t loopCounter = 0;
unsigned long lastStatsTime = 0;

// 状态回调函数
String getSystemStatus() {
    JsonDocument doc;
    
    // 系统状态
    const char* stateNames[] = {"IDLE", "LINE_FOLLOW", "OBSTACLE_AVOID", "PARKING", "FINISHED", "TESTING"};
    if (currentState >= 0 && currentState < sizeof(stateNames)/sizeof(stateNames[0])) {
        doc["state"] = stateNames[currentState];
    } else {
        doc["state"] = "UNKNOWN";
    }
    doc["uptime"] = millis() / 1000;
    doc["running"] = systemRunning;
    doc["loopFreq"] = loopCounter;  // 循环频率
    
    // 传感器数据
    JsonObject sensor = doc["sensor"].to<JsonObject>();
    sensor["linePos"] = lineSensor.getLinePosition();
    sensor["lineStates"] = lineSensor.getRawStates();
    sensor["dataReady"] = lineSensor.isDataReady();
    sensor["lostLine"] = lineSensor.isLostLine();
    sensor["laserDist"] = sensors.getLaserDistance();
    sensor["laserReady"] = sensors.isLaserReady();
    sensor["ultraDist"] = sensors.getUltrasonicDistance();
    
    // 电机数据
    JsonObject mot = doc["motor"].to<JsonObject>();
    mot["speedL"] = motor.getLeftSpeed();
    mot["speedR"] = motor.getRightSpeed();
    mot["distL"] = motor.getLeftDistance();
    mot["distR"] = motor.getRightDistance();
    mot["encL"] = motor.getLeftEncoder();
    mot["encR"] = motor.getRightEncoder();
    
    // PID调试数据
    JsonObject pid = doc["pid"].to<JsonObject>();
    pid["pTerm"] = pidController.getP();
    pid["iTerm"] = pidController.getI();
    pid["dTerm"] = pidController.getD();
    pid["error"] = pidController.getError();
    
    // 运行统计
    doc["totalTime"] = totalLineFollowTime / 1000;
    
    // 编码器调试信息
    JsonObject encDebug = doc["encDebug"].to<JsonObject>();
    encDebug["left"] = motor.getLeftDistance();
    encDebug["right"] = motor.getRightDistance();
    encDebug["diff"] = motor.getLeftDistance() - motor.getRightDistance();
    
    // 物块检测状态
    JsonObject detection = doc["detection"].to<JsonObject>();
    detection["active"] = objectDetector.isDetecting();
    detection["completed"] = objectDetector.isCompleted();
    if (objectDetector.isCompleted()) {
        ObjectMeasurement result = objectDetector.getResult();
        detection["length"] = result.length;
        detection["avgDist"] = result.avgDistance;
        detection["valid"] = result.valid;
        detection["duration"] = result.duration; // 新增：检测持续时间
        // 传递原始长度，避免前端反向计算误差
        float rawLen = (result.endPos - result.startPos);
        detection["rawLength"] = rawLen;
    }
    
    // 任务管理状态
    JsonObject tasks = doc["tasks"].to<JsonObject>();
    tasks["executing"] = taskManager.isExecuting();
    tasks["current"] = taskManager.getCurrentTaskIndex();
    tasks["total"] = taskManager.getTotalTasks();
    
    String output;
    serializeJson(doc, output);
    return output;
}

// 任务执行器 - 启动任务
bool executeTask(Task* task) {
    if (!task) return false;
    
    switch (task->type) {
        case TASK_LINE_FOLLOW:
            // 开启循迹模式
            systemRunning = true;
            currentState = STATE_LINE_FOLLOW;
            return true;
            
        case TASK_MEASURE_OBJECT:
            // 启动物块测量
            objectDetector.startDetection(
                task->params.laserBaseline, 
                task->params.laserThreshold
            );
            return true;
            
        case TASK_FORWARD:
            // 前进指定距离
            motor.resetEncoders();
            motor.setBothSpeed(task->params.speed > 0 ? task->params.speed : params.speedNormal);
            return true;
            
        case TASK_STOP:
            // 停止
            systemRunning = false;
            motor.stop();
            return true;
            
        case TASK_DELAY:
            // 延时（通过startTime判断）
            return true;
            
        case TASK_BEEP:
            // 蜂鸣
            // sensors.beep(task->params.duration > 0 ? task->params.duration : 100);
            return true;
            
        default:
            Serial.printf("⚠ Unknown task type: %d\n", task->type);
            return false;
    }
}

// 任务完成检查器
bool checkTaskCompletion(Task* task) {
    if (!task) return true;
    
    switch (task->type) {
        case TASK_LINE_FOLLOW:
            // 循迹任务需要手动停止或达到距离
            if (task->params.distance > 0) {
                float avgDist = motor.getAverageDistance();
                return avgDist >= task->params.distance;
            }
            return false;  // 无限循迹，需要其他条件停止
            
        case TASK_MEASURE_OBJECT:
            // 检查物块测量是否完成
            return objectDetector.isCompleted() || 
                   (millis() - task->startTime > 30000);  // 30秒超时
            
        case TASK_FORWARD:
            // 检查是否达到目标距离
            if (task->params.distance > 0) {
                float avgDist = motor.getAverageDistance();
                if (avgDist >= task->params.distance) {
                    motor.stop();
                    return true;
                }
            } else if (task->params.duration > 0) {
                // 按时间前进
                if (millis() - task->startTime >= task->params.duration) {
                    motor.stop();
                    return true;
                }
            }
            return false;
            
        case TASK_STOP:
            return true;  // 立即完成
            
        case TASK_DELAY:
            return (millis() - task->startTime) >= task->params.duration;
            
        case TASK_BEEP:
            return true;  // 立即完成
            
        default:
            return true;
    }
}

// 运动控制回调 (仅设置标志位，不在中断/异步任务中执行逻辑)
void handleMotionCommand(String action, float value) {
    // 映射字符串命令到枚举，确保原子操作
    if (action == "stop") pendingManualCmd = CMD_STOP;
    else if (action == "forward") pendingManualCmd = CMD_FORWARD;
    else if (action == "backward") pendingManualCmd = CMD_BACKWARD;
    else if (action == "left") pendingManualCmd = CMD_LEFT;
    else if (action == "right") pendingManualCmd = CMD_RIGHT;
    else if (action == "turn_180") pendingManualCmd = CMD_TURN_180;
    
    pendingManualValue = value;
}

// 处理挂起的命令 (在主循环中调用)
void processPendingCommands() {
    // 1. 处理测试命令
    if (pendingTestTurn) {
        pendingTestTurn = false;
        if (!systemRunning) {
            Serial.println("CMD: Starting Turn 90 Test");
            currentState = STATE_TESTING;
            currentTestState = TEST_TURN_90;
            testStartTime = millis();
            motor.resetEncoders();
            systemRunning = true;
        }
    }
    
    if (pendingTestStraight) {
        pendingTestStraight = false;
        if (!systemRunning) {
            Serial.println("CMD: Starting Straight 1m Test");
            currentState = STATE_TESTING;
            currentTestState = TEST_STRAIGHT_1M;
            testStartTime = millis();
            motor.resetEncoders();
            encoderPid.reset();
            encoderPid.setGains(params.encKp, params.encKi, params.encKd);
            systemRunning = true;
        }
    }
    
    if (pendingTestAvoid) {
        pendingTestAvoid = false;
        if (!systemRunning) {
            Serial.println("CMD: Starting Avoidance Test");
            // 直接进入避障状态
            currentState = STATE_OBSTACLE_AVOID;
            avoidSubState = AVOID_TURN_LEFT;
            avoidStateStartTime = millis();
            motor.resetEncoders();
            avoidStartLeftDist = 0;
            avoidStartRightDist = 0;
            avoidStateStartDistance = 0;
            systemRunning = true;
            // sensors.beep(100);
        }
    }

    if (pendingTestParking) {
        pendingTestParking = false;
        Serial.println("CMD: Starting Parking Test");
        
        // 重置系统状态
        systemRunning = true;
        currentState = STATE_LINE_FOLLOW;
        lineFollowStartTime = millis();
        motor.resetEncoders();
        pidController.reset();
        
        // 设置为入库测试模式
        objectDetector.stopDetection(); // 确保不处于物块检测模式
        obstacleDetectionEnabled = true; // 启用障碍物检测
        obstacleDetectCount = 1; // 假装已经避过第一个障碍物，下一个就是车库
        
        // sensors.beep(100);
        // delay(50);
        // sensors.beep(100);
        display.showDebug("TEST PARKING\nSearching...");
    }
    
    // 2. 处理手动控制命令
    if (pendingManualCmd != CMD_NONE) {
        ManualCommand cmd = pendingManualCmd;
        float val = pendingManualValue;
        pendingManualCmd = CMD_NONE; // 清除标志
        
        // 执行逻辑
        if (cmd == CMD_STOP) {
            motor.stop();
            manualControlActive = false;
            if (systemRunning) Serial.println("Manual Stop");
        } else {
            // 激活手动控制
            if (systemRunning) {
                Serial.println("Auto mode paused for manual control");
            }
            manualControlActive = true;
            
            int moveSpeed = (val > 0) ? (int)val : params.speedNormal;
            int turnSpeed = params.speedTurn;
            
            switch (cmd) {
                case CMD_FORWARD:
                    motor.setBothSpeed(moveSpeed);
                    manualControlEndTime = millis() + 10000;
                    break;
                case CMD_BACKWARD:
                    motor.setBothSpeed(-moveSpeed);
                    manualControlEndTime = millis() + 10000;
                    break;
                case CMD_LEFT:
                    motor.setLeftSpeed(-turnSpeed);
                    motor.setRightSpeed(turnSpeed);
                    manualControlEndTime = millis() + 10000;
                    break;
                case CMD_RIGHT:
                    motor.setLeftSpeed(turnSpeed);
                    motor.setRightSpeed(-turnSpeed);
                    manualControlEndTime = millis() + 10000;
                    break;
                case CMD_TURN_180:
                    motor.setLeftSpeed(turnSpeed);
                    motor.setRightSpeed(-turnSpeed);
                    manualControlEndTime = millis() + 1200;
                    break;
                default: break;
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n=== Smart Car Line Follower System ===");
    Serial.println("Version: 2.0.0 - PID Line Follow Only");
    
    // 初始化显示
    display.begin();
    display.showDebug("Initializing...");
    delay(200);
    
    // 初始化参数管理器
    display.showDebug("Loading params...");
    params.begin();
    delay(100);
    
    // 初始化Web服务器
    display.showDebug("Starting WiFi...");
    webServer.begin();
    
    // 设置ObjectDetector的WebServer引用用于日志输出
    objectDetector.setWebServer(&webServer);
    // 设置偏差修正系数 (每单位偏差减少的距离比例, 1000偏差约对应15%距离损失)
    objectDetector.setDeviationCorrection(params.objectDeviationCorrection); 
    
    // webServer.setStatusCallback(getSystemStatus); // Removed: Now using push model in loop
    webServer.setMotionCallback(handleMotionCommand);
    webServer.setWeightCallback([](int16_t weights[8]) {
        lineSensor.setWeights(weights);
        Serial.println("✓ Weights updated from web");
    });
    webServer.setCalibrationCallback([](float leftCalib, float rightCalib) {
        params.motorLeftCalib = leftCalib;
        params.motorRightCalib = rightCalib;
        motor.setCalibration(leftCalib, rightCalib);
        params.save();
        Serial.printf("✓ Motor calibration updated: L=%.3f R=%.3f\n", leftCalib, rightCalib);
    });
    webServer.setDetectionCallback([](uint16_t baseline, uint16_t threshold) {
        if (baseline == 0 && threshold == 0) {
            // 停止检测
            objectDetector.stopDetection();
            webServer.addLog("✓ Object detection stopped");
        } else {
            // 更新参数
            objectDetector.setFilterSize(params.objectFilterSize);
            objectDetector.setCorrection(params.objectLengthScale, params.objectLengthOffset);
            objectDetector.setDeviationCorrection(params.objectDeviationCorrection);
            
            // 开始检测
            objectDetector.startDetection(baseline, threshold);
            webServer.addLog("✓ Object detection started: range<" + String(threshold) + "mm");
        }
    });
    webServer.setTaskCallback([](String action, String data) -> String {
        if (action == "get") {
            return taskManager.getTasksJson();
        } else if (action == "set") {
            taskManager.loadTasksFromJson(data);
            return "{\"status\":\"ok\"}";
        } else if (action == "start") {
            taskManager.startExecution();
            return "{\"status\":\"ok\"}";
        } else if (action == "stop") {
            taskManager.stopExecution();
            return "{\"status\":\"ok\"}";
        } else if (action == "clear") {
            taskManager.clearAllTasks();
            return "{\"status\":\"ok\"}";
        } else if (action == "test_turn") {
            // 测试90度转弯 (仅设置标志，避免并发崩溃)
            pendingTestTurn = true;
            return "{\"status\":\"ok\", \"msg\":\"Command queued\"}";
        } else if (action == "test_straight") {
            // 测试直线行驶1米 (仅设置标志)
            pendingTestStraight = true;
            return "{\"status\":\"ok\", \"msg\":\"Command queued\"}";
        } else if (action == "test_avoid") {
            // 测试避障流程
            pendingTestAvoid = true;
            return "{\"status\":\"ok\", \"msg\":\"Command queued\"}";
        } else if (action == "test_parking") {
            // 测试入库流程
            pendingTestParking = true;
            return "{\"status\":\"ok\", \"msg\":\"Command queued\"}";
        }
        return "{\"status\":\"error\"}";
    });
    delay(500);
    
    // 显示WiFi信息
    String wifiInfo = "WiFi: " + String(WIFI_AP_SSID) + "\nIP: " + webServer.getIPAddress();
    display.showDebug(wifiInfo);
    Serial.println(wifiInfo);
    delay(2000);
    
    // 初始化传感器
    display.showDebug("Init sensors...");
    sensors.begin();
    delay(100);
    
    // 测试激光传感器
    Serial.println("Testing VL53L0X laser sensor...");
    for(int i = 0; i < 5; i++) {
        sensors.update();
        Serial.printf("  Test %d: Distance=%dmm, Ready=%d\n", 
            i+1, sensors.getLaserDistance(), sensors.isLaserReady());
        delay(100);
    }
    
    display.showDebug("Init line sensor...");
    lineSensor.begin();
    // 应用保存的传感器权重
    lineSensor.setWeights(params.sensorWeights);
    delay(100);
    
    // 测试循迹传感器通信
    Serial.println("Testing line sensor communication...");
    for(int i = 0; i < 5; i++) {
        lineSensor.update();
        Serial.printf("  Test %d: States=0x%02X, Ready=%d\n", 
            i+1, lineSensor.getRawStates(), lineSensor.isDataReady());
        delay(100);
    }
    
    // 初始化电机
    display.showDebug("Init motors...");
    motor.begin();
    motor.setCalibration(params.motorLeftCalib, params.motorRightCalib);
    motor.setDeadband(params.motorDeadband); // 设置死区
    motor.stop();
    delay(100);
    
    // 初始化PID控制器
    pidController.setGains(params.kp, params.ki, params.kd);
    pidController.setSetpoint(0);  // 目标位置为中心
    pidController.setIntegralRange(params.pidIntegralRange); // 设置积分分离
    pidController.setOutputLimits(-255, 255);
    
    // 初始化编码器PID
    encoderPid.setGains(params.encKp, params.encKi, params.encKd);
    encoderPid.setSetpoint(0); // 目标差值为0
    encoderPid.setOutputLimits(-50, 50); // 限制修正量
    
    // 初始化任务管理器
    taskManager.setTaskExecutor(executeTask);
    taskManager.setTaskChecker(checkTaskCompletion);
    
    Serial.println("✓ System initialized!");
    Serial.println("✓ Web interface: http://" + webServer.getIPAddress());
    Serial.println("✓ Press button to start/stop line following");
    display.showStartup();
    
    currentState = STATE_IDLE;
    systemRunning = false;
    loopCounter = 0;
    lastStatsTime = millis();
}

// 障碍物避障处理
void handleObstacleAvoidance() {
    unsigned long stepDuration = millis() - avoidStateStartTime;
    int turnSpeed = params.avoidTurnSpeed;
    int forwardSpeed = params.avoidSpeed;
    
    // 获取当前编码器距离
    float currentLeft = motor.getLeftDistance();
    float currentRight = motor.getRightDistance();
    
    // 计算自状态开始以来的增量
    float deltaLeft = currentLeft - avoidStartLeftDist;
    float deltaRight = currentRight - avoidStartRightDist;
    
    switch (avoidSubState) {
        case AVOID_TURN_LEFT:
            // 1. 左转90度离开赛道
            motor.setLeftSpeed(-turnSpeed * params.avoidS1_L);
            motor.setRightSpeed(turnSpeed * params.avoidS1_R);
            
            if (abs(deltaLeft) >= params.avoidTurn1Dist || abs(deltaRight) >= params.avoidTurn1Dist) {
                motor.brake(); delay(200); motor.stop();
                Serial.printf("✓ Step 1: Left turn done. L:%.1f R:%.1f\n", deltaLeft, deltaRight);
                
                avoidSubState = AVOID_FORWARD_OUT;
                avoidStateStartTime = millis();
                motor.resetEncoders();
                avoidStartLeftDist = 0;
                avoidStartRightDist = 0;
                // sensors.beep(50);
            }
            break;
            
        case AVOID_FORWARD_OUT:
            // 2. 直行离开赛道 (距离由网页配置 avoidForwardDist)
            {
                // 简单P控制修正万向轮拖拽导致的偏航
                // 万向轮横置时会产生巨大阻力，导致启动时偏向一边
                float error = deltaLeft - deltaRight;
                int adjustment = (int)(error * params.avoidKp); // 使用配置的Kp修正
                
                motor.setLeftSpeed((forwardSpeed * params.avoidS2_L) - adjustment);
                motor.setRightSpeed((forwardSpeed * params.avoidS2_R) + adjustment);
                
                float avgDist = (deltaLeft + deltaRight) / 2.0;
                // 使用配置的距离
                if (avgDist >= params.avoidForwardDist) {
                    motor.brake(); delay(200); motor.stop();
                    Serial.printf("✓ Step 2: Forward OUT done. Dist:%.1f\n", avgDist);
                    
                    avoidSubState = AVOID_TURN_RIGHT_1;
                    avoidStateStartTime = millis();
                    motor.resetEncoders();
                    avoidStartLeftDist = 0;
                    avoidStartRightDist = 0;
                    // sensors.beep(50);
                }
            }
            break;
            
        case AVOID_TURN_RIGHT_1:
            // 3. 右转90度 (平行于赛道)
            motor.setLeftSpeed(turnSpeed * params.avoidS3_L);
            motor.setRightSpeed(-turnSpeed * params.avoidS3_R);
            
            if (abs(deltaLeft) >= params.avoidTurn2Dist || abs(deltaRight) >= params.avoidTurn2Dist) {
                motor.brake(); delay(200); motor.stop();
                Serial.printf("✓ Step 3: Right turn 1 done.\n");
                
                avoidSubState = AVOID_FORWARD_PARALLEL;
                avoidStateStartTime = millis();
                motor.resetEncoders();
                avoidStartLeftDist = 0;
                avoidStartRightDist = 0;
                // sensors.beep(50);
            }
            break;
            
        case AVOID_FORWARD_PARALLEL:
            // 4. 直行 (平行移动，绕过障碍物)
            // 距离通常需要大于障碍物长度，这里暂时复用 avoidForwardDist 或固定值
            // 假设障碍物长度约30cm，给50cm余量
            {
                // 简单P控制修正万向轮拖拽
                float error = deltaLeft - deltaRight;
                int adjustment = (int)(error * params.avoidKp);
                
                motor.setLeftSpeed((forwardSpeed * params.avoidS4_L) - adjustment);
                motor.setRightSpeed((forwardSpeed * params.avoidS4_R) + adjustment);
                
                float avgDist = (deltaLeft + deltaRight) / 2.0;
                // 使用配置的距离
                if (avgDist >= params.avoidParallelDist) { 
                    motor.brake(); delay(200); motor.stop();
                    Serial.printf("✓ Step 4: Parallel move done. Dist:%.1f\n", avgDist);
                    
                    avoidSubState = AVOID_TURN_RIGHT_2;
                    avoidStateStartTime = millis();
                    motor.resetEncoders();
                    avoidStartLeftDist = 0;
                    avoidStartRightDist = 0;
                    // sensors.beep(50);
                }
            }
            break;
            
        case AVOID_TURN_RIGHT_2:
            // 5. 右转90度 (面向赛道)
            motor.setLeftSpeed(turnSpeed * params.avoidS5_L);
            motor.setRightSpeed(-turnSpeed * params.avoidS5_R);
            
            if (abs(deltaLeft) >= params.avoidTurn3Dist || abs(deltaRight) >= params.avoidTurn3Dist) {
                motor.brake(); delay(200); motor.stop();
                Serial.printf("✓ Step 5: Right turn 2 done.\n");
                
                avoidSubState = AVOID_FORWARD_IN;
                avoidStateStartTime = millis();
                motor.resetEncoders();
                avoidStartLeftDist = 0;
                avoidStartRightDist = 0;
                // sensors.beep(50);
            }
            break;
            
        case AVOID_FORWARD_IN:
            // 6. 直行寻找黑线
            {
                // 慢速前进寻找，同样加入修正
                float error = deltaLeft - deltaRight;
                int adjustment = (int)(error * params.avoidKp);
                
                int searchSpeed = params.speedSlow;
                motor.setLeftSpeed((searchSpeed * params.avoidS6_L) - adjustment);
                motor.setRightSpeed((searchSpeed * params.avoidS6_R) + adjustment);
                
                // 检测是否找到黑线 (直接检查原始状态，不依赖isLostLine的状态更新)
                // 只要有任意一个传感器检测到黑线(状态不为0)，即认为找到线
                if (lineSensor.isDataReady() && lineSensor.getRawStates() != 0) {
                    motor.brake(); delay(200); motor.stop();
                    Serial.println("✓ Step 6: Line found!");
                    
                    avoidSubState = AVOID_TURN_LEFT_ALIGN;
                    avoidStateStartTime = millis();
                    motor.resetEncoders();
                    avoidStartLeftDist = 0;
                    avoidStartRightDist = 0;
                    // sensors.beep(100);
                } 
                // 超时或距离过长保护
                else if (motor.getAverageDistance() >= params.avoidSearchDist) {
                    Serial.println("⚠ Line not found, forcing align");
                    avoidSubState = AVOID_TURN_LEFT_ALIGN; // 强制进入下一步
                    motor.resetEncoders();
                    avoidStartLeftDist = 0;
                    avoidStartRightDist = 0;
                }
            }
            break;
            
        case AVOID_TURN_LEFT_ALIGN:
            // 7. 左转90度对齐赛道
            motor.setLeftSpeed(-turnSpeed);
            motor.setRightSpeed(turnSpeed);
            
            if (abs(deltaLeft) >= params.avoidFinalTurnDist || abs(deltaRight) >= params.avoidFinalTurnDist) {
                motor.brake(); delay(200); motor.stop();
                Serial.println("✓ Step 7: Align done, resuming line follow");
                
                currentState = STATE_LINE_FOLLOW;
                avoidSubState = AVOID_NONE;
                pidController.reset();
                
                // 记录避障完成时间，开始监测稳定性
                avoidanceFinishTime = millis();
                postAvoidanceStable = false;
                
                // sensors.beep(100);
                // delay(50);
                // sensors.beep(100);
            }
            break;
            
        default:
            break;
    }
    
    // 超时保护
    if (millis() - avoidStateStartTime > AVOID_TIME_MS) {
        Serial.println("⚠ Avoidance timeout, returning to line follow");
        currentState = STATE_LINE_FOLLOW;
        avoidSubState = AVOID_NONE;
        motor.stop();
    }
}

// 入库停车处理
void handleParking() {
    float ultraDist = sensors.getUltrasonicDistance();
    
    // 简单的P控制保持直线 (使用编码器)
    // 目标是左右轮走过的距离相等
    float error = motor.getLeftDistance() - motor.getRightDistance();
    int adjustment = (int)(error * params.encKp); // 使用编码器PID参数或固定Kp
    
    switch (parkingSubState) {
        case PARK_APPROACH:
            // 阶段1: 接近车库
            // 如果距离还很远(>减速距离)，可以用稍快一点的速度(如speedSlow)
            // 如果距离进入减速范围(<减速距离)，用parkingSpeedSlow
            
            if (ultraDist > params.parkingDistSlow) {
                // 还没到减速区，保持慢速接近
                motor.setLeftSpeed(params.speedSlow - adjustment);
                motor.setRightSpeed(params.speedSlow + adjustment);
            } else {
                // 进入减速区
                motor.setLeftSpeed(params.parkingSpeedSlow - adjustment);
                motor.setRightSpeed(params.parkingSpeedSlow + adjustment);
                
                // 检查是否进入极慢速区
                if (ultraDist <= params.parkingDistVerySlow) {
                    Serial.printf("✓ Parking: Entering Very Slow Zone (Dist: %.1fcm)\n", ultraDist);
                    parkingSubState = PARK_VERY_SLOW;
                }
            }
            break;
            
        case PARK_VERY_SLOW:
            // 阶段2: 极慢速靠近
            motor.setLeftSpeed(params.parkingSpeedVerySlow - adjustment);
            motor.setRightSpeed(params.parkingSpeedVerySlow + adjustment);
            
            // 检查是否到达停止距离
            if (ultraDist <= params.parkingDistStop) {
                Serial.printf("✓ Parking: Stop Distance Reached (Dist: %.1fcm)\n", ultraDist);
                motor.brake();
                delay(200);
                motor.stop();
                
                parkingSubState = PARK_STOP;
                parkingStateStartTime = millis();
            }
            break;
            
        case PARK_STOP:
            // 阶段3: 确认停止
            motor.stop();
            parkingSubState = PARK_ALARM;
            parkingStateStartTime = millis();
            Serial.println("✓ Parking: Stopped, Alarm starting...");
            break;
            
        case PARK_ALARM:
            // 阶段4: 报警3秒
            sensors.setAlarm(true);
            
            if (millis() - parkingStateStartTime >= 3000) {
                sensors.setAlarm(false);
                Serial.println("✓ Parking completed!");
                currentState = STATE_FINISHED;
                systemRunning = false;
            }
            break;
    }
    
    // 调试输出 (每500ms)
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500) {
        Serial.printf("[Parking] State:%d Dist:%.1fcm\n", parkingSubState, ultraDist);
        lastDebug = millis();
    }
}

void updateSensors() {
    lineSensor.update();
    sensors.update();  // 更新激光等传感器
    motor.update();
    
    // 更新物块检测器（如果正在检测）
    if (objectDetector.isDetecting()) {
        objectDetector.update(lineSensor.getLinePosition());
        
        // 如果检测完成，启用障碍物检测
        if (objectDetector.isCompleted() && !obstacleDetectionEnabled) {
            obstacleDetectionEnabled = true;
            Serial.println("✓ Object measurement completed, obstacle detection enabled");
            // sensors.beep(50);
            // delay(50);
            // sensors.beep(50);
        }
    }
    
    // 更新循环计数器（用于监控频率）
    loopCounter++;
    
    // 每秒重置计数器
    if (millis() - lastStatsTime >= 1000) {
        lastStatsTime = millis();
        loopCounter = 0;
    }
}

// PID循迹控制
void lineFollowControl() {
    static bool wasLost = false; // 记录上次是否丢线

    // 避障后稳定性检测逻辑
    if (avoidanceFinishTime > 0 && !postAvoidanceStable) {
        if (lineSensor.isLostLine()) {
            // 如果在稳定期内丢线，重置计时器
            avoidanceFinishTime = millis();
        } else {
            // 持续识线超过1秒
            if (millis() - avoidanceFinishTime > 1000) {
                postAvoidanceStable = true;
                Serial.println("✓ Post-avoidance stability achieved: Lost line -> Straight mode enabled");
            }
        }
    }

    // 检查数据就绪
    if (!lineSensor.isDataReady()) {
        Serial.println("⚠ Line sensor data not ready!");
        motor.stop();
        return;
    }
    
    // 障碍物检测（物块测量完成后启用）
    if (obstacleDetectionEnabled && obstacleDetectCount < 2) {
        float ultraDist = sensors.getUltrasonicDistance();
        
        // 检测到障碍物
        if (ultraDist < params.obstacleDetectDist && ultraDist > 2.0) {
            obstacleDetectCount++;
            Serial.printf("\n🚧 Obstacle %d detected! Distance: %.1fcm\n", obstacleDetectCount, ultraDist);
            
            if (obstacleDetectCount == 1) {
                // 第一次：执行避障
                Serial.println("=== Starting Obstacle Avoidance ===");
                
                // 立即停车，防止冲向障碍物
                motor.brake();
                delay(500);
                motor.stop();
                
                currentState = STATE_OBSTACLE_AVOID;
                avoidSubState = AVOID_TURN_LEFT;
                avoidStateStartTime = millis();
                motor.resetEncoders();
                avoidStartLeftDist = 0;
                avoidStartRightDist = 0;
                avoidStateStartDistance = 0;
                // sensors.beep(100); // 短促提示音
                return;
            } else if (obstacleDetectCount == 2) {
                // 第二次：执行入库停车
                Serial.println("=== Starting Parking Procedure ===");
                currentState = STATE_PARKING;
                parkingSubState = PARK_APPROACH;
                parkingStateStartTime = millis();
                motor.resetEncoders();
                // sensors.beep(200); // 长提示音
                return;
            }
        }
    }
    
    // 获取线位置 (-1000 到 +1000)
    int16_t linePosition = lineSensor.getLinePosition();
    
    // 检测丢线
    if (lineSensor.isLostLine()) {
        if (!wasLost) {
            Serial.println("⚠ Line lost! Searching...");
            wasLost = true;
        }
        
        // 特殊逻辑：如果避障后已经稳定行驶过1秒，丢线后直接走直线
        if (postAvoidanceStable) {
            motor.setBothSpeed(params.speedSlow); // 使用慢速直行
            return;
        }

        // 丢线时减速搜索
        int searchSpeed = params.speedSlow;
        int16_t lastPos = lineSensor.getLastPosition();
        
        // 修改：去除直行搜索，总是旋转搜索
        if (lastPos >= 0) {
            // 上次在右边或中间，右转搜索
            motor.setLeftSpeed(searchSpeed);
            motor.setRightSpeed(searchSpeed / 3);
        } else {
            // 上次在左边，左转搜索
            motor.setLeftSpeed(searchSpeed / 3);
            motor.setRightSpeed(searchSpeed);
        }
        return;
    }
    
    // 如果刚找回线，重置PID
    if (wasLost) {
        Serial.println("✓ Line found! Resetting PID...");
        pidController.reset();
        wasLost = false;
        // 找回线时短暂蜂鸣提示
        // sensors.beep(50); 
    }
    
    // 更新PID参数（支持Web实时调整）
    // 动态PID策略：直线稳，弯道狠
    float effectiveKp, effectiveKi, effectiveKd;
    int currentSpeedNormal, currentSpeedFast, currentSpeedTurn;
    
    // 根据物块检测状态选择参数组
    if (objectDetector.isCompleted()) {
        // Phase 2: 测距完成后
        effectiveKp = params.kpPost;
        effectiveKi = params.kiPost;
        effectiveKd = params.kdPost;
        currentSpeedNormal = params.speedNormalPost;
        currentSpeedFast = params.speedFastPost;
        currentSpeedTurn = params.speedTurnPost;
    } else {
        // Phase 1: 测距前及测距中
        effectiveKp = params.kp;
        effectiveKi = params.ki;
        effectiveKd = params.kd;
        currentSpeedNormal = params.speedNormal;
        currentSpeedFast = params.speedFast;
        currentSpeedTurn = params.speedTurn;
    }
    
    // 特殊模式：物块测量时需要极高的直线稳定性
    if (objectDetector.isDetecting()) {
        // 测量模式：强力维持直线，防止蛇形走位导致里程偏大
        effectiveKp *= 2.5; // 大幅增加Kp，快速纠偏
        effectiveKd *= 3.0; // 大幅增加Kd，强力阻尼防止震荡
        // 此时不使用小误差缩放，保持全程高刚性
    } else {
        // 普通模式
        // 如果误差很小（在直线上）
        if (abs(linePosition) < params.pidSmallErrorThres) {
            effectiveKp *= params.pidKpSmallScale; // 降低比例作用，减少高频抖动
            effectiveKd *= params.pidKdSmallScale; // 增加微分阻尼，防止微小超调
        }
    }
    
    pidController.setGains(effectiveKp, effectiveKi, effectiveKd);
    pidController.setIntegralRange(params.pidIntegralRange); // 实时更新积分分离阈值
    motor.setDeadband(params.motorDeadband); // 实时更新死区
    
    // PID计算差速
    float pidOutput = pidController.compute(linePosition);
    
    // 基础速度
    int baseSpeed = currentSpeedNormal;
    
    // 优化：基于误差的连续动态速度调整
    // 误差越大，速度越慢。使用二次曲线使直道更快，弯道更稳
    float errorRatio = constrain(abs(linePosition) / 1000.0f, 0.0f, 1.0f);
    
    // 动态速度公式: Base = Min + (Max - Min) * (1 - ratio^2)
    // ratio=0(直道) -> MaxSpeed
    // ratio=1(急弯) -> MinSpeed
    int maxSpeed = currentSpeedFast;
    int minSpeed = currentSpeedTurn; // 转弯速度作为下限
    
    baseSpeed = minSpeed + (int)((maxSpeed - minSpeed) * (1.0f - errorRatio * errorRatio));
    
    // 极端情况处理：如果误差极大(>800)，强制使用更低的速度
    if (abs(linePosition) > 800) {
        baseSpeed = params.speedSlow;
    }
    
    // 计算左右轮速度
    int leftSpeed = baseSpeed - pidOutput;
    int rightSpeed = baseSpeed + pidOutput;
    
    // 改进：差速过大时，允许内侧轮反转(原地转向辅助)以获得更小的转弯半径
    // 但限制反转速度，防止突然掉头
    // leftSpeed = constrain(leftSpeed, -100, 255);
    // rightSpeed = constrain(rightSpeed, -100, 255);
    
    // 目前保持正转逻辑，仅限幅
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    // 设置电机
    motor.setLeftSpeed(leftSpeed);
    motor.setRightSpeed(rightSpeed);
    
    // 调试输出
#if DEBUG_PID
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 200) {  // 每200ms输出一次
        Serial.printf("Pos:%5d | P:%6.1f I:%6.1f D:%6.1f | Out:%6.1f | L:%4d R:%4d\n",
            linePosition, 
            pidController.getP(), 
            pidController.getI(), 
            pidController.getD(),
            pidOutput,
            leftSpeed, 
            rightSpeed);
        lastDebugTime = millis();
    }
#endif
}

// 测试模式处理
void handleTestMode() {
    unsigned long stepDuration = millis() - testStartTime;
    int turnSpeed = params.avoidTurnSpeed;
    int forwardSpeed = params.avoidSpeed;
    
    // 获取当前编码器距离
    float currentLeft = motor.getLeftDistance();
    float currentRight = motor.getRightDistance();
    
    switch (currentTestState) {
        case TEST_TURN_90:
            {
                float target = params.turn90Dist;
                float current = max(abs(currentLeft), abs(currentRight));
                float remaining = target - current;
                
                int currentSpeed = turnSpeed;
                
                // 减速逻辑：剩余距离小于40%或50mm时开始减速
                // 避免速度过快导致过冲或打滑
                float slowDownThres = max(target * 0.4f, 50.0f);
                
                if (remaining < slowDownThres) {
                    // 线性减速至最低启动速度 (防止停转)
                    // 修复: 提高转弯时的最低速度，防止在接近目标时因阻力过大而停转导致超时
                    int minSpeed = max(100, params.motorDeadband + 50); 
                    float ratio = remaining / slowDownThres; // 1.0 -> 0.0
                    
                    currentSpeed = minSpeed + (int)((turnSpeed - minSpeed) * ratio);
                    currentSpeed = max(currentSpeed, minSpeed);
                }
                
                motor.setLeftSpeed(-currentSpeed);
                motor.setRightSpeed(currentSpeed);
                
                if (current >= target) {
                    motor.brake(); // 执行刹车动作
                    delay(300);    // 保持刹车300ms以完全停止
                    motor.stop();
                    
                    Serial.printf("TEST: Turn 90 done. L:%.1f R:%.1f\n", currentLeft, currentRight);
                    currentState = STATE_IDLE;
                    currentTestState = TEST_NONE;
                    systemRunning = false;
                    // sensors.beep(200);
                }
            }
            break;
            
        case TEST_STRAIGHT_1M:
            {
                // 简单的P控制保持直线
                float error = currentLeft - currentRight;
                float adjustment = encoderPid.compute(error);
                
                int leftSpd = forwardSpeed - adjustment;
                int rightSpd = forwardSpeed + adjustment;
                
                motor.setLeftSpeed(leftSpd);
                motor.setRightSpeed(rightSpd);
                
                float avgDist = (currentLeft + currentRight) / 2.0;
                if (avgDist >= 1000) { // 测试走1米
                    motor.stop();
                    Serial.printf("TEST: Straight 1m done. Err:%.1f\n", error);
                    currentState = STATE_IDLE;
                    currentTestState = TEST_NONE;
                    systemRunning = false;
                    // sensors.beep(200);
                }
            }
            break;
            
        default:
            motor.stop();
            currentState = STATE_IDLE;
            break;
    }
    
    // 超时保护 (10秒)
    if (stepDuration > 10000) {
        Serial.println("⚠ Test timeout");
        motor.stop();
        currentState = STATE_IDLE;
        currentTestState = TEST_NONE;
        systemRunning = false;
        // sensors.beep(500);
    }
}

void loop() {
    // 处理Web挂起的命令
    processPendingCommands();

    // 更新传感器数据
    updateSensors();
    
    // 按键状态机 - 改进的防抖和切换逻辑
    bool buttonNow = sensors.isButtonPressed();
    
    if (buttonNow && !buttonWasPressed && !buttonProcessed) {
        // 按钮刚按下
        buttonPressStart = millis();
        buttonWasPressed = true;
    } else if (!buttonNow && buttonWasPressed && !buttonProcessed) {
        // 按钮刚释放
        unsigned long pressDuration = millis() - buttonPressStart;
        
        if (pressDuration >= 50 && pressDuration < 2000) {
            // 有效短按：切换运行状态
            systemRunning = !systemRunning;
            buttonProcessed = true;
            
            if (systemRunning) {
                currentState = STATE_LINE_FOLLOW;
                lineFollowStartTime = millis();
                motor.resetEncoders();
                pidController.reset();  // 重置PID状态
                
                // 重置避障后状态
                avoidanceFinishTime = 0;
                postAvoidanceStable = false;
                
                // 自动启动物块检测
                currentState = STATE_LINE_FOLLOW;
                lineFollowStartTime = millis();
                motor.resetEncoders();
                pidController.reset();  // 重置PID状态
                
                // 自动启动物块检测
                objectDetector.setFilterSize(params.objectFilterSize);
                objectDetector.setCorrection(params.objectLengthScale, params.objectLengthOffset);
                objectDetector.startDetection(0, params.objectDetectDist);
                
                // 重置障碍物检测状态
                obstacleDetectCount = 0;
                obstacleDetectionEnabled = false;  // 等物块测量完成后再启用
                
                display.showDebug("RUNNING\nPress to stop");
            } else {
                // 停止
                Serial.println("\n=== SYSTEM STOP ===");
                // sensors.beep(200);
                
                motor.stop();
                currentState = STATE_IDLE;
                
                // 停止检测
                if (objectDetector.isDetecting()) {
                    objectDetector.stopDetection();
                }
                
                totalLineFollowTime += millis() - lineFollowStartTime;
                
                Serial.printf("✓ Total run time: %lu seconds\n", totalLineFollowTime / 1000);
                display.showDebug("STOPPED\nPress to start");
            }
        } else if (pressDuration >= 2000) {
            // 长按：重置统计
            totalLineFollowTime = 0;
            motor.resetEncoders();
            Serial.println("✓ Statistics reset");
            // sensors.beep(50);
            // delay(100);
            // sensors.beep(50);
            // delay(100);
            // sensors.beep(50);
            buttonProcessed = true;
        }
    } else if (!buttonNow && buttonProcessed) {
        // 按钮完全释放后重置处理标志
        buttonProcessed = false;
        buttonWasPressed = false;
    }
    
    // 手动控制模式处理
    if (manualControlActive) {
        if (millis() >= manualControlEndTime) {
            motor.stop();
            manualControlActive = false;
            Serial.println("✓ Manual control completed");
        }
        delay(10);
        return;  // 手动模式下不执行自动逻辑
    }
    
    // 更新任务管理器
    taskManager.update();

    // 定时更新Web状态 (200ms interval) - 解决并发崩溃问题的关键
    // 将状态生成移至主循环，避免Web任务直接访问共享资源
    static unsigned long lastWebUpdate = 0;
    if (millis() - lastWebUpdate > 200) {
        String status = getSystemStatus();
        webServer.updateStatusJson(status);
        lastWebUpdate = millis();
    }
    
    // 状态机
    switch (currentState) {
        case STATE_IDLE:
            motor.stop();
            break;
            
        case STATE_LINE_FOLLOW:
            if (systemRunning) {
                lineFollowControl();
            } else {
                motor.stop();
            }
            break;
            
        case STATE_OBSTACLE_AVOID:
            handleObstacleAvoidance();
            break;
            
        case STATE_PARKING:
            handleParking();
            break;
            
        case STATE_FINISHED:
            motor.stop();
            break;

        case STATE_TESTING:
            handleTestMode();
            break;
    }
    
    // 更新显示
#if DEBUG_OLED
    static unsigned long lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate > 100) {
        int16_t linePos = lineSensor.getLinePosition();
        uint8_t states = lineSensor.getRawStates();
        
        display.clear();
        Adafruit_SSD1306* oled = display.getDisplay();
        if (oled) {
            oled->setCursor(0, 0);
            oled->setTextSize(1);
            
            // 第一行：状态
            oled->print(systemRunning ? "RUN " : "IDLE");
            oled->printf(" T:%lus\n", (millis() - lineFollowStartTime) / 1000);
            
            // 第二行：传感器状态
            oled->printf("S:0x%02X P:%d\n", states, linePos);
            
            // 第三行：PID输出
            oled->printf("P:%.0f I:%.0f D:%.0f\n", 
                pidController.getP(), 
                pidController.getI(), 
                pidController.getD());
            
            // 第四行：电机速度
            oled->printf("L:%d R:%d\n", 
                (int)motor.getLeftSpeed(), 
                (int)motor.getRightSpeed());
            
            // 分隔线
            oled->drawLine(0, 32, 128, 32, SSD1306_WHITE);
            
            // 如果有测量结果，优先显示结果
            if (objectDetector.getResult().length > 0) {
                oled->setCursor(0, 36);
                oled->setTextSize(1);
                oled->print("Len:");
                oled->setCursor(30, 34); // 稍微调整位置以适应大字体
                oled->setTextSize(3);    // 使用更大的字体
                oled->print((int)objectDetector.getResult().length);
                oled->setTextSize(1);
                oled->print("mm");
            } else {
                // 下半部分：里程和激光距离
                oled->setCursor(0, 36);
                oled->setTextSize(1);
                oled->printf("Dist:%.1fm\n", motor.getAverageDistance() / 1000.0);
                
                // 激光距离
                uint16_t laserDist = sensors.getLaserDistance();
                oled->printf("Laser:%dmm\n", laserDist);
                
                oled->printf("Freq:%dHz", loopCounter);
            }
        }
        
        display.update();
        lastDisplayUpdate = millis();
    }
#endif
    
    // 移除固定延时，全速运行
    // delay(10); 
}
