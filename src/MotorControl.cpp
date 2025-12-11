#include "MotorControl.h"

MotorControl::MotorControl() {
    lastLeftCount = 0;
    lastRightCount = 0;
    lastUpdateTime = 0;
    leftSpeed = 0;
    rightSpeed = 0;
    leftCalib = 1.0;
    rightCalib = 1.0;
    deadband = 0;
}

void MotorControl::begin() {
    // 配置电机控制引脚
    pinMode(PIN_MOTOR_L_I1, OUTPUT);
    pinMode(PIN_MOTOR_L_I2, OUTPUT);
    pinMode(PIN_MOTOR_R_I1, OUTPUT);
    pinMode(PIN_MOTOR_R_I2, OUTPUT);
    
    // 初始化编码器
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    leftEncoder.attachFullQuad(PIN_ENCODER_L_A, PIN_ENCODER_L_B);
    rightEncoder.attachFullQuad(PIN_ENCODER_R_A, PIN_ENCODER_R_B);
    leftEncoder.clearCount();
    rightEncoder.clearCount();
    
    // 设置PWM
    setupPWM();
    
    stop();
    Serial.println("Motor control initialized");
}

void MotorControl::setupPWM() {
    ledcSetup(PWM_CHANNEL_L1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_L2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_R1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_R2, PWM_FREQ, PWM_RESOLUTION);
    
    ledcAttachPin(PIN_MOTOR_L_I1, PWM_CHANNEL_L1);
    ledcAttachPin(PIN_MOTOR_L_I2, PWM_CHANNEL_L2);
    ledcAttachPin(PIN_MOTOR_R_I1, PWM_CHANNEL_R1);
    ledcAttachPin(PIN_MOTOR_R_I2, PWM_CHANNEL_R2);
}

void MotorControl::setPWM(uint8_t channel1, uint8_t channel2, int speed) {
    speed = constrain(speed, -255, 255);
    
    int outputPWM = 0;
    if (speed > 0) {
        // 死区补偿映射
        outputPWM = map(speed, 1, 255, deadband, 255);
        ledcWrite(channel1, outputPWM);
        ledcWrite(channel2, 0);
    } else if (speed < 0) {
        outputPWM = map(-speed, 1, 255, deadband, 255);
        ledcWrite(channel1, 0);
        ledcWrite(channel2, outputPWM);
    } else {
        // 同时低电平滑行
        ledcWrite(channel1, 0);
        ledcWrite(channel2, 0);
    }
}

void MotorControl::setLeftSpeed(int speed) {
    int calibratedSpeed = (int)(speed * leftCalib);
    setPWM(PWM_CHANNEL_L1, PWM_CHANNEL_L2, calibratedSpeed);
}

void MotorControl::setRightSpeed(int speed) {
    int calibratedSpeed = (int)(speed * rightCalib);
    setPWM(PWM_CHANNEL_R1, PWM_CHANNEL_R2, calibratedSpeed);
}

void MotorControl::setBothSpeed(int speed) {
    setLeftSpeed(speed);
    setRightSpeed(speed);
}

void MotorControl::setDifferentialSpeed(int baseSpeed, int turnAdjust) {
    // turnAdjust: 负数左转,正数右转
    int leftSpeed = baseSpeed - turnAdjust;
    int rightSpeed = baseSpeed + turnAdjust;
    
    setLeftSpeed(leftSpeed);
    setRightSpeed(rightSpeed);
}

void MotorControl::stop() {
    ledcWrite(PWM_CHANNEL_L1, 0);
    ledcWrite(PWM_CHANNEL_L2, 0);
    ledcWrite(PWM_CHANNEL_R1, 0);
    ledcWrite(PWM_CHANNEL_R2, 0);
}

void MotorControl::brake() {
    // 同时高电平刹车
    ledcWrite(PWM_CHANNEL_L1, 255);
    ledcWrite(PWM_CHANNEL_L2, 255);
    ledcWrite(PWM_CHANNEL_R1, 255);
    ledcWrite(PWM_CHANNEL_R2, 255);
}

long MotorControl::getLeftEncoder() {
    return leftEncoder.getCount();
}

long MotorControl::getRightEncoder() {
    return rightEncoder.getCount();
}

void MotorControl::resetEncoders() {
    leftEncoder.clearCount();
    rightEncoder.clearCount();
    lastLeftCount = 0;
    lastRightCount = 0;
}

float MotorControl::getLeftDistance() {
    return leftEncoder.getCount() * MM_PER_PULSE;
}

float MotorControl::getRightDistance() {
    return rightEncoder.getCount() * MM_PER_PULSE;
}

float MotorControl::getAverageDistance() {
    return (getLeftDistance() + getRightDistance()) / 2.0;
}

void MotorControl::update() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastUpdateTime) / 1000.0;  // 转换为秒
    
    if (deltaTime >= 0.05) {  // 每50ms更新一次
        long currentLeftCount = leftEncoder.getCount();
        long currentRightCount = rightEncoder.getCount();
        
        long deltaLeft = currentLeftCount - lastLeftCount;
        long deltaRight = currentRightCount - lastRightCount;
        
        leftSpeed = (deltaLeft * MM_PER_PULSE) / deltaTime;
        rightSpeed = (deltaRight * MM_PER_PULSE) / deltaTime;
        
        lastLeftCount = currentLeftCount;
        lastRightCount = currentRightCount;
        lastUpdateTime = currentTime;
        
#if DEBUG_ENCODER
        Serial.printf("Speed L:%.1f R:%.1f mm/s\n", leftSpeed, rightSpeed);
#endif
    }
}

float MotorControl::getLeftSpeed() {
    return leftSpeed;
}

float MotorControl::getRightSpeed() {
    return rightSpeed;
}

void MotorControl::setCalibration(float leftCalib, float rightCalib) {
    this->leftCalib = constrain(leftCalib, 0.5, 1.5);
    this->rightCalib = constrain(rightCalib, 0.5, 1.5);
    Serial.printf("Motor calibration set: L=%.3f R=%.3f\n", this->leftCalib, this->rightCalib);
}

void MotorControl::setDeadband(int deadband) {
    this->deadband = constrain(deadband, 0, 100);
}
