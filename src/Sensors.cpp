#include "Sensors.h"

Sensors::Sensors() {
    laserReady = false;
    laserDistance = 0;
    ultrasonicDistance = 0;
    lastUltrasonicTime = 0;
    laserErrorCount = 0;
    lastLaserUpdateTime = 0;
}

void Sensors::begin() {
    // 初始化I2C1 (激光传感器 VL53L0X)
    // 注意: BMI160也使用I2C1,但会在IMU::begin()中再次初始化
    Wire.begin(PIN_I2C1_SDA, PIN_I2C1_SCL);
    Wire.setClock(400000);  // 设置I2C频率为400kHz
    delay(100);
    
    // 扫描I2C设备
    Serial.println("Scanning I2C bus...");
    for(uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if(Wire.endTransmission() == 0) {
            Serial.printf("  Found device at 0x%02X\n", addr);
        }
    }
    
    // 初始化VL53L0X
    Serial.println("Initializing VL53L0X...");
    if (laser.begin(VL53L0X_I2C_ADDR, false, &Wire)) {
        Serial.println("✓ VL53L0X found, starting continuous mode (20ms)...");
        // 启用连续测量模式，设置20ms采样周期 (默认是30ms)
        // 这会牺牲最大测量距离，但提高响应速度
        laser.startRangeContinuous(20); 
        laserReady = true;
        Serial.println("✓ VL53L0X initialized successfully");
    } else {
        Serial.println("✗ VL53L0X init failed!");
        laserReady = false;
    }
    
    // 初始化超声波
    pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
    pinMode(PIN_ULTRASONIC_ECHO, INPUT);
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
    
    // 初始化按键
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    
    // 初始化声光报警
    pinMode(PIN_ALARM, OUTPUT);
    digitalWrite(PIN_ALARM, LOW);
}

void Sensors::update() {
    // 更新激光测距（增加滤波，提高稳定性）
    if (laserReady && laser.isRangeComplete()) {
        uint16_t newReading = laser.readRange();
        lastLaserUpdateTime = millis();
        
        // 检查是否为错误值 (8190/8191通常表示超时或超出量程)
        if (newReading >= 8190) {
            // 这是一个有效状态，表示"超出量程"或"无物体"
            // 我们应该更新距离为最大值，以便上层逻辑知道当前没有物体
            laserDistance = 8190;
            
            // 不需要复位，因为这可能是正常的无物体状态
            // 传感器死机由下方的超时检测处理
            laserErrorCount = 0;
            
            // 更新滤波器的历史值，防止下次检测到物体时误判为突变
            // 但设为8190可能会导致下次检测到近距离物体(如100mm)时差值过大
            // 所以这里不更新lastReading，或者将其重置为0让滤波器重新初始化
        } else {
            laserErrorCount = 0; // 读数正常，清零计数器
            
            // 简单的一阶滤波，平滑激光读数
            static uint16_t lastReading = newReading;
            if (abs((int)newReading - (int)lastReading) < 300 || lastReading == 0 || lastReading >= 8190) {
                // 正常变化，使用加权平均
                laserDistance = (lastReading * 3 + newReading * 7) / 10;  // 70%新值，30%旧值
                lastReading = laserDistance;
            } else {
                // 突变，可能是噪声，保持上次值
                static int jumpCount = 0;
                jumpCount++;
                if (jumpCount > 2) {
                    // 连续3次突变，接受新值
                    laserDistance = newReading;
                    lastReading = newReading;
                    jumpCount = 0;
                }
            }
        }
        
        // 调试输出（每秒一次）
        static unsigned long lastDebug = 0;
        if (millis() - lastDebug > 1000) {
            Serial.printf("[Laser] Raw:%d Filtered:%dmm\n", newReading, laserDistance);
            lastDebug = millis();
        }
    } else if (laserReady) {
        // 如果长时间没有数据更新 (超过500ms)
        if (millis() - lastLaserUpdateTime > 500) {
             Serial.println("⚠ VL53L0X timeout, resetting...");
             resetLaser();
             lastLaserUpdateTime = millis();
        }
    }
    
    // 更新超声波测距(限制更新频率)
    unsigned long currentTime = millis();
    if (currentTime - lastUltrasonicTime > 50) {  // 每50ms更新一次
        ultrasonicDistance = measureUltrasonic();
        lastUltrasonicTime = currentTime;
    }
}

float Sensors::measureUltrasonic() {
    // 发送触发脉冲
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
    
    // 读取回波
    long duration = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, 30000);  // 30ms超时
    
    if (duration == 0) {
        return 999.9;  // 超时,返回最大值
    }
    
    // 计算距离 (cm)
    // 声速 = 340m/s = 0.034cm/us
    // 距离 = (时间 * 声速) / 2
    float distance = duration * 0.034 / 2.0;
    
    return distance;
}

float Sensors::getUltrasonicDistance() {
    return ultrasonicDistance;
}

uint16_t Sensors::getLaserDistance() {
    return laserDistance;
}

bool Sensors::isButtonPressed() {
    return digitalRead(PIN_BUTTON) == LOW;  // 按下接地
}

bool Sensors::waitForButton() {
    Serial.println("Waiting for button press...");
    while (!isButtonPressed()) {
        delay(10);
    }
    delay(50);  // 消抖
    while (isButtonPressed()) {
        delay(10);
    }
    delay(50);
    Serial.println("Button pressed!");
    return true;
}

bool Sensors::checkButtonLongPress(unsigned long duration) {
    if (!isButtonPressed()) {
        return false;
    }
    
    unsigned long startTime = millis();
    while (isButtonPressed()) {
        if (millis() - startTime >= duration) {
            // 等待释放
            while (isButtonPressed()) {
                delay(10);
            }
            delay(50);  // 消抖
            return true;
        }
        delay(10);
    }
    
    return false;
}

void Sensors::setAlarm(bool on) {
    digitalWrite(PIN_ALARM, on ? HIGH : LOW);
}

void Sensors::beep(int duration) {
    setAlarm(true);
    delay(duration);
    setAlarm(false);
}

void Sensors::resetLaser() {
    // 尝试软复位
    Wire.begin(PIN_I2C1_SDA, PIN_I2C1_SCL); // 重新初始化I2C总线
    Wire.setClock(400000);
    
    if (laser.begin(VL53L0X_I2C_ADDR, false, &Wire)) {
        laser.startRangeContinuous(20);
        Serial.println("✓ VL53L0X reset success");
        laserReady = true;
    } else {
        Serial.println("✗ VL53L0X reset failed");
        laserReady = false;
    }
}
