#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "config.h"

class Sensors {
public:
    Sensors();
    void begin();
    void update();
    
    // 超声波测距 (cm)
    float getUltrasonicDistance();
    
    // 激光测距 (mm)
    uint16_t getLaserDistance();
    bool isLaserReady() { return laserReady; }
    
    // 按键状态
    bool isButtonPressed();
    bool waitForButton();  // 阻塞等待按键按下
    bool checkButtonLongPress(unsigned long duration);  // 检测长按
    
    // 声光报警
    void setAlarm(bool on);
    void beep(int duration);

private:
    Adafruit_VL53L0X laser;
    bool laserReady;
    uint16_t laserDistance;
    
    unsigned long lastUltrasonicTime;
    float ultrasonicDistance;
    
    float measureUltrasonic();
    
    // 激光传感器错误处理
    int laserErrorCount;
    unsigned long lastLaserUpdateTime;
    void resetLaser();
};

#endif
