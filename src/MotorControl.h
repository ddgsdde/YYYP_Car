#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <ESP32Encoder.h>
#include "config.h"

class MotorControl {
public:
    MotorControl();
    void begin();
    
    // 设置电机速度 (-255 到 +255, 负数为反转)
    void setLeftSpeed(int speed);
    void setRightSpeed(int speed);
    void setBothSpeed(int speed);
    void setDifferentialSpeed(int baseSpeed, int turnAdjust);
    
    // 停止
    void stop();
    void brake();  // 刹车(短接)
    
    // 编码器读取
    long getLeftEncoder();
    long getRightEncoder();
    void resetEncoders();
    
    // 里程计算
    float getLeftDistance();   // mm
    float getRightDistance();  // mm
    float getAverageDistance(); // mm
    
    // 速度计算 (需要定期调用update)
    void update();
    float getLeftSpeed();      // mm/s
    float getRightSpeed();     // mm/s
    
    // 电机校准系数
    void setCalibration(float leftCalib, float rightCalib);
    void setDeadband(int deadband); // 设置死区
    
private:
    ESP32Encoder leftEncoder;
    ESP32Encoder rightEncoder;
    
    int deadband; // 死区值
    
    long lastLeftCount;
    long lastRightCount;
    unsigned long lastUpdateTime;
    
    float leftSpeed;   // mm/s
    float rightSpeed;  // mm/s
    
    float leftCalib;   // 左电机校准系数
    float rightCalib;  // 右电机校准系数
    
    void setupPWM();
    void setPWM(uint8_t channel1, uint8_t channel2, int speed);
};

#endif
