#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
    PIDController(float kp, float ki, float kd);
    
    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    void setSetpoint(float setpoint);
    void setIntegralRange(float range); // 设置积分分离范围
    
    float compute(float input);
    void reset();
    
    // 调试信息
    float getP() { return pTerm; }
    float getI() { return iTerm; }
    float getD() { return dTerm; }
    float getError() { return lastError; }

private:
    float kp, ki, kd;
    float setpoint;
    float lastError;
    float integral;
    float integralRange; // 积分分离阈值
    float outputMin, outputMax;
    unsigned long lastTime;
    
    float pTerm, iTerm, dTerm;  // 用于调试
};

#endif
