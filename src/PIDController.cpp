#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    setpoint = 0;
    lastError = 0;
    integral = 0;
    outputMin = -255;
    outputMax = 255;
    lastTime = 0;
    integralRange = 10000; // 默认不分离
    pTerm = iTerm = dTerm = 0;
}

void PIDController::setGains(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PIDController::setOutputLimits(float min, float max) {
    outputMin = min;
    outputMax = max;
}

void PIDController::setSetpoint(float setpoint) {
    this->setpoint = setpoint;
}

void PIDController::setIntegralRange(float range) {
    this->integralRange = range;
}

float PIDController::compute(float input) {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0;  // 转换为秒
    
    if (lastTime == 0 || deltaTime > 1.0) {  // 首次运行或时间间隔过长
        lastTime = currentTime;
        lastError = setpoint - input;
        integral = 0;  // 重置积分
        
        // 优化：首次运行立即返回P项，而不是0
        // 这样在丢线找回时能立即产生纠正力
        pTerm = kp * lastError;
        iTerm = 0;
        dTerm = 0;
        
        return constrain(pTerm, outputMin, outputMax);
    }
    
    // 限制计算频率,避免deltaTime太小导致微分爆炸
    if (deltaTime < 0.01) {  // 小于10ms不计算
        return constrain(pTerm + iTerm + dTerm, outputMin, outputMax);
    }
    
    float error = setpoint - input;
    
    // P项
    pTerm = kp * error;
    
    // I项(带抗饱和与积分分离)
    if (ki > 0) {
        // 积分分离：只有误差在允许范围内才进行积分
        if (abs(error) < integralRange) {
            float tempIntegral = integral + error * deltaTime;
            // 只在输出未饱和时累积积分
            float tempOutput = pTerm + ki * tempIntegral;
            if (tempOutput >= outputMin && tempOutput <= outputMax) {
                integral = tempIntegral;
            }
        } else {
            integral = 0; // 误差过大时清除积分
        }
        
        integral = constrain(integral, -500, 500);  // 积分限幅
        iTerm = ki * integral;
    } else {
        integral = 0;
        iTerm = 0;
    }
    
    // D项(微分先行,对输入微分而不是误差)
    float derivative = -(input - (setpoint - lastError)) / deltaTime;
    dTerm = kd * derivative;
    
    // 计算输出
    float output = pTerm + iTerm + dTerm;
    output = constrain(output, outputMin, outputMax);
    
    lastError = error;
    lastTime = currentTime;
    
    return output;
}

void PIDController::reset() {
    lastError = 0;
    integral = 0;
    lastTime = 0;
    pTerm = iTerm = dTerm = 0;
}
