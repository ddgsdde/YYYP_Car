#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <Arduino.h>
#include "config.h"

class LineSensor {
public:
    LineSensor();
    void begin();
    void update();
    
    // 获取传感器状态 (0=白色/无线, 1=黑色/有线)
    uint8_t getState(uint8_t index);
    uint8_t getRawStates();  // 获取原始8位状态
    
    // 获取模拟值 (0-4095, 越大越黑)
    uint16_t getAnalog(uint8_t index);
    
    // 计算线位置 (-1000 到 +1000, 0=中心, 电赛标准)
    int16_t getLinePosition();
    
    // 获取检测到的传感器数量
    uint8_t getActiveCount();
    
    // 检测特殊情况
    bool isAllWhite();  // 全白(丢线)
    bool isAllBlack();  // 全黑(可能是起点/终点标记)
    bool isLostLine();  // 判断是否丢线(需要搜索)
    
    bool isDataReady() { return dataReady; }
    
    // 获取上次有效位置
    int16_t getLastPosition() { return lastValidPosition; }
    
    // 设置/获取传感器权重
    void setWeights(int16_t newWeights[8]);
    void getWeights(int16_t outWeights[8]);

private:
    HardwareSerial* uart;
    uint8_t states;              // 8位状态数据
    uint16_t analogValues[LINE_SENSOR_COUNT];
    bool dataReady;
    
    int16_t lastValidPosition;   // 上次有效位置
    uint8_t lostLineCount;       // 丢线计数
    int16_t weights[8];          // 传感器权重(可调)
    
    uint8_t calculateCheckCode(uint8_t* buf);
    void requestStates();
    void requestAnalog();
    
    // 非阻塞通信变量
    unsigned long lastRequestTime;
    bool waitingResponse;
};

#endif
