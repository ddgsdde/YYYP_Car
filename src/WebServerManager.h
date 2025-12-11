#ifndef WEB_SERVER_MANAGER_H
#define WEB_SERVER_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "config.h"
#include "ParameterManager.h"

class WebServerManager {
public:
    WebServerManager(ParameterManager* params);
    void begin();
    void setStatusCallback(String (*callback)());
    void setMotionCallback(void (*callback)(String action, float value));
    void setWeightCallback(void (*callback)(int16_t weights[8]));
    void setCalibrationCallback(void (*callback)(float leftCalib, float rightCalib));
    void setDetectionCallback(void (*callback)(uint16_t baseline, uint16_t threshold));
    void setTaskCallback(String (*callback)(String action, String data));
    
    void addLog(String message);  // 添加日志
    String getLogs();              // 获取日志JSON
    void clearLogs();              // 清空日志
    
    void updateStatusJson(const String& json); // 更新状态JSON (由主循环调用)
    String getIPAddress();

private:
    AsyncWebServer* server;
    ParameterManager* paramManager;
    // String (*statusCallback)(); // 移除回调，改为主动更新
    String currentStatusJson;      // 缓存的状态JSON
    SemaphoreHandle_t mutex;       // 互斥锁，保护共享资源
    
    void (*motionCallback)(String action, float value);
    void (*weightCallback)(int16_t weights[8]);
    void (*calibrationCallback)(float leftCalib, float rightCalib);
    void (*detectionCallback)(uint16_t baseline, uint16_t threshold);
    String (*taskCallback)(String action, String data);
    
    // 日志缓冲区
    static const int MAX_LOGS = 50;  // 减少日志数量以节省内存
    String logs[MAX_LOGS];
    int logIndex;
    int logCount;
    
    void setupRoutes();
    String generateHTML();
};

#endif
