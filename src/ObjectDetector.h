#ifndef OBJECT_DETECTOR_H
#define OBJECT_DETECTOR_H

#include <Arduino.h>
#include "Sensors.h"
#include "MotorControl.h"

// 前向声明
class WebServerManager;

// 物块检测状态
enum DetectionState {
    DETECT_IDLE,          // 空闲
    DETECT_WAITING,       // 等待物块出现
    DETECT_IN_OBJECT,     // 正在经过物块
    DETECT_COMPLETED,     // 检测完成
    DETECT_FAILED         // 检测失败
};

// 物块检测结果
struct ObjectMeasurement {
    float length;         // 物块长度 (mm)
    float avgDistance;    // 平均距离 (mm)
    float minDistance;    // 最小距离 (mm)
    float startPos;       // 起始位置 (mm)
    float endPos;         // 结束位置 (mm)
    bool valid;           // 测量是否有效
    unsigned long timestamp; // 测量时间戳
    unsigned long duration;  // 检测持续时间 (ms)
};

// 历史数据缓冲 (用于精确边缘检测)
struct HistorySample {
    unsigned long timestamp;
    uint16_t laserDist;
    float globalDist; // 全局累积修正距离
};

class ObjectDetector {
public:
    ObjectDetector(Sensors* sensors, MotorControl* motor);
    
    // 设置WebServer用于日志输出
    void setWebServer(WebServerManager* server);
    
    // 开始检测
    void startDetection(uint16_t baselineDistance = 800, uint16_t threshold = 100);
    
    // 更新检测（需要在主循环中调用）
    void update(int16_t linePosition = 0);
    
    // 停止检测
    void stopDetection();
    
    // 获取检测状态
    DetectionState getState() { return state; }
    bool isDetecting() { return state != DETECT_IDLE && state != DETECT_COMPLETED && state != DETECT_FAILED; }
    bool isCompleted() { return state == DETECT_COMPLETED; }
    
    // 获取测量结果
    ObjectMeasurement getResult() { return result; }
    
    // 重置检测器
    void reset();
    
    // 配置参数
    void setStableCount(int count) { stableCountThreshold = count; }
    void setTimeout(unsigned long ms) { timeoutMs = ms; }
    void setFilterSize(int size) { filterSize = size; }
    void setCorrection(float scale, float offset) { lengthScale = scale; lengthOffset = offset; }
    void setDeviationCorrection(float ratio) { deviationCorrectionRatio = ratio; }

private:
    Sensors* sensors;
    MotorControl* motor;
    WebServerManager* webServer;  // 用于日志输出
    
    DetectionState state;
    ObjectMeasurement result;
    
    // 检测参数
    uint16_t baselineDistance;   // 基线距离 (无物块时的距离)
    uint16_t detectThreshold;     // 检测阈值
    int stableCountThreshold;     // 稳定计数阈值
    unsigned long timeoutMs;      // 超时时间
    int filterSize;               // 滤波窗口大小
    float lengthScale;            // 长度乘数
    float lengthOffset;           // 长度加数
    float deviationCorrectionRatio; // 偏差修正系数 (每单位偏差减少的距离比例)
    
    // 检测过程变量
    int stableCount;              // 稳定计数器
    float startEncoderPos;        // 起始编码器位置 (用于旧的简单计算)
    float endEncoderPos;          // 结束编码器位置
    
    // 新增：累积距离计算
    float accumulatedDistance;    // 累积的有效距离
    float lastEncoderPos;         // 上一次的编码器读数

    unsigned long startTime;      // 开始时间 (整个检测任务)
    unsigned long objectEnterTime; // 物块进入时间

    
    // 滤波缓冲区
    static const int MAX_FILTER_SIZE = 20;
    uint16_t filterBuffer[MAX_FILTER_SIZE];
    int filterIndex;
    int filterCount;
    
    // 距离采样（用于滤波和统计）
    static const int MAX_SAMPLES = 100;
    uint16_t distanceSamples[MAX_SAMPLES];
    int sampleCount;
    
    // 辅助函数
    float getAverageEncoderDistance();  // 获取左右编码器平均距离
    void addDistanceSample(uint16_t distance);
    float calculateMedianDistance();     // 计算中位数距离
    float calculateAverageDistance();    // 计算平均距离
    bool isDistanceStable(uint16_t distance, uint16_t baseline, uint16_t threshold);
    void log(String message);            // 日志输出（同时到串口和网页）
    
    // 新增：滑动窗口滤波
    uint16_t getFilteredDistance(uint16_t rawDistance);

    // 历史数据缓冲 (用于精确边缘检测)
    static const int HISTORY_SIZE = 50; // 50个样本，约500ms-1s的历史
    HistorySample historyBuffer[HISTORY_SIZE];
    int historyIndex;
    
    // 简化的路径测量
    float globalPathDistance;      // 累积的编码器距离
    float lastGlobalEncoderPos;    // 上次的编码器读数
    
    // 蛇形走位补偿
    float lastLeftDist;            // 上次左轮里程
    float lastRightDist;           // 上次右轮里程
    float serpentineCorrection;    // 累积的蛇形修正量
    bool enableSerpentineCorrection; // 是否启用蛇形修正

    void pushHistory(uint16_t dist, float globalDist);
    float findPreciseCrossingPoint(bool entering, uint16_t threshold);
    float calculateSerpentineCorrection(float leftDelta, float rightDelta);
};

#endif
