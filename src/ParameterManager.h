#ifndef PARAMETER_MANAGER_H
#define PARAMETER_MANAGER_H

#include <Arduino.h>
#include <Preferences.h>
#include "config.h"

class ParameterManager {
public:
    ParameterManager();
    void begin();
    
    // PID参数 (Phase 1: Start -> Measurement End)
    float kp, ki, kd;
    
    // PID参数 (Phase 2: After Measurement)
    float kpPost, kiPost, kdPost;
    
    // 速度参数 (Phase 1)
    int speedSlow;
    int speedNormal;
    int speedFast;
    int speedTurn;

    // 速度参数 (Phase 2)
    int speedNormalPost;
    int speedFastPost;
    int speedTurnPost;
    
    // 阈值参数
    int obstacleDetectDist;
    int objectDetectDist;
    
    // 避障参数
    int avoidTurnDist;
    int avoidForwardDist;
    int avoidParallelDist; // 平行移动距离 (Step 4)
    int avoidSpeed;        // 避障直行速度
    int avoidTurnSpeed;    // 避障转弯速度
    float avoidKp;         // 避障直行修正系数
    float avoidFinalTurnDist; // 最后回正转向行程 (Step 7)
    float avoidTurn1Dist;  // 避障第1步左转行程
    float avoidTurn2Dist;  // 避障第3步右转行程
    float avoidTurn3Dist;  // 避障第5步右转行程
    int avoidSearchDist;   // 避障第6步直行搜索最大距离
    
    // 避障步骤速度系数 (Step 1-6)
    // S1: 左转, S2: 直行Out, S3: 右转1, S4: 直行Parallel, S5: 右转2, S6: 直行In
    float avoidS1_L, avoidS1_R;
    float avoidS2_L, avoidS2_R;
    float avoidS3_L, avoidS3_R;
    float avoidS4_L, avoidS4_R;
    float avoidS5_L, avoidS5_R;
    float avoidS6_L, avoidS6_R;
    
    // 车库参数
    int parkingDistSlow;      // 减速距离 (cm)
    int parkingDistVerySlow;  // 极慢速距离 (cm)
    int parkingDistStop;      // 停止距离 (cm)
    int parkingSpeedSlow;     // 减速速度
    int parkingSpeedVerySlow; // 极慢速速度
    
    // 电机校准系数 (0.5 ~ 1.5, 默认1.0)
    float motorLeftCalib;   // 左电机校准系数
    float motorRightCalib;  // 右电机校准系数
    
    // 高级PID参数
    int pidIntegralRange;      // 积分分离阈值
    int motorDeadband;         // 电机死区
    int pidSmallErrorThres;    // 直线判定阈值
    float pidKpSmallScale;     // 直线Kp缩放
    float pidKdSmallScale;     // 直线Kd缩放
    
    // 物体测量参数
    int objectFilterSize;      // 滤波窗口大小
    float objectLengthScale;   // 长度乘数
    float objectLengthOffset;  // 长度加数
    float objectDeviationCorrection; // 偏差修正系数
    
    // 编码器闭环控制参数
    float encKp, encKi, encKd; // 编码器直线保持PID
    float turn90Dist;          // 90度转向对应的单轮行程(mm)
    
    // 传感器权重
    int16_t sensorWeights[8];

    // 保存和加载
    void save();
    void load();
    void reset();  // 恢复默认值
    
    // 获取JSON字符串
    String toJson();
    void fromJson(String json);

private:
    Preferences preferences;
};

#endif
