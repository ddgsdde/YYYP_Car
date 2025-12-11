#include "ParameterManager.h"
#include <ArduinoJson.h>

ParameterManager::ParameterManager() {
    // 默认值 (Phase 1)
    kp = KP_LINE;
    ki = KI_LINE;
    kd = KD_LINE;
    
    // 默认值 (Phase 2) - 默认与Phase 1相同
    kpPost = KP_LINE;
    kiPost = KI_LINE;
    kdPost = KD_LINE;
    
    speedSlow = SPEED_SLOW;
    speedNormal = SPEED_NORMAL;
    speedFast = SPEED_FAST;
    speedTurn = SPEED_TURN;
    
    speedNormalPost = SPEED_NORMAL;
    speedFastPost = SPEED_FAST;
    speedTurnPost = SPEED_TURN;
    
    obstacleDetectDist = OBSTACLE_DETECT_DIST;
    objectDetectDist = OBJECT_DETECT_DIST;
    
    avoidTurnDist = 100;
    avoidForwardDist = 400;
    avoidParallelDist = 500;
    avoidSpeed = SPEED_NORMAL;
    avoidTurnSpeed = SPEED_TURN;
    avoidKp = 2.0;
    avoidFinalTurnDist = 118.0;
    avoidTurn1Dist = 118.0;
    avoidTurn2Dist = 118.0;
    avoidTurn3Dist = 118.0;
    avoidSearchDist = 800;
    
    parkingDistSlow = 60;
    parkingDistVerySlow = 30;
    parkingDistStop = 10;
    parkingSpeedSlow = 100;
    parkingSpeedVerySlow = 60;
    
    motorLeftCalib = 1.0;
    motorRightCalib = 1.0;
    
    // 高级PID默认值
    pidIntegralRange = PID_INTEGRAL_RANGE;
    motorDeadband = MOTOR_DEADBAND;
    pidSmallErrorThres = PID_SMALL_ERROR_THRES;
    pidKpSmallScale = PID_KP_SMALL_SCALE;
    pidKdSmallScale = PID_KD_SMALL_SCALE;
    
    // 物体测量默认值（优先保证稳定性）
    objectFilterSize = 5;          // 5点滤波，平衡稳定性与响应
    objectLengthScale = OBJECT_LENGTH_SCALE;
    objectLengthOffset = OBJECT_LENGTH_OFFSET;
    objectDeviationCorrection = 0.0;

    // 编码器闭环参数默认值
    encKp = 1.0;   // 差速修正比例
    encKi = 0.0;
    encKd = 0.0;
    // 轮距15cm，原地旋转90度，单轮行程 = pi * 150 * (90/360) ≈ 117.8mm
    turn90Dist = 118.0; 

    // 避障步骤系数默认值
    avoidS1_L = 1.0; avoidS1_R = 1.0;
    avoidS2_L = 1.0; avoidS2_R = 1.0;
    avoidS3_L = 1.0; avoidS3_R = 1.0;
    avoidS4_L = 1.0; avoidS4_R = 1.0;
    avoidS5_L = 1.0; avoidS5_R = 1.0;
    avoidS6_L = 1.0; avoidS6_R = 1.0;
    
    // 传感器权重默认值
    sensorWeights[0] = -1000;
    sensorWeights[1] = -700;
    sensorWeights[2] = -400;
    sensorWeights[3] = -100;
    sensorWeights[4] = 100;
    sensorWeights[5] = 400;
    sensorWeights[6] = 700;
    sensorWeights[7] = 1000;
}

void ParameterManager::begin() {
    preferences.begin("smartcar", false);
    load();
}

void ParameterManager::save() {
    preferences.putFloat("kp", kp);
    preferences.putFloat("ki", ki);
    preferences.putFloat("kd", kd);
    
    preferences.putFloat("kpPost", kpPost);
    preferences.putFloat("kiPost", kiPost);
    preferences.putFloat("kdPost", kdPost);
    
    preferences.putInt("speedSlow", speedSlow);
    preferences.putInt("speedNormal", speedNormal);
    preferences.putInt("speedFast", speedFast);
    preferences.putInt("speedTurn", speedTurn);
    
    preferences.putInt("spdNormPost", speedNormalPost);
    preferences.putInt("spdFastPost", speedFastPost);
    preferences.putInt("spdTurnPost", speedTurnPost);
    
    preferences.putInt("obstacleDist", obstacleDetectDist);
    preferences.putInt("objectDist", objectDetectDist);
    
    preferences.putInt("avoidTurn", avoidTurnDist);
    preferences.putInt("avoidForward", avoidForwardDist);
    preferences.putInt("avoidParallel", avoidParallelDist);
    preferences.putInt("avoidSpeed", avoidSpeed);
    preferences.putInt("avoidTurnSpd", avoidTurnSpeed);
    preferences.putFloat("avoidKp", avoidKp);
    preferences.putFloat("avoidFinal", avoidFinalTurnDist);
    preferences.putFloat("avoidTurn1", avoidTurn1Dist);
    preferences.putFloat("avoidTurn2", avoidTurn2Dist);
    preferences.putFloat("avoidTurn3", avoidTurn3Dist);
    preferences.putInt("avoidSearch", avoidSearchDist);
    
    preferences.putFloat("avS1L", avoidS1_L); preferences.putFloat("avS1R", avoidS1_R);
    preferences.putFloat("avS2L", avoidS2_L); preferences.putFloat("avS2R", avoidS2_R);
    preferences.putFloat("avS3L", avoidS3_L); preferences.putFloat("avS3R", avoidS3_R);
    preferences.putFloat("avS4L", avoidS4_L); preferences.putFloat("avS4R", avoidS4_R);
    preferences.putFloat("avS5L", avoidS5_L); preferences.putFloat("avS5R", avoidS5_R);
    preferences.putFloat("avS6L", avoidS6_L); preferences.putFloat("avS6R", avoidS6_R);
    
    preferences.putInt("pkDistSlow", parkingDistSlow);
    preferences.putInt("pkDistVSlow", parkingDistVerySlow);
    preferences.putInt("pkDistStop", parkingDistStop);
    preferences.putInt("pkSpdSlow", parkingSpeedSlow);
    preferences.putInt("pkSpdVSlow", parkingSpeedVerySlow);
    
    preferences.putFloat("motorLCalib", motorLeftCalib);
    preferences.putFloat("motorRCalib", motorRightCalib);
    
    preferences.putInt("pidIntRange", pidIntegralRange);
    preferences.putInt("deadband", motorDeadband);
    preferences.putInt("smallErr", pidSmallErrorThres);
    preferences.putFloat("kpScale", pidKpSmallScale);
    preferences.putFloat("kdScale", pidKdSmallScale);
    
    preferences.putInt("objFilter", objectFilterSize);
    preferences.putFloat("objScale", objectLengthScale);
    preferences.putFloat("objOffset", objectLengthOffset);
    preferences.putFloat("objDevCorr", objectDeviationCorrection);
    
    preferences.putFloat("encKp", encKp);
    preferences.putFloat("encKi", encKi);
    preferences.putFloat("encKd", encKd);
    preferences.putFloat("turn90", turn90Dist);
    
    // 保存传感器权重
    for (int i = 0; i < 8; i++) {
        String key = "w" + String(i);
        preferences.putInt(key.c_str(), sensorWeights[i]);
    }

    Serial.println("Parameters saved!");
}

void ParameterManager::load() {
    kp = preferences.getFloat("kp", KP_LINE);
    ki = preferences.getFloat("ki", KI_LINE);
    kd = preferences.getFloat("kd", KD_LINE);
    
    kpPost = preferences.getFloat("kpPost", KP_LINE);
    kiPost = preferences.getFloat("kiPost", KI_LINE);
    kdPost = preferences.getFloat("kdPost", KD_LINE);
    
    speedSlow = preferences.getInt("speedSlow", SPEED_SLOW);
    speedNormal = preferences.getInt("speedNormal", SPEED_NORMAL);
    speedFast = preferences.getInt("speedFast", SPEED_FAST);
    speedTurn = preferences.getInt("speedTurn", SPEED_TURN);
    
    speedNormalPost = preferences.getInt("spdNormPost", SPEED_NORMAL);
    speedFastPost = preferences.getInt("spdFastPost", SPEED_FAST);
    speedTurnPost = preferences.getInt("spdTurnPost", SPEED_TURN);
    
    obstacleDetectDist = preferences.getInt("obstacleDist", OBSTACLE_DETECT_DIST);
    objectDetectDist = preferences.getInt("objectDist", OBJECT_DETECT_DIST);
    
    avoidTurnDist = preferences.getInt("avoidTurn", 100);
    avoidForwardDist = preferences.getInt("avoidForward", 400);
    avoidParallelDist = preferences.getInt("avoidParallel", 500);
    avoidSpeed = preferences.getInt("avoidSpeed", SPEED_NORMAL);
    avoidTurnSpeed = preferences.getInt("avoidTurnSpd", SPEED_TURN);
    avoidKp = preferences.getFloat("avoidKp", 2.0);
    avoidFinalTurnDist = preferences.getFloat("avoidFinal", 118.0);
    avoidTurn1Dist = preferences.getFloat("avoidTurn1", 118.0);
    avoidTurn2Dist = preferences.getFloat("avoidTurn2", 118.0);
    avoidTurn3Dist = preferences.getFloat("avoidTurn3", 118.0);
    avoidSearchDist = preferences.getInt("avoidSearch", 800);
    
    avoidS1_L = preferences.getFloat("avS1L", 1.0); avoidS1_R = preferences.getFloat("avS1R", 1.0);
    avoidS2_L = preferences.getFloat("avS2L", 1.0); avoidS2_R = preferences.getFloat("avS2R", 1.0);
    avoidS3_L = preferences.getFloat("avS3L", 1.0); avoidS3_R = preferences.getFloat("avS3R", 1.0);
    avoidS4_L = preferences.getFloat("avS4L", 1.0); avoidS4_R = preferences.getFloat("avS4R", 1.0);
    avoidS5_L = preferences.getFloat("avS5L", 1.0); avoidS5_R = preferences.getFloat("avS5R", 1.0);
    avoidS6_L = preferences.getFloat("avS6L", 1.0); avoidS6_R = preferences.getFloat("avS6R", 1.0);
    
    parkingDistSlow = preferences.getInt("pkDistSlow", 60);
    parkingDistVerySlow = preferences.getInt("pkDistVSlow", 30);
    parkingDistStop = preferences.getInt("pkDistStop", 10);
    parkingSpeedSlow = preferences.getInt("pkSpdSlow", 100);
    parkingSpeedVerySlow = preferences.getInt("pkSpdVSlow", 60);
    
    motorLeftCalib = preferences.getFloat("motorLCalib", 1.0);
    motorRightCalib = preferences.getFloat("motorRCalib", 1.0);
    
    pidIntegralRange = preferences.getInt("pidIntRange", PID_INTEGRAL_RANGE);
    motorDeadband = preferences.getInt("deadband", MOTOR_DEADBAND);
    pidSmallErrorThres = preferences.getInt("smallErr", PID_SMALL_ERROR_THRES);
    pidKpSmallScale = preferences.getFloat("kpScale", PID_KP_SMALL_SCALE);
    pidKdSmallScale = preferences.getFloat("kdScale", PID_KD_SMALL_SCALE);
    
    objectFilterSize = preferences.getInt("objFilter", 5);
    objectLengthScale = preferences.getFloat("objScale", OBJECT_LENGTH_SCALE);
    objectLengthOffset = preferences.getFloat("objOffset", OBJECT_LENGTH_OFFSET);
    objectDeviationCorrection = preferences.getFloat("objDevCorr", 0.0);
    
    encKp = preferences.getFloat("encKp", 1.0);
    encKi = preferences.getFloat("encKi", 0.0);
    encKd = preferences.getFloat("encKd", 0.0);
    turn90Dist = preferences.getFloat("turn90", 118.0);
    
    // 加载传感器权重
    int16_t defaultWeights[] = {-1000, -700, -400, -100, 100, 400, 700, 1000};
    for (int i = 0; i < 8; i++) {
        String key = "w" + String(i);
        sensorWeights[i] = preferences.getInt(key.c_str(), defaultWeights[i]);
    }

    // 安全检查：防止非法参数导致电机不转
    if (motorLeftCalib < 0.1 || motorLeftCalib > 2.0) motorLeftCalib = 1.0;
    if (motorRightCalib < 0.1 || motorRightCalib > 2.0) motorRightCalib = 1.0;
    if (motorDeadband < 0 || motorDeadband > 100) motorDeadband = 30;
    
    // 检查避障系数
    if (avoidS1_L < 0.1) avoidS1_L = 1.0; if (avoidS1_R < 0.1) avoidS1_R = 1.0;
    if (avoidS2_L < 0.1) avoidS2_L = 1.0; if (avoidS2_R < 0.1) avoidS2_R = 1.0;
    if (avoidS3_L < 0.1) avoidS3_L = 1.0; if (avoidS3_R < 0.1) avoidS3_R = 1.0;
    if (avoidS4_L < 0.1) avoidS4_L = 1.0; if (avoidS4_R < 0.1) avoidS4_R = 1.0;
    if (avoidS5_L < 0.1) avoidS5_L = 1.0; if (avoidS5_R < 0.1) avoidS5_R = 1.0;
    if (avoidS6_L < 0.1) avoidS6_L = 1.0; if (avoidS6_R < 0.1) avoidS6_R = 1.0;

    Serial.println("Parameters loaded!");
}

void ParameterManager::reset() {
    preferences.clear();
    
    // 恢复默认值
    kp = KP_LINE;
    ki = KI_LINE;
    kd = KD_LINE;
    
    kpPost = KP_LINE;
    kiPost = KI_LINE;
    kdPost = KD_LINE;
    
    speedSlow = SPEED_SLOW;
    speedNormal = SPEED_NORMAL;
    speedFast = SPEED_FAST;
    speedTurn = SPEED_TURN;
    
    speedNormalPost = SPEED_NORMAL;
    speedFastPost = SPEED_FAST;
    speedTurnPost = SPEED_TURN;
    
    obstacleDetectDist = OBSTACLE_DETECT_DIST;
    objectDetectDist = OBJECT_DETECT_DIST;
    
    avoidTurnDist = 100;
    avoidForwardDist = 400;
    avoidParallelDist = 500;
    avoidSpeed = SPEED_NORMAL;
    avoidTurnSpeed = SPEED_TURN;
    avoidKp = 2.0;
    avoidFinalTurnDist = 118.0;
    avoidTurn1Dist = 118.0;
    avoidTurn2Dist = 118.0;
    avoidTurn3Dist = 118.0;
    avoidSearchDist = 800;
    
    parkingDistSlow = 60;
    parkingDistVerySlow = 30;
    parkingDistStop = 10;
    parkingSpeedSlow = 100;
    parkingSpeedVerySlow = 60;
    
    motorLeftCalib = 1.0;
    motorRightCalib = 1.0;
    
    pidIntegralRange = PID_INTEGRAL_RANGE;
    motorDeadband = MOTOR_DEADBAND;
    pidSmallErrorThres = PID_SMALL_ERROR_THRES;
    pidKpSmallScale = PID_KP_SMALL_SCALE;
    pidKdSmallScale = PID_KD_SMALL_SCALE;
    
    objectFilterSize = 5;
    objectLengthScale = OBJECT_LENGTH_SCALE;
    objectLengthOffset = OBJECT_LENGTH_OFFSET;
    objectDeviationCorrection = 0.0;
    
    encKp = 1.0;
    encKi = 0.0;
    encKd = 0.0;
    turn90Dist = 118.0;

    // 避障步骤系数默认值
    avoidS1_L = 1.0; avoidS1_R = 1.0;
    avoidS2_L = 1.0; avoidS2_R = 1.0;
    avoidS3_L = 1.0; avoidS3_R = 1.0;
    avoidS4_L = 1.0; avoidS4_R = 1.0;
    avoidS5_L = 1.0; avoidS5_R = 1.0;
    avoidS6_L = 1.0; avoidS6_R = 1.0;

    // 重置传感器权重
    sensorWeights[0] = -1000;
    sensorWeights[1] = -700;
    sensorWeights[2] = -400;
    sensorWeights[3] = -100;
    sensorWeights[4] = 100;
    sensorWeights[5] = 400;
    sensorWeights[6] = 700;
    sensorWeights[7] = 1000;

    save();
    Serial.println("Parameters reset to default!");
}

String ParameterManager::toJson() {
    JsonDocument doc;
    
    JsonObject pid = doc["pid"].to<JsonObject>();
    pid["kp"] = kp;
    pid["ki"] = ki;
    pid["kd"] = kd;
    pid["kpPost"] = kpPost;
    pid["kiPost"] = kiPost;
    pid["kdPost"] = kdPost;
    
    JsonObject speed = doc["speed"].to<JsonObject>();
    speed["slow"] = speedSlow;
    speed["normal"] = speedNormal;
    speed["fast"] = speedFast;
    speed["turn"] = speedTurn;
    speed["normalPost"] = speedNormalPost;
    speed["fastPost"] = speedFastPost;
    speed["turnPost"] = speedTurnPost;
    
    JsonObject threshold = doc["threshold"].to<JsonObject>();
    threshold["obstacle"] = obstacleDetectDist;
    threshold["object"] = objectDetectDist;
    
    JsonObject avoid = doc["avoid"].to<JsonObject>();
    avoid["turn"] = avoidTurnDist;
    avoid["forward"] = avoidForwardDist;
    avoid["parallel"] = avoidParallelDist;
    avoid["speed"] = avoidSpeed;
    avoid["turnSpeed"] = avoidTurnSpeed;
    avoid["kp"] = avoidKp;
    avoid["finalTurn"] = avoidFinalTurnDist;
    avoid["turn1"] = avoidTurn1Dist;
    avoid["turn2"] = avoidTurn2Dist;
    avoid["turn3"] = avoidTurn3Dist;
    avoid["search"] = avoidSearchDist;
    
    JsonObject avSteps = doc["avoidSteps"].to<JsonObject>();
    avSteps["s1l"] = avoidS1_L; avSteps["s1r"] = avoidS1_R;
    avSteps["s2l"] = avoidS2_L; avSteps["s2r"] = avoidS2_R;
    avSteps["s3l"] = avoidS3_L; avSteps["s3r"] = avoidS3_R;
    avSteps["s4l"] = avoidS4_L; avSteps["s4r"] = avoidS4_R;
    avSteps["s5l"] = avoidS5_L; avSteps["s5r"] = avoidS5_R;
    avSteps["s6l"] = avoidS6_L; avSteps["s6r"] = avoidS6_R;
    
    JsonObject park = doc["parking"].to<JsonObject>();
    park["distSlow"] = parkingDistSlow;
    park["distVSlow"] = parkingDistVerySlow;
    park["distStop"] = parkingDistStop;
    park["spdSlow"] = parkingSpeedSlow;
    park["spdVSlow"] = parkingSpeedVerySlow;
    
    JsonObject calib = doc["motorCalib"].to<JsonObject>();
    calib["left"] = motorLeftCalib;
    calib["right"] = motorRightCalib;
    
    JsonObject adv = doc["advanced"].to<JsonObject>();
    adv["intRange"] = pidIntegralRange;
    adv["deadband"] = motorDeadband;
    adv["smallErr"] = pidSmallErrorThres;
    adv["kpScale"] = pidKpSmallScale;
    adv["kdScale"] = pidKdSmallScale;
    
    JsonObject obj = doc["object"].to<JsonObject>();
    obj["filter"] = objectFilterSize;
    obj["scale"] = objectLengthScale;
    obj["offset"] = objectLengthOffset;
    obj["devCorr"] = objectDeviationCorrection;
    obj["threshold"] = objectDetectDist; // 添加激光检测阈值
    
    JsonObject enc = doc["encoder"].to<JsonObject>();
    enc["kp"] = encKp;
    enc["ki"] = encKi;
    enc["kd"] = encKd;
    enc["turn90"] = turn90Dist;
    
    // 添加传感器权重
    JsonArray w = doc["weights"].to<JsonArray>();
    for (int i = 0; i < 8; i++) {
        w.add(sensorWeights[i]);
    }

    String output;
    serializeJson(doc, output);
    return output;
}

void ParameterManager::fromJson(String json) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json);
    
    if (error) {
        Serial.print("JSON parse error: ");
        Serial.println(error.c_str());
        return;
    }
    
    if (doc["pid"].is<JsonObject>()) {
        kp = doc["pid"]["kp"] | kp;
        ki = doc["pid"]["ki"] | ki;
        kd = doc["pid"]["kd"] | kd;
        kpPost = doc["pid"]["kpPost"] | kpPost;
        kiPost = doc["pid"]["kiPost"] | kiPost;
        kdPost = doc["pid"]["kdPost"] | kdPost;
    }
    
    if (doc["speed"].is<JsonObject>()) {
        speedSlow = doc["speed"]["slow"] | speedSlow;
        speedNormal = doc["speed"]["normal"] | speedNormal;
        speedFast = doc["speed"]["fast"] | speedFast;
        speedTurn = doc["speed"]["turn"] | speedTurn;
        speedNormalPost = doc["speed"]["normalPost"] | speedNormalPost;
        speedFastPost = doc["speed"]["fastPost"] | speedFastPost;
        speedTurnPost = doc["speed"]["turnPost"] | speedTurnPost;
    }
    
    if (doc["threshold"].is<JsonObject>()) {
        obstacleDetectDist = doc["threshold"]["obstacle"] | obstacleDetectDist;
    }
    
    if (doc["weights"].is<JsonArray>()) {
        JsonArray w = doc["weights"];
        for (int i = 0; i < 8; i++) {
            sensorWeights[i] = w[i] | sensorWeights[i];
        }
    }
    
    if (doc["avoid"].is<JsonObject>()) {
        avoidTurnDist = doc["avoid"]["turn"] | avoidTurnDist;
        avoidForwardDist = doc["avoid"]["forward"] | avoidForwardDist;
        avoidParallelDist = doc["avoid"]["parallel"] | avoidParallelDist;
        avoidSpeed = doc["avoid"]["speed"] | avoidSpeed;
        avoidTurnSpeed = doc["avoid"]["turnSpeed"] | avoidTurnSpeed;
        avoidKp = doc["avoid"]["kp"] | avoidKp;
        avoidFinalTurnDist = doc["avoid"]["finalTurn"] | avoidFinalTurnDist;
        avoidTurn1Dist = doc["avoid"]["turn1"] | avoidTurn1Dist;
        avoidTurn2Dist = doc["avoid"]["turn2"] | avoidTurn2Dist;
        avoidTurn3Dist = doc["avoid"]["turn3"] | avoidTurn3Dist;
        avoidSearchDist = doc["avoid"]["search"] | avoidSearchDist;
    }
    
    if (doc["avoidSteps"].is<JsonObject>()) {
        avoidS1_L = doc["avoidSteps"]["s1l"] | avoidS1_L; avoidS1_R = doc["avoidSteps"]["s1r"] | avoidS1_R;
        avoidS2_L = doc["avoidSteps"]["s2l"] | avoidS2_L; avoidS2_R = doc["avoidSteps"]["s2r"] | avoidS2_R;
        avoidS3_L = doc["avoidSteps"]["s3l"] | avoidS3_L; avoidS3_R = doc["avoidSteps"]["s3r"] | avoidS3_R;
        avoidS4_L = doc["avoidSteps"]["s4l"] | avoidS4_L; avoidS4_R = doc["avoidSteps"]["s4r"] | avoidS4_R;
        avoidS5_L = doc["avoidSteps"]["s5l"] | avoidS5_L; avoidS5_R = doc["avoidSteps"]["s5r"] | avoidS5_R;
        avoidS6_L = doc["avoidSteps"]["s6l"] | avoidS6_L; avoidS6_R = doc["avoidSteps"]["s6r"] | avoidS6_R;
    }
    
    if (doc["parking"].is<JsonObject>()) {
        parkingDistSlow = doc["parking"]["distSlow"] | parkingDistSlow;
        parkingDistVerySlow = doc["parking"]["distVSlow"] | parkingDistVerySlow;
        parkingDistStop = doc["parking"]["distStop"] | parkingDistStop;
        parkingSpeedSlow = doc["parking"]["spdSlow"] | parkingSpeedSlow;
        parkingSpeedVerySlow = doc["parking"]["spdVSlow"] | parkingSpeedVerySlow;
    }
    
    if (doc["motorCalib"].is<JsonObject>()) {
        motorLeftCalib = doc["motorCalib"]["left"] | motorLeftCalib;
        motorRightCalib = doc["motorCalib"]["right"] | motorRightCalib;
        
        // 限制范围
        if (motorLeftCalib < 0.5) motorLeftCalib = 0.5;
        if (motorLeftCalib > 1.5) motorLeftCalib = 1.5;
        if (motorRightCalib < 0.5) motorRightCalib = 0.5;
        if (motorRightCalib > 1.5) motorRightCalib = 1.5;
    }
    
    if (doc["advanced"].is<JsonObject>()) {
        pidIntegralRange = doc["advanced"]["intRange"] | pidIntegralRange;
        motorDeadband = doc["advanced"]["deadband"] | motorDeadband;
        pidSmallErrorThres = doc["advanced"]["smallErr"] | pidSmallErrorThres;
        pidKpSmallScale = doc["advanced"]["kpScale"] | pidKpSmallScale;
        pidKdSmallScale = doc["advanced"]["kdScale"] | pidKdSmallScale;
    }
    
    if (doc["object"].is<JsonObject>()) {
        objectFilterSize = doc["object"]["filter"] | objectFilterSize;
        objectLengthScale = doc["object"]["scale"] | objectLengthScale;
        objectLengthOffset = doc["object"]["offset"] | objectLengthOffset;
        objectDeviationCorrection = doc["object"]["devCorr"] | objectDeviationCorrection;
        objectDetectDist = doc["object"]["threshold"] | objectDetectDist; // 解析激光检测阈值
    }
    
    if (doc["encoder"].is<JsonObject>()) {
        encKp = doc["encoder"]["kp"] | encKp;
        encKi = doc["encoder"]["ki"] | encKi;
        encKd = doc["encoder"]["kd"] | encKd;
        turn90Dist = doc["encoder"]["turn90"] | turn90Dist;
    }

    save();
}
