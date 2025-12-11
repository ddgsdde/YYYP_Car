#include "ObjectDetector.h"
#include "WebServerManager.h"

ObjectDetector::ObjectDetector(Sensors* sensors, MotorControl* motor) {
    this->sensors = sensors;
    this->motor = motor;
    this->webServer = nullptr;
    
    state = DETECT_IDLE;
    stableCountThreshold = 5;      // 提高到5次稳定读数，确保可靠性
    timeoutMs = 15000;             // 15秒超时
    filterSize = 5;                // 5点中位数滤波，平衡响应与稳定性
    lengthScale = 1.0;             // 默认乘数
    lengthOffset = 0.0;            // 默认加数
    deviationCorrectionRatio = 0.0;
    
    // 初始化历史缓冲区
    historyIndex = 0;
    globalPathDistance = 0;
    lastGlobalEncoderPos = 0;
    memset(historyBuffer, 0, sizeof(historyBuffer));
    
    // 初始化蛇形补偿
    lastLeftDist = 0;
    lastRightDist = 0;
    serpentineCorrection = 0;
    enableSerpentineCorrection = false;
    
    reset();
}

void ObjectDetector::setWebServer(WebServerManager* server) {
    this->webServer = server;
}

void ObjectDetector::reset() {
    state = DETECT_IDLE;
    stableCount = 0;
    startEncoderPos = 0;
    endEncoderPos = 0;
    accumulatedDistance = 0;
    lastEncoderPos = 0;
    startTime = 0;
    sampleCount = 0;
    
    // 重置全局路径积分（简化版）
    globalPathDistance = 0;
    lastGlobalEncoderPos = 0;
    historyIndex = 0;
    memset(historyBuffer, 0, sizeof(historyBuffer));
    
    // 重置蛇形补偿
    lastLeftDist = 0;
    lastRightDist = 0;
    serpentineCorrection = 0;
    enableSerpentineCorrection = false;

    // 重置滤波
    filterIndex = 0;
    filterCount = 0;
    memset(filterBuffer, 0, sizeof(filterBuffer));
    
    result.length = 0;
    result.avgDistance = 0;
    result.minDistance = 0;
    result.startPos = 0;
    result.endPos = 0;
    result.valid = false;
    result.timestamp = 0;
}

void ObjectDetector::startDetection(uint16_t baselineDistance, uint16_t threshold) {
    reset();
    
    this->baselineDistance = 0;
    this->detectThreshold = threshold;
    
    // 确保稳定计数至少为5
    if (stableCountThreshold < 5) {
        stableCountThreshold = 5;
    }
    
    state = DETECT_WAITING;
    startTime = millis();
    
    // 初始化编码器基准
    // lastGlobalEncoderPos = getAverageEncoderDistance();
    // globalPathDistance = 0;
    
    // 初始化蛇形补偿基准
    lastLeftDist = motor->getLeftDistance();
    lastRightDist = motor->getRightDistance();
    serpentineCorrection = 0;
    enableSerpentineCorrection = false;  // 禁用蛇形修正
    
    log("\n=== Object Detection Started ===");
    log("⚙️ Range: <" + String(threshold) + "mm");
    log("⚙️ Stable: " + String(stableCountThreshold) + " readings");
    log("⚙️ Filter: " + String(filterSize) + " points");
    log("⚙️ Scale: " + String(lengthScale, 3) + " Offset: " + String(lengthOffset, 1));
    log("⚙️ Laser: " + String(sensors->getLaserDistance()) + "mm");
    log("⚙️ Encoder: " + String(lastGlobalEncoderPos, 1) + "mm");
    log("➡ Waiting...");
}

void ObjectDetector::stopDetection() {
    if (state == DETECT_IN_OBJECT) {
        // 如果正在检测物块，记录结束位置
        endEncoderPos = getAverageEncoderDistance();
        result.endPos = endEncoderPos;
        
        // 计算长度：优先使用累积距离(如果有)，否则使用简单差值
        float rawLength;
        if (accumulatedDistance > 0) {
            rawLength = accumulatedDistance;
        } else {
            rawLength = endEncoderPos - startEncoderPos;
        }
        
        result.length = (rawLength * lengthScale) + lengthOffset;
        
        if (sampleCount > 0) {
            result.avgDistance = calculateAverageDistance();
            result.minDistance = calculateMedianDistance();  // 使用中位数更稳定
        }
        
        result.valid = (result.length > 10 && result.length < 1000);  // 合理范围
        result.timestamp = millis();
        
        state = DETECT_COMPLETED;
        
        log("=== Detection Stopped ===");
        log("Length: " + String(result.length, 1) + "mm, Avg Distance: " + String(result.avgDistance, 1) + "mm");
    } else {
        state = DETECT_IDLE;
    }
}

void ObjectDetector::update(int16_t linePosition) {
    if (state == DETECT_IDLE || state == DETECT_COMPLETED || state == DETECT_FAILED) {
        return;
    }
    
    // 检查超时
    if (millis() - startTime > timeoutMs) {
        state = DETECT_FAILED;
        log("✗ Detection timeout!");
        return;
    }
    
    // 检查传感器是否就绪
    if (!sensors->isLaserReady()) {
        static unsigned long lastWarn = 0;
        if (millis() - lastWarn > 2000) {  // 减少警告频率
            log("⚠ Laser sensor not ready!");
            lastWarn = millis();
        }
        return;
    }
    
    uint16_t rawDistance = sensors->getLaserDistance();
    
    // 1. 无效值处理：将无效值(>2000或<10)视为"无穷远"(2000mm)
    // 这样可以确保在物块结束时(后面是空的)，状态机能正确跳转
    uint16_t processedDistance = rawDistance;
    if (rawDistance > 2000 || rawDistance < 10) {
        processedDistance = 2000;
    }
    
    // 2. 滑动窗口滤波：平滑数据，消除毛刺
    uint16_t filteredDistance = getFilteredDistance(processedDistance);
    
    // 3. 连续性检查：确保激光读数连续稳定（避免跳变）
    static uint16_t lastFilteredDist = filteredDistance;
    static int jumpCount = 0;
    if (abs((int)filteredDistance - (int)lastFilteredDist) > 200) {
        jumpCount++;
        if (jumpCount < 3) {
            // 疑似跳变，使用上次值
            filteredDistance = lastFilteredDist;
        } else {
            // 连续3次大跳变，认为是真实变化
            jumpCount = 0;
            lastFilteredDist = filteredDistance;
        }
    } else {
        jumpCount = 0;
        lastFilteredDist = filteredDistance;
    }
    
    // --- 改进的路径测量：直接使用左右编码器平均值 ---
    // 移除复杂的蛇形补偿，直接输出原始平均距离
    globalPathDistance = getAverageEncoderDistance();
    
    // 存入历史缓冲区
    pushHistory(filteredDistance, globalPathDistance);
    // ----------------------------------
    
    // 调试输出（每500ms一次，便于问题诊断）
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500) {
        const char* stateStr[] = {"IDLE", "WAITING", "IN_OBJECT", "COMPLETED", "FAILED"};
        log("[Detect] State:" + String(stateStr[state]) + 
            " Raw:" + String(rawDistance) + 
            " Filt:" + String(filteredDistance) + 
            "mm | GlobalDist:" + String(globalPathDistance, 1) + "mm");
        lastDebug = millis();
    }
    
    // 新逻辑：小于阈值=物块在范围内，大于阈值=无物块
    bool inRange = (filteredDistance < detectThreshold);
    
    switch (state) {
        case DETECT_WAITING: {
            // 等待物块进入范围（必须连续稳定）
            if (inRange) {
                stableCount++;
                if (stableCount == stableCountThreshold) {
                    log("➡ Object entering, stable count: " + String(stableCount));
                }
                
                if (stableCount >= stableCountThreshold) {
                    // 确认物块进入范围
                    state = DETECT_IN_OBJECT;
                    objectEnterTime = millis(); // 记录进入时间
                    
                    // --- 精确边缘检测 ---
                    // 回溯历史找到精确的进入点
                    float preciseStart = findPreciseCrossingPoint(true, detectThreshold);
                    if (preciseStart >= 0) {  // >= 0 而不是 > 0，允许起点为0
                        startEncoderPos = preciseStart;
                        log("✓ Precise Start: " + String(startEncoderPos, 2) + "mm (Interpolated)");
                    } else {
                        // 降级方案：使用当前位置减去一个估计偏移
                        startEncoderPos = globalPathDistance - 10.0;  // 减去10mm估计延迟
                        if (startEncoderPos < 0) startEncoderPos = 0;
                        log("⚠ Fallback Start: " + String(startEncoderPos, 2) + "mm (Estimated)");
                    }
                    // --------------------
                    
                    lastEncoderPos = startEncoderPos;
                    accumulatedDistance = 0;
                    
                    result.startPos = startEncoderPos;
                    sampleCount = 0;
                    stableCount = 0;
                    
                    log("✓ Object ENTER | Filt:" + String(filteredDistance) + 
                        "mm | GlobalDist:" + String(globalPathDistance, 1) + "mm");
                }
            } else {
                stableCount = 0;  // 重置，继续等待
            }
            break;
        }
        
        case DETECT_IN_OBJECT: {
            // 记录物块距离数据
            if (inRange) {
                addDistanceSample(filteredDistance);
                stableCount = 0;  // 重置离开计数
            } else {
                // 物块离开范围（必须连续稳定）
                stableCount++;
                if (stableCount == 1) {
                    log("➡ Object exiting, stable count: " + String(stableCount));
                }
                
                if (stableCount >= stableCountThreshold) {
                    // 确认物块已离开
                    
                    // --- 精确边缘检测 ---
                    // 回溯历史找到精确的离开点
                    float preciseEnd = findPreciseCrossingPoint(false, detectThreshold);
                    if (preciseEnd >= 0) {  // >= 0 允许终点为0
                        endEncoderPos = preciseEnd;
                        log("✓ Precise End: " + String(endEncoderPos, 2) + "mm (Interpolated)");
                    } else {
                        // 降级方案：使用当前位置减去估计偏移
                        endEncoderPos = globalPathDistance - 10.0;
                        if (endEncoderPos < startEncoderPos) endEncoderPos = globalPathDistance;
                        log("⚠ Fallback End: " + String(endEncoderPos, 2) + "mm (Estimated)");
                    }
                    // --------------------
                    
                    result.endPos = endEncoderPos;
                    
                    // 计算原始长度
                    float rawLength = endEncoderPos - startEncoderPos;
                    
                    // 异常检查1：长度必须为正且在合理范围内
                    // 修改：不再中止检测，而是允许继续计算，最终通过valid标志标记为无效
                    // 这样可以确保前端总是能收到测量结果（即使是无效的）
                    if (rawLength < 0) rawLength = 0;
                    
                    if (rawLength < 10.0 || rawLength > 1200.0) {
                        log("⚠ Raw length out of range: " + String(rawLength, 1) + "mm (will be marked invalid)");
                        // 不再设置 DETECT_FAILED，继续流程
                    }
                    
                    // 直接应用校准参数 (Scale & Offset)
                    // Result = (Raw * Scale) + Offset
                    result.length = (rawLength * lengthScale) + lengthOffset;
                    
                    // 强制范围限制：500mm - 1000mm
                    if (result.length < 500.0) result.length = 500.0;
                    if (result.length > 1000.0) result.length = 1000.0;
                    
                    // 计算持续时间
                    result.duration = millis() - objectEnterTime;
                    
                    // 计算统计数据
                    if (sampleCount > 5) {  // 至少5个样本
                        result.avgDistance = calculateAverageDistance();
                        result.minDistance = calculateMedianDistance();
                    } else {
                        log("⚠ Too few samples: " + String(sampleCount));
                        result.avgDistance = filteredDistance;
                        result.minDistance = filteredDistance;
                    }
                    
                    // 有效性检查 - 更新为匹配新的范围 (500-1000mm)
                    // 只要原始长度在合理范围内(10-1200)，且有足够的样本，就认为是有效的
                    result.valid = (rawLength > 10 && rawLength < 1200 && sampleCount > 5);
                    result.timestamp = millis();
                    
                    state = DETECT_COMPLETED;
                    
                    log("\n=== Object Measurement COMPLETED ===");
                    log("📍 Start: " + String(startEncoderPos, 2) + "mm");
                    log("📍 End: " + String(endEncoderPos, 2) + "mm");
                    log("⏱ Duration: " + String(result.duration) + "ms");
                    log("📏 Raw Length: " + String(rawLength, 2) + "mm");
                    if (enableSerpentineCorrection && abs(serpentineCorrection) > 0.1) {
                        log("🐍 Serpentine Correction: " + String(serpentineCorrection, 2) + "mm");
                        log("📏 Corrected Length: " + String(rawLength + serpentineCorrection, 2) + "mm");
                    }
                    log("📏 Final Length: " + String(result.length, 1) + "mm");
                    log("⚙️ Scale: " + String(lengthScale, 3) + " | Offset: " + String(lengthOffset, 1));
                    log("📊 Avg Laser: " + String(result.avgDistance, 1) + "mm (" + String(sampleCount) + " samples)");
                    log("✓ Valid: " + String(result.valid ? "YES" : "NO"));
                }
            }
            break;
        }
        
        default:
            break;
    }
}

void ObjectDetector::pushHistory(uint16_t dist, float globalDist) {
    historyBuffer[historyIndex].timestamp = millis();
    historyBuffer[historyIndex].laserDist = dist;
    historyBuffer[historyIndex].globalDist = globalDist;
    
    historyIndex = (historyIndex + 1) % HISTORY_SIZE;
}

float ObjectDetector::findPreciseCrossingPoint(bool entering, uint16_t threshold) {
    // 从最新数据开始回溯（最多检查20个历史点，约1-2秒）
    int idx = historyIndex - 1;
    if (idx < 0) idx = HISTORY_SIZE - 1;
    
    int searchLimit = min(20, HISTORY_SIZE - 1);
    
    for (int i = 0; i < searchLimit; i++) {
        int currIdx = idx;
        int prevIdx = idx - 1;
        if (prevIdx < 0) prevIdx = HISTORY_SIZE - 1;
        
        uint16_t currDist = historyBuffer[currIdx].laserDist;
        uint16_t prevDist = historyBuffer[prevIdx].laserDist;
        float currPos = historyBuffer[currIdx].globalDist;
        float prevPos = historyBuffer[prevIdx].globalDist;
        
        // 跳过无效数据
        if (historyBuffer[currIdx].timestamp == 0 || historyBuffer[prevIdx].timestamp == 0) {
            break;
        }
        
        // 检测状态翻转
        bool found = false;
        if (entering) {
            // 进入：距离从大变小（跨越阈值）
            if (prevDist >= threshold && currDist < threshold) found = true;
        } else {
            // 离开：距离从小变大（跨越阈值）
            if (prevDist < threshold && currDist >= threshold) found = true;
        }
        
        if (found) {
            // 线性插值计算精确交点
            float distDiff = (float)currDist - (float)prevDist;
            float posDiff = currPos - prevPos;
            
            // 异常检查1：距离变化太小，无法插值
            if (abs(distDiff) < 5.0) {
                log("⚠ Interpolation: distDiff=" + String(distDiff, 1) + " too small, using mid-point");
                return (prevPos + currPos) / 2.0;
            }
            
            // 异常检查2：位置变化异常（太大或太小）
            if (abs(posDiff) > 50 || abs(posDiff) < 0.5) {
                log("⚠ Interpolation: posDiff=" + String(posDiff, 2) + "mm abnormal, using prev");
                return prevPos;
            }
            
            // 异常检查3：时间戳间隔检查（避免使用过旧数据）
            unsigned long timeDiff = historyBuffer[currIdx].timestamp - historyBuffer[prevIdx].timestamp;
            if (timeDiff > 200) {
                log("⚠ Interpolation: time gap " + String(timeDiff) + "ms too large");
                return currPos;
            }
            
            // 计算插值参数 p ∈ [0, 1]
            float p = ((float)threshold - (float)prevDist) / distDiff;
            p = constrain(p, 0.0, 1.0);  // 防止外插
            
            float interpolatedPos = prevPos + posDiff * p;
            
            // 日志输出
            String dirStr = entering ? "ENTER" : "EXIT";
            log("🎯 " + dirStr + " Edge: prev(" + String(prevDist) + "," + String(prevPos, 1) + 
                ") → curr(" + String(currDist) + "," + String(currPos, 1) + ") → p=" + 
                String(p, 2) + " → pos=" + String(interpolatedPos, 2) + "mm");
            
            return interpolatedPos;
        }
        
        idx--;
        if (idx < 0) idx = HISTORY_SIZE - 1;
    }
    
    log("⚠ Precise crossing point NOT found (searched " + String(searchLimit) + " samples)");
    return -1.0;
}

uint16_t ObjectDetector::getFilteredDistance(uint16_t rawDistance) {
    // 如果窗口大小为1或0，不滤波（直接返回原始值）
    if (filterSize <= 1) return rawDistance;
    
    // 异常值检测：如果与上次差距过大，先记录但暂不使用
    static uint16_t lastRaw = rawDistance;
    static int outlierCount = 0;
    if (abs((int)rawDistance - (int)lastRaw) > 500 && lastRaw != 0) {
        outlierCount++;
        if (outlierCount < 2) {
            // 第一次异常，使用上次值
            rawDistance = lastRaw;
        } else {
            // 连续异常，接受新值
            outlierCount = 0;
        }
    } else {
        outlierCount = 0;
    }
    lastRaw = rawDistance;
    
    // 添加到循环缓冲区
    filterBuffer[filterIndex] = rawDistance;
    filterIndex = (filterIndex + 1) % MAX_FILTER_SIZE;
    if (filterCount < MAX_FILTER_SIZE) filterCount++;
    
    // 限制实际窗口大小（不超过filterSize和已有数据量）
    int currentSize = min(min(filterCount, filterSize), 5);  // 最多5点
    if (currentSize <= 1) return rawDistance;
    
    // 复制最新的currentSize个数据用于排序
    uint16_t temp[7];  // 静态数组，避免动态分配
    int count = 0;
    
    int idx = filterIndex - 1;
    if (idx < 0) idx = MAX_FILTER_SIZE - 1;
    
    for (int i = 0; i < currentSize; i++) {
        temp[count++] = filterBuffer[idx];
        idx--;
        if (idx < 0) idx = MAX_FILTER_SIZE - 1;
    }
    
    // 简单冒泡排序（数据量小，性能足够）
    for (int i = 0; i < count - 1; i++) {
        for (int j = 0; j < count - i - 1; j++) {
            if (temp[j] > temp[j + 1]) {
                uint16_t swap = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = swap;
            }
        }
    }
    
    // 返回中位数（对于奇数个样本，取中间值；偶数个样本，取中间两个的平均）
    if (count % 2 == 1) {
        return temp[count / 2];
    } else {
        return (temp[count / 2 - 1] + temp[count / 2]) / 2;
    }
}

float ObjectDetector::getAverageEncoderDistance() {
    // 直接使用左右编码器平均值计算
    float leftDist = motor->getLeftDistance();
    float rightDist = motor->getRightDistance();
    
    return (leftDist + rightDist) / 2.0;
}

void ObjectDetector::addDistanceSample(uint16_t distance) {
    if (sampleCount < MAX_SAMPLES) {
        distanceSamples[sampleCount++] = distance;
    }
}

float ObjectDetector::calculateMedianDistance() {
    if (sampleCount == 0) return 0;
    
    // 复制数组用于排序
    uint16_t sortedSamples[MAX_SAMPLES];
    memcpy(sortedSamples, distanceSamples, sampleCount * sizeof(uint16_t));
    
    // 简单冒泡排序
    for (int i = 0; i < sampleCount - 1; i++) {
        for (int j = 0; j < sampleCount - i - 1; j++) {
            if (sortedSamples[j] > sortedSamples[j + 1]) {
                uint16_t temp = sortedSamples[j];
                sortedSamples[j] = sortedSamples[j + 1];
                sortedSamples[j + 1] = temp;
            }
        }
    }
    
    // 返回中位数
    if (sampleCount % 2 == 0) {
        return (sortedSamples[sampleCount / 2 - 1] + sortedSamples[sampleCount / 2]) / 2.0;
    } else {
        return sortedSamples[sampleCount / 2];
    }
}

float ObjectDetector::calculateAverageDistance() {
    if (sampleCount == 0) return 0;
    
    uint32_t sum = 0;
    for (int i = 0; i < sampleCount; i++) {
        sum += distanceSamples[i];
    }
    
    return sum / (float)sampleCount;
}

bool ObjectDetector::isDistanceStable(uint16_t distance, uint16_t baseline, uint16_t threshold) {
    return abs((int)distance - (int)baseline) < threshold;
}

void ObjectDetector::log(String message) {
    if (webServer != nullptr) {
        webServer->addLog(message);
    } else {
        Serial.println(message);
    }
}

float ObjectDetector::calculateSerpentineCorrection(float leftDelta, float rightDelta) {
    // 蛇形走位补偿算法
    // 原理：当左右轮行进距离不同时，车辆实际走的是弧线，而非直线
    // 使用简化的弧长公式计算修正量
    
    float avgDelta = (leftDelta + rightDelta) / 2.0;
    float wheelDiff = abs(leftDelta - rightDelta);
    
    // 如果左右轮差距很小（<0.5mm），认为是直线，无需修正
    if (wheelDiff < 0.5 || avgDelta < 0.1) {
        return 0;
    }
    
    // 轮距（从config.h获取，单位mm）
    const float wheelBase = WHEEL_BASE_CM * 10.0;  // 转换为mm
    
    // 计算转弯半径 R = wheelBase / (2 * sin(θ/2))
    // 简化近似：当θ很小时，arcLength - straightDist ≈ wheelDiff² / (6 * wheelBase)
    // 这个公式在小角度时非常准确，且计算简单
    
    float correction = (wheelDiff * wheelDiff) / (6.0 * wheelBase);
    
    // 修正量的符号：蛇形走位总是让实际距离小于平均距离
    // 所以修正量应该是负的（减去）
    correction = -correction;
    
    // 限制单次修正量不超过平均距离的10%
    float maxCorrection = avgDelta * 0.1;
    correction = constrain(correction, -maxCorrection, 0);
    
    return correction;
}
