#include "LineSensor.h"

LineSensor::LineSensor() {
    uart = new HardwareSerial(1);
    states = 0;
    dataReady = false;
    memset(analogValues, 0, sizeof(analogValues));
    lastValidPosition = 0;
    lostLineCount = 0;
    // 默认权重 - 优化为非线性权重
    // 中间平缓(稳直线)，两边陡峭(快转弯)
    weights[0] = -1000;
    weights[1] = -700;
    weights[2] = -400;
    weights[3] = -100; // 降低中心区域权重，减少直线抖动
    weights[4] = 100;
    weights[5] = 400;
    weights[6] = 700;
    weights[7] = 1000;
    
    lastRequestTime = 0;
    waitingResponse = false;
}

void LineSensor::begin() {
    // 初始化串口
    uart->begin(LINE_UART_BAUD, SERIAL_8N1, PIN_LINE_RX, PIN_LINE_TX);
    delay(100);
    
    // 清空缓冲区
    while(uart->available()) {
        uart->read();
    }
    
    // 设置为手动模式 (模式0：手动请求，主动控制)
    uart->write(0);
    delay(50);  // 等待模块响应
    
    Serial.println("✓ Line sensor initialized (Manual mode)");
}

uint8_t LineSensor::calculateCheckCode(uint8_t* buf) {
    uint16_t sum = 0;
    uint8_t index = 2;
    uint8_t indexEnd = (buf[3] + index + 2);
    for (uint8_t i = index; i < indexEnd; ++i) {
        sum += buf[i];
    }
    return (uint8_t)(~sum);
}

void LineSensor::update() {
    // 非阻塞通信状态机
    if (!waitingResponse) {
        // 限制请求频率 (每4ms一次 = 250Hz)
        if (millis() - lastRequestTime >= 4) {
            if (uart->availableForWrite()) {
                // 发送前清空缓冲区，防止读取到旧数据
                while(uart->available()) uart->read();
                
                uart->write(1); // 发送读取状态指令
                waitingResponse = true;
                lastRequestTime = millis();
            }
        }
    } else {
        // 等待响应
        if (uart->available()) {
            uint8_t temp;
            uart->readBytes(&temp, 1);
            states = temp;
            dataReady = true;
            waitingResponse = false;
            
#if DEBUG_LINE_SENSOR
            Serial.print("Line States: 0x");
            Serial.print(states, HEX);
            Serial.println();
#endif
        } else {
            // 检查超时 (8ms无响应则重试)
            if (millis() - lastRequestTime > 8) {
                waitingResponse = false;
                // dataReady = false; // 可选：超时是否认为数据无效？暂时保持旧值
#if DEBUG_LINE_SENSOR
                // Serial.println("⚠ Line sensor timeout!"); // 减少串口刷屏
#endif
            }
        }
    }
}

uint8_t LineSensor::getState(uint8_t index) {
    if (index >= LINE_SENSOR_COUNT) return 0;
    return (states >> index) & 0x01;
}

uint8_t LineSensor::getRawStates() {
    return states;
}

uint16_t LineSensor::getAnalog(uint8_t index) {
    if (index >= LINE_SENSOR_COUNT) return 0;
    return analogValues[index];
}

int16_t LineSensor::getLinePosition() {
    // 电赛标准算法: 加权平均法
    // 传感器排列: [0][1][2][3][4][5][6][7]
    // 使用可配置的权重
    
    int32_t weightedSum = 0;
    int32_t activeSum = 0;
    
    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        if (getState(i) == 1) {  // 检测到黑线
            weightedSum += weights[i];
            activeSum += 1000;  // 权重基数
        }
    }
    
    if (activeSum == 0) {
        // 完全丢线,增加计数
        lostLineCount++;
        // 返回上次位置的1.2倍(加速搜索)
        return constrain(lastValidPosition * 1.2, -1000, 1000);
    }
    
    // 找到线,重置计数
    lostLineCount = 0;
    
    // 计算位置
    int16_t position = (int16_t)(weightedSum / (activeSum / 1000));
    position = constrain(position, -1000, 1000);
    
    // 记录有效位置
    lastValidPosition = position;
    
    return position;
}

uint8_t LineSensor::getActiveCount() {
    uint8_t count = 0;
    for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
        if (getState(i) == 1) count++;
    }
    return count;
}

bool LineSensor::isLostLine() {
    // 连续3次以上丢线才算真正丢线
    return (lostLineCount >= 3);
}

bool LineSensor::isAllWhite() {
    return (states == 0x00);
}

bool LineSensor::isAllBlack() {
    return (states == 0xFF);
}

void LineSensor::setWeights(int16_t newWeights[8]) {
    for (int i = 0; i < 8; i++) {
        weights[i] = newWeights[i];
    }
    Serial.println("Sensor weights updated:");
    for (int i = 0; i < 8; i++) {
        Serial.printf("  [%d]: %d\n", i, weights[i]);
    }
}

void LineSensor::getWeights(int16_t outWeights[8]) {
    for (int i = 0; i < 8; i++) {
        outWeights[i] = weights[i];
    }
}
