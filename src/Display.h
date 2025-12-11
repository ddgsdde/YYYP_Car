#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "config.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

class Display {
public:
    Display();
    void begin();
    void clear();
    void update();
    
    // 显示状态信息
    void showStatus(SystemState state, float linePos, float distance);
    void showMeasurement(float objectLength);
    void showSpeed(float leftSpeed, float rightSpeed);
    void showDebug(String msg);
    void showStartup();
    void showFinished();
    
    // 获取显示对象用于自定义绘制
    Adafruit_SSD1306* getDisplay() { return display; }
    
private:
    Adafruit_SSD1306* display;
    TwoWire* wire2;
    
    String stateToString(SystemState state);
};

#endif
