#include "Display.h"

Display::Display() {
    wire2 = new TwoWire(1);
    display = nullptr;
}

void Display::begin() {
    // 初始化I2C2
    wire2->begin(PIN_I2C2_SDA, PIN_I2C2_SCL);
    
    // 初始化OLED
    display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, wire2, OLED_RESET);
    
    if (!display->begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("SSD1306 init failed!");
        return;
    }
    
    display->clearDisplay();
    display->setTextSize(1);
    display->setTextColor(SSD1306_WHITE);
    display->setCursor(0, 0);
    display->println("System Init...");
    display->display();
    
    Serial.println("Display initialized");
}

void Display::clear() {
    if (display) {
        display->clearDisplay();
    }
}

void Display::update() {
    if (display) {
        display->display();
    }
}

String Display::stateToString(SystemState state) {
    switch (state) {
        case STATE_IDLE: return "IDLE";
        case STATE_LINE_FOLLOW: return "LINE FOLLOW";
        default: return "UNKNOWN";
    }
}

void Display::showStatus(SystemState state, float linePos, float distance) {
    if (!display) return;
    
    clear();
    
    // 上半部分:状态信息
    display->setCursor(0, 0);
    display->setTextSize(1);
    
    display->print("State: ");
    display->println(stateToString(state));
    
    display->print("Line: ");
    display->println(linePos, 1);
    
    display->print("Dist: ");
    display->print(distance, 1);
    display->println(" cm");
    
    // 分隔线
    display->drawLine(0, 32, 128, 32, SSD1306_WHITE);
}

void Display::showMeasurement(float objectLength) {
    if (!display) return;
    
    // 下半部分:物体测量结果
    display->setCursor(0, 38);
    display->setTextSize(1);
    display->print("Object:");
    display->setCursor(0, 48);
    display->setTextSize(2);
    display->print(objectLength, 0);
    display->println("mm");
}

void Display::showSpeed(float leftSpeed, float rightSpeed) {
    if (!display) return;
    
    display->setCursor(0, 48);
    display->setTextSize(1);
    display->print("L:");
    display->print(leftSpeed, 0);
    display->print(" R:");
    display->println(rightSpeed, 0);
}

void Display::showDebug(String msg) {
    if (!display) return;
    
    clear();
    display->setCursor(0, 0);
    display->setTextSize(1);
    display->println(msg);
    update();
}

void Display::showStartup() {
    if (!display) return;
    
    clear();
    display->setCursor(10, 10);
    display->setTextSize(2);
    display->println("READY!");
    display->setCursor(0, 40);
    display->setTextSize(1);
    display->println("Press button to start");
    update();
}

void Display::showFinished() {
    if (!display) return;
    
    clear();
    display->setCursor(20, 20);
    display->setTextSize(2);
    display->println("DONE!");
    update();
}
