# 智能循迹小车二次开发文档

本文档旨在帮助开发者理解项目架构，并进行二次开发（如添加新功能、修改控制逻辑、增加参数等）。

## 1. 项目简介

本项目是一个基于 ESP32-S3 的智能循迹小车系统，具备以下核心功能：
- **高精度循迹**：基于 PID 算法的黑线循迹。
- **多阶段控制**：支持测距前、测距后使用不同的 PID 和速度参数。
- **智能避障**：基于超声波和编码器的多步避障逻辑（左转->直行->平行->回线）。
- **物块测量**：使用激光测距传感器测量路边物块长度。
- **自动入库**：识别终点并自动入库停车。
- **Web 交互**：通过 WiFi 提供 Web 界面，支持实时参数调整、状态监控和日志查看。

## 2. 开发环境搭建

1.  **IDE**: Visual Studio Code
2.  **插件**: PlatformIO IDE
3.  **硬件**: 4D Systems GEN4-ESP32 16MB (ESP32S3-R8N16) 或兼容 ESP32S3 开发板。
4.  **依赖库** (自动安装):
    - `ESP32Encoder`, `QuickPID` (运动控制)
    - `Adafruit_VL53L0X`, `NewPing` (传感器)
    - `ESPAsyncWebServer`, `ArduinoJson` (网络与数据)
    - `Adafruit SSD1306`, `Adafruit GFX` (OLED 显示)

## 3. 项目结构说明

```text
├── platformio.ini      # PlatformIO 配置文件 (依赖、板卡定义)
├── src/                # 源代码目录
│   ├── main.cpp        # 主程序入口，包含状态机和主循环
│   ├── config.h        # 全局硬件引脚定义和常量
│   ├── ParameterManager.*  # 参数管理 (NVS存储、JSON序列化)
│   ├── WebServerManager.*  # Web 服务器与 WebSocket 通信
│   ├── MotorControl.*      # 电机底层驱动与编码器读取
│   ├── PIDController.*     # PID 算法实现
│   ├── LineSensor.*        # 循迹传感器处理
│   ├── Sensors.*           # 综合传感器管理 (激光、超声波等)
│   ├── ObjectDetector.*    # 物块检测与测量逻辑
│   ├── TaskManager.*       # 任务队列管理
│   └── Display.*           # OLED 显示管理
└── include/            # 头文件目录
```

## 4. 核心架构解析

### 4.1 系统状态机 (`main.cpp`)
系统通过 `SystemState` 枚举管理运行状态：
- `STATE_IDLE`: 待机状态，电机停止。
- `STATE_LINE_FOLLOW`: 循迹模式，核心控制逻辑。
- `STATE_OBSTACLE_AVOID`: 避障模式，执行多步避障动作。
- `STATE_PARKING`: 入库模式，执行停车逻辑。
- `STATE_TESTING`: 测试模式，用于调试特定动作（如转弯、直行）。

### 4.2 参数管理系统 (`ParameterManager`)
所有可配置参数（PID、速度、阈值等）都存储在 `ParameterManager` 中。
- **存储**: 使用 ESP32 `Preferences` (NVS) 掉电保存。
- **交互**: 提供 `toJson()` 和 `fromJson()` 方法，方便与 Web 端交互。

### 4.3 Web 交互系统 (`WebServerManager`)
- 基于 `ESPAsyncWebServer`。
- 前端 HTML/JS 硬编码在 `generateHTML()` 中（为了单文件部署便利）。
- 通过 HTTP GET/POST 接口交换 JSON 数据。

## 5. 常见开发场景指南

### 5.1 如何添加一个新的配置参数？

如果你需要添加一个新的参数（例如 `newParam`），请遵循以下步骤：

1.  **修改 `src/ParameterManager.h`**:
    在 `Parameters` 结构体中添加变量：
    ```cpp
    struct Parameters {
        // ... existing params
        float newParam; // [新增]
    };
    ```

2.  **修改 `src/ParameterManager.cpp`**:
    - 在构造函数中设置默认值：
      ```cpp
      newParam = 1.0f;
      ```
    - 在 `load()` 方法中读取 NVS：
      ```cpp
      newParam = prefs.getFloat("newParam", 1.0f);
      ```
    - 在 `save()` 方法中写入 NVS：
      ```cpp
      prefs.putFloat("newParam", newParam);
      ```
    - 在 `toJson()` 方法中添加到 JSON：
      ```cpp
      doc["newParam"] = newParam;
      ```
    - 在 `fromJson()` 方法中从 JSON 读取：
      ```cpp
      if (doc.containsKey("newParam")) newParam = doc["newParam"];
      ```

3.  **修改 `src/WebServerManager.cpp`**:
    在 `generateHTML()` 的 JavaScript 部分，找到 `loadParams` 和 `saveParams` 函数，添加对应的字段映射，并在 HTML 表单中添加 `<input>` 元素。

### 5.2 如何修改避障逻辑？

避障逻辑位于 `src/main.cpp` 的 `handleObstacleAvoidance()` 函数中。
它是一个基于 `avoidSubState` 的子状态机：
1.  `AVOID_TURN_LEFT`: 左转离开赛道。
2.  `AVOID_FORWARD_OUT`: 直行一段距离。
3.  `AVOID_TURN_RIGHT_1`: 右转平行。
4.  ...

**修改建议**:
- 调整 `ParameterManager` 中的 `avoid*` 相关参数（如 `avoidTurn1Dist`）通常能满足大部分需求。
- 如果需要修改动作顺序，请直接修改 `switch(avoidSubState)` 中的逻辑跳转。

### 5.3 如何添加新的传感器？

1.  **硬件连接**: 确定引脚并在 `src/config.h` 中定义。
2.  **驱动封装**: 建议在 `src/Sensors.h/.cpp` 中添加初始化和读取代码，或者创建新的类。
3.  **数据集成**: 在 `src/main.cpp` 的 `updateSensors()` 中调用更新，并通过 `getSystemStatus()` 将数据发送到 Web 端以便调试。

## 6. 调试技巧

- **串口调试**: 波特率 `115200`。系统启动时会打印详细信息。
- **Web 日志**: 浏览器访问小车 IP，页面底部有实时 Console 日志，通过 `webServer.addLog()` 输出。
- **OLED 显示**: 实时显示 PID 输出、传感器状态和 IP 地址。
- **状态监控**: Web 界面会以 5Hz 频率刷新 JSON 状态，包含所有传感器数据和内部变量。

## 7. 编译与上传

在 VS Code 终端中运行：

- **编译**: `platformio run`
- **上传**: `platformio run --target upload`
- **清理**: `platformio run --target clean`

## 8. 注意事项

- **并发安全**: `WebServer` 回调在独立任务中运行，修改全局变量（如 `currentState`）时需注意线程安全，或使用标志位（如 `pendingManualCmd`）在主循环中处理。
- **NVS 寿命**: 避免在循环中高频调用 `params.save()`，这会损耗 Flash 寿命。仅在 Web 端点击“保存”时调用。
