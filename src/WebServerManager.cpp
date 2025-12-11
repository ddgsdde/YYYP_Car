#include "WebServerManager.h"

WebServerManager::WebServerManager(ParameterManager* params) {
    paramManager = params;
    server = new AsyncWebServer(WEB_SERVER_PORT);
    // statusCallback = nullptr; // 移除
    motionCallback = nullptr;
    weightCallback = nullptr;
    calibrationCallback = nullptr;
    detectionCallback = nullptr;
    taskCallback = nullptr;
    logIndex = 0;
    logCount = 0;
    
    // 初始化互斥锁
    mutex = xSemaphoreCreateMutex();
    currentStatusJson = "{\"status\":\"initializing\"}";
}

void WebServerManager::begin() {
    // 先尝试连接WiFi
    Serial.println("Connecting to WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < WIFI_CONNECT_TIMEOUT) {
        delay(100);
        Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        // 成功连接到WiFi
        Serial.println("\n✓ WiFi connected!");
        Serial.print("✓ IP: ");
        Serial.println(WiFi.localIP());
    } else {
        // 连接失败，创建AP热点
        Serial.println("\n✗ WiFi connection failed, starting AP mode...");
        WiFi.mode(WIFI_AP);
        WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
        
        Serial.println("✓ WiFi AP started");
        Serial.print("✓ SSID: ");
        Serial.println(WIFI_AP_SSID);
        Serial.print("✓ IP: ");
        Serial.println(WiFi.softAPIP());
    }
    
    setupRoutes();
    server->begin();
    Serial.println("✓ Web server started");
}

// void WebServerManager::setStatusCallback(String (*callback)()) {
//     statusCallback = callback;
// }

void WebServerManager::updateStatusJson(const String& json) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        currentStatusJson = json;
        xSemaphoreGive(mutex);
    }
}

void WebServerManager::setMotionCallback(void (*callback)(String action, float value)) {
    motionCallback = callback;
}

void WebServerManager::setWeightCallback(void (*callback)(int16_t weights[8])) {
    weightCallback = callback;
}

void WebServerManager::setCalibrationCallback(void (*callback)(float leftCalib, float rightCalib)) {
    calibrationCallback = callback;
}

void WebServerManager::setDetectionCallback(void (*callback)(uint16_t baseline, uint16_t threshold)) {
    detectionCallback = callback;
}

void WebServerManager::setTaskCallback(String (*callback)(String action, String data)) {
    taskCallback = callback;
}

String WebServerManager::getIPAddress() {
    if (WiFi.getMode() == WIFI_STA && WiFi.status() == WL_CONNECTED) {
        return WiFi.localIP().toString();
    } else {
        return WiFi.softAPIP().toString();
    }
}

void WebServerManager::setupRoutes() {
    // 主页
    server->on("/", HTTP_GET, [this](AsyncWebServerRequest *request){
        request->send(200, "text/html", generateHTML());
    });
    
    // 获取参数
    server->on("/api/params", HTTP_GET, [this](AsyncWebServerRequest *request){
        request->send(200, "application/json", paramManager->toJson());
    });
    
    // 更新参数
    server->on("/api/params", HTTP_POST, [](AsyncWebServerRequest *request){}, 
        NULL, 
        [this](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
            String json = "";
            if (len > 0) {
                char* temp = (char*)malloc(len + 1);
                if (temp) {
                    memcpy(temp, data, len);
                    temp[len] = '\0';
                    json = String(temp);
                    free(temp);
                }
            }
            paramManager->fromJson(json);
            request->send(200, "application/json", "{\"status\":\"ok\"}");
        }
    );
    
    // 重置参数
    server->on("/api/reset", HTTP_POST, [this](AsyncWebServerRequest *request){
        paramManager->reset();
        request->send(200, "application/json", "{\"status\":\"ok\"}");
    });
    
    // 获取实时状态
    server->on("/api/status", HTTP_GET, [this](AsyncWebServerRequest *request){
        String json = "{}";
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            json = currentStatusJson;
            xSemaphoreGive(mutex);
        }
        request->send(200, "application/json", json);
    });
    
    // 传感器测试
    server->on("/api/test/sensors", HTTP_GET, [this](AsyncWebServerRequest *request){
        String json = "{}";
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            json = currentStatusJson;
            xSemaphoreGive(mutex);
        }
        request->send(200, "application/json", json);
    });
    
    // 运动控制
    server->on("/api/motion", HTTP_POST, [](AsyncWebServerRequest *request){}, 
        NULL, 
        [this](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
            // 安全构建字符串
            String json = "";
            if (len > 0) {
                char* temp = (char*)malloc(len + 1);
                if (temp) {
                    memcpy(temp, data, len);
                    temp[len] = '\0';
                    json = String(temp);
                    free(temp);
                }
            }
            
            // 解析JSON
            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, json);
            
            if (!error && motionCallback) {
                String action = doc["action"].as<String>();
                float value = doc["value"] | 0.0;
                
                motionCallback(action, value);
                request->send(200, "application/json", "{\"status\":\"ok\",\"action\":\"" + action + "\"}");
            } else {
                request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
            }
        }
    );
    
    // 传感器权重配置
    server->on("/api/weights", HTTP_POST, [](AsyncWebServerRequest *request){}, 
        NULL, 
        [this](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
            String json = "";
            if (len > 0) {
                char* temp = (char*)malloc(len + 1);
                if (temp) {
                    memcpy(temp, data, len);
                    temp[len] = '\0';
                    json = String(temp);
                    free(temp);
                }
            }
            
            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, json);
            
            if (!error && weightCallback) {
                int16_t weights[8];
                for (int i = 0; i < 8; i++) {
                    weights[i] = doc["weights"][i] | 0;
                    // 更新参数管理器中的权重
                    paramManager->sensorWeights[i] = weights[i];
                }
                // 保存参数
                paramManager->save();
                
                weightCallback(weights);
                request->send(200, "application/json", "{\"status\":\"ok\"}");
            } else {
                request->send(400, "application/json", "{\"status\":\"error\"}");
            }
        }
    );
    
    // 电机校准
    server->on("/api/calibration", HTTP_POST, [](AsyncWebServerRequest *request){}, 
        NULL, 
        [this](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
            String json = "";
            if (len > 0) {
                char* temp = (char*)malloc(len + 1);
                if (temp) {
                    memcpy(temp, data, len);
                    temp[len] = '\0';
                    json = String(temp);
                    free(temp);
                }
            }
            
            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, json);
            
            if (!error && calibrationCallback) {
                float leftCalib = doc["leftCalib"] | 1.0;
                float rightCalib = doc["rightCalib"] | 1.0;
                
                calibrationCallback(leftCalib, rightCalib);
                request->send(200, "application/json", "{\"status\":\"ok\"}");
            } else {
                request->send(400, "application/json", "{\"status\":\"error\"}");
            }
        }
    );
    
    // 物块检测API
    server->on("/api/detection/start", HTTP_POST, [](AsyncWebServerRequest *request){}, 
        NULL, 
        [this](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
            String json = "";
            if (len > 0) {
                char* temp = (char*)malloc(len + 1);
                if (temp) {
                    memcpy(temp, data, len);
                    temp[len] = '\0';
                    json = String(temp);
                    free(temp);
                }
            }
            
            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, json);
            
            if (!error && detectionCallback) {
                uint16_t baseline = doc["baseline"] | 800;
                uint16_t threshold = doc["threshold"] | 100;
                
                // 同时更新参数
                if (doc["filter"]) paramManager->objectFilterSize = doc["filter"];
                if (doc["scale"]) paramManager->objectLengthScale = doc["scale"];
                if (doc["offset"]) paramManager->objectLengthOffset = doc["offset"];
                if (doc["devCorr"]) paramManager->objectDeviationCorrection = doc["devCorr"];
                paramManager->save();
                
                detectionCallback(baseline, threshold);
                request->send(200, "application/json", "{\"status\":\"ok\"}");
            } else {
                request->send(400, "application/json", "{\"status\":\"error\"}");
            }
        }
    );
    
    // 停止检测
    server->on("/api/detection/stop", HTTP_POST, [this](AsyncWebServerRequest *request){
        if (detectionCallback) {
            detectionCallback(0, 0);  // 传0表示停止
            request->send(200, "application/json", "{\"status\":\"ok\"}");
        } else {
            request->send(400, "application/json", "{\"status\":\"error\"}");
        }
    });
    
    // 任务管理API
    server->on("/api/tasks", HTTP_GET, [this](AsyncWebServerRequest *request){
        if (taskCallback) {
            String result = taskCallback("get", "");
            request->send(200, "application/json", result);
        } else {
            request->send(200, "application/json", "{\"tasks\":[]}");
        }
    });
    
    server->on("/api/tasks", HTTP_POST, [](AsyncWebServerRequest *request){}, 
        NULL, 
        [this](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
            // 安全地构建字符串，避免读取未分配的内存
            String json = "";
            if (len > 0) {
                // 分配临时缓冲区，多一个字节用于null终止符
                char* temp = (char*)malloc(len + 1);
                if (temp) {
                    memcpy(temp, data, len);
                    temp[len] = '\0'; // 确保null终止
                    json = String(temp);
                    free(temp);
                }
            }
            
            String action = "set";
            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, json);
            if (!error && doc.containsKey("action")) {
                String act = doc["action"].as<String>();
                if (act == "test_turn" || act == "test_straight" || act == "test_avoid" || act == "test_parking") {
                    action = act;
                }
            }
            
            if (taskCallback) {
                String result = taskCallback(action, json);
                request->send(200, "application/json", result);
            } else {
                request->send(400, "application/json", "{\"status\":\"error\"}");
            }
        }
    );
    
    server->on("/api/tasks/start", HTTP_POST, [this](AsyncWebServerRequest *request){
        if (taskCallback) {
            taskCallback("start", "");
            request->send(200, "application/json", "{\"status\":\"ok\"}");
        } else {
            request->send(400, "application/json", "{\"status\":\"error\"}");
        }
    });
    
    server->on("/api/tasks/stop", HTTP_POST, [this](AsyncWebServerRequest *request){
        if (taskCallback) {
            taskCallback("stop", "");
            request->send(200, "application/json", "{\"status\":\"ok\"}");
        } else {
            request->send(400, "application/json", "{\"status\":\"error\"}");
        }
    });
    
    server->on("/api/tasks/clear", HTTP_POST, [this](AsyncWebServerRequest *request){
        if (taskCallback) {
            taskCallback("clear", "");
            request->send(200, "application/json", "{\"status\":\"ok\"}");
        } else {
            request->send(400, "application/json", "{\"status\":\"error\"}");
        }
    });
    
    // 获取日志
    server->on("/api/logs", HTTP_GET, [this](AsyncWebServerRequest *request){
        request->send(200, "application/json", getLogs());
    });
    
    // 清空日志
    server->on("/api/logs/clear", HTTP_POST, [this](AsyncWebServerRequest *request){
        clearLogs();
        request->send(200, "application/json", "{\"status\":\"ok\"}");
    });
}

String WebServerManager::generateHTML() {
    return R"rawliteral(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>CYBER-TRACKER // 智能车控制台</title>
    <style>
        :root {
            --bg-color: #050505;
            --card-bg: #0a0a0f;
            --card-border: #1f1f2e;
            --primary: #00f3ff;
            --primary-dim: rgba(0, 243, 255, 0.1);
            --secondary: #ff0055;
            --secondary-dim: rgba(255, 0, 85, 0.1);
            --accent: #ffee00;
            --text-main: #e0e0e0;
            --text-dim: #888;
            --success: #00ff9d;
            --warning: #ffb800;
            --danger: #ff2a2a;
            --font-tech: 'Segoe UI', 'Roboto', Helvetica, Arial, sans-serif;
            --font-mono: 'Consolas', 'Monaco', monospace;
        }

        * { margin: 0; padding: 0; box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
        
        body { 
            font-family: var(--font-tech);
            background-color: var(--bg-color);
            color: var(--text-main);
            min-height: 100vh;
            background-image: 
                linear-gradient(rgba(0, 243, 255, 0.03) 1px, transparent 1px),
                linear-gradient(90deg, rgba(0, 243, 255, 0.03) 1px, transparent 1px);
            background-size: 40px 40px;
            padding-bottom: 80px; /* Space for bottom bar */
            overflow-x: hidden;
        }

        /* Cyberpunk Scrollbar */
        ::-webkit-scrollbar { width: 8px; }
        ::-webkit-scrollbar-track { background: var(--bg-color); }
        ::-webkit-scrollbar-thumb { background: var(--card-border); border: 1px solid var(--primary); }
        ::-webkit-scrollbar-thumb:hover { background: var(--primary); }

        .container { 
            max-width: 1400px; 
            margin: 0 auto; 
            padding: 15px;
        }

        /* Header */
        .header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 20px 0;
            margin-bottom: 20px;
            border-bottom: 1px solid var(--primary);
            position: relative;
        }
        .header::after {
            content: ''; position: absolute; bottom: -1px; right: 0; width: 30%; height: 1px;
            background: var(--secondary); box-shadow: 0 0 10px var(--secondary);
        }
        .brand {
            font-size: 1.5rem;
            font-weight: 900;
            letter-spacing: 2px;
            color: var(--primary);
            text-transform: uppercase;
            text-shadow: 0 0 10px var(--primary-dim);
            display: flex;
            align-items: center;
            gap: 10px;
        }
        .brand span { color: var(--text-main); font-size: 0.8em; opacity: 0.7; font-weight: normal; }

        /* Grid Layout */
        .dashboard-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
            gap: 20px;
        }
        @media (max-width: 768px) {
            .dashboard-grid { grid-template-columns: 1fr; }
            .header { flex-direction: column; align-items: flex-start; gap: 10px; }
        }

        /* Cyber Card */
        .cyber-card {
            background: var(--card-bg);
            border: 1px solid var(--card-border);
            position: relative;
            padding: 20px;
            transition: transform 0.3s ease, box-shadow 0.3s ease;
            clip-path: polygon(
                0 0, 
                100% 0, 
                100% calc(100% - 15px), 
                calc(100% - 15px) 100%, 
                0 100%
            );
        }
        .cyber-card::before {
            content: ''; position: absolute; top: 0; left: 0; width: 4px; height: 100%;
            background: var(--primary); opacity: 0.5;
        }
        .cyber-card:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 20px rgba(0,0,0,0.5);
            border-color: var(--primary);
        }
        .cyber-card h2 {
            font-size: 1.1rem;
            color: var(--primary);
            margin-bottom: 20px;
            text-transform: uppercase;
            letter-spacing: 1px;
            display: flex;
            align-items: center;
            gap: 10px;
            border-bottom: 1px solid var(--card-border);
            padding-bottom: 10px;
        }
        .cyber-card.danger-zone::before { background: var(--secondary); }
        .cyber-card.danger-zone h2 { color: var(--secondary); }

        /* Inputs */
        .param-grid {
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(140px, 1fr));
            gap: 15px;
        }
        .input-group {
            position: relative;
        }
        .input-group label {
            display: block;
            font-size: 0.75rem;
            color: var(--text-dim);
            margin-bottom: 5px;
            text-transform: uppercase;
        }
        .cyber-input {
            width: 100%;
            background: rgba(0,0,0,0.3);
            border: 1px solid var(--card-border);
            color: var(--primary);
            padding: 8px 10px;
            font-family: var(--font-mono);
            font-size: 1rem;
            transition: all 0.3s;
        }
        .cyber-input:focus {
            outline: none;
            border-color: var(--primary);
            box-shadow: 0 0 10px var(--primary-dim);
            background: rgba(0, 243, 255, 0.05);
        }

        /* Buttons */
        .cyber-btn {
            background: transparent;
            border: 1px solid var(--primary);
            color: var(--primary);
            padding: 10px 20px;
            font-family: var(--font-tech);
            font-weight: bold;
            text-transform: uppercase;
            letter-spacing: 1px;
            cursor: pointer;
            transition: all 0.2s;
            position: relative;
            overflow: hidden;
            display: inline-flex;
            align-items: center;
            justify-content: center;
            gap: 8px;
            width: 100%;
        }
        .cyber-btn:hover {
            background: var(--primary);
            color: #000;
            box-shadow: 0 0 15px var(--primary);
        }
        .cyber-btn:active { transform: scale(0.98); }
        
        .cyber-btn.secondary { border-color: var(--text-dim); color: var(--text-dim); }
        .cyber-btn.secondary:hover { background: var(--text-dim); color: #000; box-shadow: 0 0 15px rgba(255,255,255,0.2); }
        
        .cyber-btn.danger { border-color: var(--secondary); color: var(--secondary); }
        .cyber-btn.danger:hover { background: var(--secondary); color: #fff; box-shadow: 0 0 15px var(--secondary); }

        .btn-row { display: flex; gap: 10px; margin-top: 15px; }

        /* Sensor Status */
        .sensor-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(100px, 1fr));
            gap: 10px;
        }
        .sensor-box {
            background: rgba(255,255,255,0.03);
            border: 1px solid var(--card-border);
            padding: 10px;
            text-align: center;
            position: relative;
        }
        .sensor-box.active { border-color: var(--success); color: var(--success); box-shadow: inset 0 0 10px rgba(0,255,157,0.1); }
        .sensor-box.warning { border-color: var(--warning); color: var(--warning); }
        .sensor-box.error { border-color: var(--danger); color: var(--danger); animation: flash 1s infinite; }
        .sensor-icon { font-size: 1.5rem; margin-bottom: 5px; display: block; }
        .sensor-val { font-family: var(--font-mono); font-size: 0.9rem; font-weight: bold; }
        .sensor-label { font-size: 0.7rem; opacity: 0.7; margin-top: 3px; }

        /* Motion Control Pad */
        .d-pad {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            max-width: 300px;
            margin: 0 auto;
        }
        .d-pad button { aspect-ratio: 1; font-size: 1.5rem; }

        /* Logs */
        .log-terminal {
            background: #000;
            border: 1px solid var(--card-border);
            padding: 10px;
            font-family: var(--font-mono);
            font-size: 0.8rem;
            height: 200px;
            overflow-y: auto;
            color: var(--text-dim);
            border-left: 3px solid var(--accent);
        }
        .log-line { margin-bottom: 2px; border-bottom: 1px solid rgba(255,255,255,0.05); padding-bottom: 2px; }

        /* Toast */
        .toast-container {
            position: fixed; top: 20px; right: 20px; z-index: 9999;
        }
        .cyber-toast {
            background: rgba(0,0,0,0.9);
            border: 1px solid var(--primary);
            color: var(--primary);
            padding: 15px 25px;
            margin-bottom: 10px;
            backdrop-filter: blur(5px);
            box-shadow: 0 5px 15px rgba(0,0,0,0.5);
            transform: translateX(120%);
            transition: transform 0.3s cubic-bezier(0.68, -0.55, 0.27, 1.55);
            display: flex; align-items: center; gap: 10px;
        }
        .cyber-toast.show { transform: translateX(0); }
        .cyber-toast.error { border-color: var(--danger); color: var(--danger); }
        .cyber-toast.success { border-color: var(--success); color: var(--success); }

        /* Animations */
        @keyframes flash { 0%, 100% { opacity: 1; } 50% { opacity: 0.5; } }
        @keyframes scan { 0% { background-position: 0 0; } 100% { background-position: 0 100%; } }

        /* Bottom Action Bar */
        .bottom-bar {
            position: fixed; bottom: 0; left: 0; width: 100%;
            background: rgba(10,10,15,0.95);
            border-top: 1px solid var(--card-border);
            padding: 10px 20px;
            display: flex; justify-content: center; gap: 15px;
            backdrop-filter: blur(10px);
            z-index: 100;
        }
        .bottom-bar button { flex: 1; max-width: 200px; }

        /* Range Slider Style */
        input[type=range] {
            -webkit-appearance: none; width: 100%; background: transparent;
        }
        input[type=range]::-webkit-slider-thumb {
            -webkit-appearance: none; height: 16px; width: 16px;
            background: var(--primary); cursor: pointer; margin-top: -6px;
            box-shadow: 0 0 10px var(--primary);
        }
        input[type=range]::-webkit-slider-runnable-track {
            width: 100%; height: 4px; cursor: pointer;
            background: var(--card-border);
        }
        
        /* Table */
        .cyber-table { width: 100%; border-collapse: collapse; font-size: 0.85rem; }
        .cyber-table th { text-align: left; color: var(--text-dim); padding: 8px; border-bottom: 1px solid var(--card-border); }
        .cyber-table td { padding: 8px; border-bottom: 1px solid rgba(255,255,255,0.05); }
        .cyber-table input { width: 60px; padding: 4px; text-align: center; }

        /* Status Badge */
        .status-badge {
            padding: 5px 10px; border: 1px solid var(--primary); 
            color: var(--primary); font-size: 0.8rem; text-transform: uppercase;
            background: rgba(0, 243, 255, 0.1);
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <div class="brand">
                <div>CYBER<span style="color:var(--secondary)">TRACKER</span></div>
                <span>v2.0</span>
            </div>
            <div id="connectionStatus" class="status-badge">SYSTEM ONLINE</div>
        </div>

        <div class="dashboard-grid">
            <!-- Column 1: Sensors & Status -->
            <div style="display: flex; flex-direction: column; gap: 20px;">
                <!-- Sensor Status -->
                <div class="cyber-card">
                    <h2>📡 传感器阵列</h2>
                    <div class="sensor-grid">
                        <div class="sensor-box" id="card-line">
                            <span class="sensor-icon">👁️</span>
                            <div class="sensor-val" id="line-value">--</div>
                            <div class="sensor-label">循迹</div>
                        </div>
                        <div class="sensor-box" id="card-laser">
                            <span class="sensor-icon">📏</span>
                            <div class="sensor-val" id="laser-value">--</div>
                            <div class="sensor-label">激光</div>
                        </div>
                        <div class="sensor-box" id="card-ultra">
                            <span class="sensor-icon">🦇</span>
                            <div class="sensor-val" id="ultra-value">--</div>
                            <div class="sensor-label">超声波</div>
                        </div>
                        <div class="sensor-box" id="card-encoder-l">
                            <span class="sensor-icon">⚙️L</span>
                            <div class="sensor-val" id="encoder-l-value">--</div>
                            <div class="sensor-label" id="encoder-l-pulse" style="font-size: 0.7rem; color: var(--text-dim);">-- P</div>
                            <div class="sensor-label">左编码器</div>
                        </div>
                        <div class="sensor-box" id="card-encoder-r">
                            <span class="sensor-icon">⚙️R</span>
                            <div class="sensor-val" id="encoder-r-value">--</div>
                            <div class="sensor-label" id="encoder-r-pulse" style="font-size: 0.7rem; color: var(--text-dim);">-- P</div>
                            <div class="sensor-label">右编码器</div>
                        </div>
                    </div>
                    <button class="cyber-btn secondary" onclick="testAllSensors()" style="margin-top: 15px; font-size: 0.8rem;">
                        🔄 刷新传感器状态
                    </button>
                </div>

                <!-- Manual Control -->
                <div class="cyber-card danger-zone">
                    <h2>🎮 手动超控</h2>
                    <div class="d-pad">
                        <div></div>
                        <button class="cyber-btn" 
                            onmousedown="startMotion('forward')" onmouseup="stopMotion()" onmouseleave="stopMotion()"
                            ontouchstart="startMotion('forward')" ontouchend="stopMotion()">▲</button>
                        <div></div>
                        
                        <button class="cyber-btn" 
                            onmousedown="startMotion('left')" onmouseup="stopMotion()" onmouseleave="stopMotion()"
                            ontouchstart="startMotion('left')" ontouchend="stopMotion()">◀</button>
                        <button class="cyber-btn danger" 
                            onmousedown="stopMotion()" ontouchstart="stopMotion()">■</button>
                        <button class="cyber-btn" 
                            onmousedown="startMotion('right')" onmouseup="stopMotion()" onmouseleave="stopMotion()"
                            ontouchstart="startMotion('right')" ontouchend="stopMotion()">▶</button>
                        
                        <div></div>
                        <button class="cyber-btn" 
                            onmousedown="startMotion('backward')" onmouseup="stopMotion()" onmouseleave="stopMotion()"
                            ontouchstart="startMotion('backward')" ontouchend="stopMotion()">▼</button>
                        <div></div>
                    </div>
                    <div class="btn-row">
                        <button class="cyber-btn secondary" onclick="sendMotionCommand('turn_180', false)">↻ 180°掉头</button>
                    </div>
                </div>

                <!-- Logs -->
                <div class="cyber-card">
                    <h2>📋 系统日志</h2>
                    <div class="log-terminal" id="logDisplay">
                        > System initializing...
                    </div>
                    <div class="btn-row" style="margin-top: 10px;">
                        <button class="cyber-btn secondary" onclick="clearLogs()" style="font-size: 0.8rem;">清空</button>
                        <button class="cyber-btn secondary" onclick="toggleAutoScroll()" id="autoScrollBtn" style="font-size: 0.8rem;">滚动: ON</button>
                    </div>
                </div>
            </div>

            <!-- Column 2: Parameters -->
            <div style="display: flex; flex-direction: column; gap: 20px;">
                <!-- PID Config -->
                <div class="cyber-card">
                    <h2>🎯 PID 控制核心</h2>
                    <div class="param-grid">
                        <div class="input-group">
                            <label>Kp (比例)</label>
                            <input type="number" id="kp" class="cyber-input" step="0.01">
                        </div>
                        <div class="input-group">
                            <label>Ki (积分)</label>
                            <input type="number" id="ki" class="cyber-input" step="0.001">
                        </div>
                        <div class="input-group">
                            <label>Kd (微分)</label>
                            <input type="number" id="kd" class="cyber-input" step="0.1">
                        </div>
                    </div>
                    
                    <div style="margin: 10px 0; border-top: 1px dashed var(--card-border); padding-top: 5px;">
                        <div style="font-size: 0.8rem; color: var(--text-dim); margin-bottom: 5px;">Phase 2: 测距后参数</div>
                        <div class="param-grid">
                            <div class="input-group"><label>Kp (后)</label><input type="number" id="kpPost" class="cyber-input" step="0.01"></div>
                            <div class="input-group"><label>Ki (后)</label><input type="number" id="kiPost" class="cyber-input" step="0.001"></div>
                            <div class="input-group"><label>Kd (后)</label><input type="number" id="kdPost" class="cyber-input" step="0.1"></div>
                        </div>
                    </div>
                    
                    <details style="margin-top: 15px; border-top: 1px dashed var(--card-border); padding-top: 10px;">
                        <summary style="color: var(--primary); cursor: pointer; font-size: 0.9rem;">⚙️ 高级 PID 设置</summary>
                        <div class="param-grid" style="margin-top: 15px;">
                            <div class="input-group"><label>积分分离</label><input type="number" id="pidIntRange" class="cyber-input"></div>
                            <div class="input-group"><label>电机死区</label><input type="number" id="motorDeadband" class="cyber-input"></div>
                            <div class="input-group"><label>直线阈值</label><input type="number" id="pidSmallErrorThres" class="cyber-input"></div>
                            <div class="input-group"><label>直线Kp缩放</label><input type="number" id="pidKpSmallScale" class="cyber-input" step="0.1"></div>
                            <div class="input-group"><label>直线Kd缩放</label><input type="number" id="pidKdSmallScale" class="cyber-input" step="0.1"></div>
                        </div>
                    </details>
                </div>

                <!-- Speed Config -->
                <div class="cyber-card">
                    <h2>⚡ 动力分配 (PWM)</h2>
                    <div class="param-grid">
                        <div class="input-group"><label>慢速 (搜索)</label><input type="number" id="speedSlow" class="cyber-input"></div>
                        <div class="input-group"><label>正常 (巡航)</label><input type="number" id="speedNormal" class="cyber-input"></div>
                        <div class="input-group"><label>快速 (直线)</label><input type="number" id="speedFast" class="cyber-input"></div>
                        <div class="input-group"><label>转弯 (机动)</label><input type="number" id="speedTurn" class="cyber-input"></div>
                    </div>
                    <div style="margin: 10px 0; border-top: 1px dashed var(--card-border); padding-top: 5px;">
                        <div style="font-size: 0.8rem; color: var(--text-dim); margin-bottom: 5px;">Phase 2: 测距后速度</div>
                        <div class="param-grid">
                            <div class="input-group"><label>正常 (后)</label><input type="number" id="speedNormalPost" class="cyber-input"></div>
                            <div class="input-group"><label>快速 (后)</label><input type="number" id="speedFastPost" class="cyber-input"></div>
                            <div class="input-group"><label>转弯 (后)</label><input type="number" id="speedTurnPost" class="cyber-input"></div>
                        </div>
                    </div>
                </div>

                <!-- Avoidance Config -->
                <div class="cyber-card">
                    <h2>🚧 避障协议</h2>
                    <div class="param-grid">
                        <div class="input-group"><label>触发距离(cm)</label><input type="number" id="obstacleDetectDist" class="cyber-input"></div>
                        <div class="input-group"><label>直行速度</label><input type="number" id="avoidSpeed" class="cyber-input"></div>
                        <div class="input-group"><label>转弯速度</label><input type="number" id="avoidTurnSpeed" class="cyber-input"></div>
                        <div class="input-group"><label>修正Kp</label><input type="number" id="avoidKp" class="cyber-input" step="0.1"></div>
                    </div>
                    <div style="margin: 15px 0; height: 1px; background: var(--card-border);"></div>
                    <div class="param-grid">
                        <div class="input-group"><label>前进绕行(mm)</label><input type="number" id="avoidForwardDist" class="cyber-input"></div>
                        <div class="input-group"><label>平行移动(mm)</label><input type="number" id="avoidParallelDist" class="cyber-input"></div>
                        <div class="input-group"><label>90°基准(mm)</label><input type="number" id="turn90Dist" class="cyber-input"></div>
                        <div class="input-group"><label>最后回正(mm)</label><input type="number" id="avoidFinalTurnDist" class="cyber-input"></div>
                    </div>
                    <div class="param-grid" style="margin-top: 15px;">
                        <div class="input-group"><label>Step1 左转(mm)</label><input type="number" id="avoidTurn1Dist" class="cyber-input"></div>
                        <div class="input-group"><label>Step3 右转(mm)</label><input type="number" id="avoidTurn2Dist" class="cyber-input"></div>
                        <div class="input-group"><label>Step5 右转(mm)</label><input type="number" id="avoidTurn3Dist" class="cyber-input"></div>
                        <div class="input-group"><label>搜线距离(mm)</label><input type="number" id="avoidSearchDist" class="cyber-input"></div>
                    </div>
                    
                    <details style="margin-top: 15px;">
                        <summary style="color: var(--text-dim); cursor: pointer; font-size: 0.8rem;">⚙️ 步骤速度微调</summary>
                        <table class="cyber-table" style="margin-top: 10px;">
                            <tr><th>步骤</th><th>左系数</th><th>右系数</th></tr>
                            <tr><td>1.左转</td><td><input id="avS1L" class="cyber-input"></td><td><input id="avS1R" class="cyber-input"></td></tr>
                            <tr><td>2.直行Out</td><td><input id="avS2L" class="cyber-input"></td><td><input id="avS2R" class="cyber-input"></td></tr>
                            <tr><td>3.右转1</td><td><input id="avS3L" class="cyber-input"></td><td><input id="avS3R" class="cyber-input"></td></tr>
                            <tr><td>4.平行</td><td><input id="avS4L" class="cyber-input"></td><td><input id="avS4R" class="cyber-input"></td></tr>
                            <tr><td>5.右转2</td><td><input id="avS5L" class="cyber-input"></td><td><input id="avS5R" class="cyber-input"></td></tr>
                            <tr><td>6.直行In</td><td><input id="avS6L" class="cyber-input"></td><td><input id="avS6R" class="cyber-input"></td></tr>
                        </table>
                    </details>

                    <div class="btn-row">
                        <button class="cyber-btn" onclick="testTurn()">测试转弯</button>
                        <button class="cyber-btn" onclick="testStraight()">测试直线</button>
                        <button class="cyber-btn danger" onclick="testAvoid()">测试避障</button>
                    </div>
                </div>
            </div>

            <!-- Column 3: Advanced & Tools -->
            <div style="display: flex; flex-direction: column; gap: 20px;">
                <!-- Sensor Weights -->
                <div class="cyber-card">
                    <h2>⚖️ 传感器权重</h2>
                    <div class="param-grid" style="grid-template-columns: repeat(4, 1fr);">
                        <div class="input-group"><label>S0</label><input type="number" id="weight0" class="cyber-input"></div>
                        <div class="input-group"><label>S1</label><input type="number" id="weight1" class="cyber-input"></div>
                        <div class="input-group"><label>S2</label><input type="number" id="weight2" class="cyber-input"></div>
                        <div class="input-group"><label>S3</label><input type="number" id="weight3" class="cyber-input"></div>
                        <div class="input-group"><label>S4</label><input type="number" id="weight4" class="cyber-input"></div>
                        <div class="input-group"><label>S5</label><input type="number" id="weight5" class="cyber-input"></div>
                        <div class="input-group"><label>S6</label><input type="number" id="weight6" class="cyber-input"></div>
                        <div class="input-group"><label>S7</label><input type="number" id="weight7" class="cyber-input"></div>
                    </div>
                    <button class="cyber-btn" onclick="applyWeights()" style="margin-top: 15px;">应用权重</button>
                </div>

                <!-- Parking -->
                <div class="cyber-card">
                    <h2>🅿️ 自动泊车</h2>
                    <div class="param-grid">
                        <div class="input-group"><label>减速距离</label><input type="number" id="pkDistSlow" class="cyber-input"></div>
                        <div class="input-group"><label>极慢距离</label><input type="number" id="pkDistVSlow" class="cyber-input"></div>
                        <div class="input-group"><label>停止距离</label><input type="number" id="pkDistStop" class="cyber-input"></div>
                        <div class="input-group"><label>减速PWM</label><input type="number" id="pkSpdSlow" class="cyber-input"></div>
                        <div class="input-group"><label>极慢PWM</label><input type="number" id="pkSpdVSlow" class="cyber-input"></div>
                    </div>
                    <button class="cyber-btn" onclick="testParking()" style="margin-top: 15px;">测试入库</button>
                </div>

                <!-- Object Detection -->
                <div class="cyber-card">
                    <h2>📏 激光测距</h2>
                    <div style="text-align: center; margin-bottom: 15px;">
                        <div style="font-size: 2rem; color: var(--primary); font-weight: bold;" id="currentLaserDistance">-- mm</div>
                        <div style="font-size: 0.8rem; color: var(--text-dim);">实时距离</div>
                    </div>
                    <div class="param-grid">
                        <div class="input-group"><label>检测阈值</label><input type="number" id="detectionBaseline" class="cyber-input"></div>
                        <div class="input-group"><label>Scale</label><input type="number" id="objLengthScale" class="cyber-input" step="0.01"></div>
                        <div class="input-group"><label>Offset</label><input type="number" id="objLengthOffset" class="cyber-input" step="0.1"></div>
                        <div class="input-group"><label>DevCorr</label><input type="number" id="objDeviationCorrection" class="cyber-input" step="0.0001"></div>
                    </div>
                    
                    <div id="detectionResult" style="display: none; background: rgba(255,255,255,0.05); padding: 10px; margin: 10px 0; border-radius: 4px;">
                        <div style="display: flex; justify-content: space-between;">
                            <span>结果: <strong id="detectionLength" style="color:var(--success)">--</strong></span>
                            <span>状态: <strong id="detectionStatus">--</strong></span>
                        </div>
                        <div style="font-size: 0.8rem; color: var(--text-dim); margin-top: 5px;">
                            原始: <span id="detectionRawLength">--</span> | 侧距: <span id="detectionAvgDist">--</span> | 耗时: <span id="detectionDuration">--</span>
                        </div>
                    </div>

                    <div class="btn-row">
                        <button class="cyber-btn" onclick="startObjectDetection()">开始测量</button>
                        <button class="cyber-btn danger" onclick="stopDetection()">停止</button>
                        <button class="cyber-btn secondary" onclick="resetDetection()">重置</button>
                    </div>
                </div>

                <!-- Calibration -->
                <div class="cyber-card">
                    <h2>🔧 电机校准</h2>
                    <div class="param-grid">
                        <div class="input-group"><label>左系数</label><input type="number" id="motorLeftCalib" class="cyber-input" step="0.01"></div>
                        <div class="input-group"><label>右系数</label><input type="number" id="motorRightCalib" class="cyber-input" step="0.01"></div>
                        <div class="input-group"><label>测试PWM</label><input type="number" id="calibTestPWM" class="cyber-input"></div>
                        <div class="input-group"><label>时长(s)</label><input type="number" id="calibTestDuration" class="cyber-input"></div>
                    </div>
                    
                    <div id="calibTestResult" style="display: none; margin-top: 10px; font-size: 0.9rem;">
                        <div style="display: flex; justify-content: space-between; margin-bottom: 5px;">
                            <span>L: <span id="calibLeftSpeed" style="color:var(--primary)">--</span></span>
                            <span>R: <span id="calibRightSpeed" style="color:var(--success)">--</span></span>
                        </div>
                        <div id="calibSuggestion" style="color: var(--warning); font-size: 0.8rem;">--</div>
                    </div>

                    <div class="btn-row">
                        <button class="cyber-btn" onclick="startCalibTest()">测试</button>
                        <button class="cyber-btn" onclick="saveCalibration()">保存</button>
                        <button class="cyber-btn secondary" onclick="stopCalibTest()">停止</button>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <div class="bottom-bar">
        <button class="cyber-btn" onclick="saveParams()">💾 保存参数</button>
        <button class="cyber-btn secondary" onclick="loadParams()">🔄 读取</button>
        <button class="cyber-btn danger" onclick="resetParams()">⚠️ 重置</button>
    </div>

    <div class="toast-container" id="toast-container"></div>

    <script>
        // Toast System
        function showToast(message, type = 'success') {
            const container = document.getElementById('toast-container');
            const toast = document.createElement('div');
            toast.className = `cyber-toast ${type}`;
            toast.innerHTML = `
                <span style="font-size: 1.2em">${type === 'success' ? '✅' : type === 'error' ? '❌' : 'ℹ️'}</span>
                <span>${message}</span>
            `;
            container.appendChild(toast);
            
            // Trigger animation
            requestAnimationFrame(() => toast.classList.add('show'));
            
            setTimeout(() => {
                toast.classList.remove('show');
                setTimeout(() => toast.remove(), 300);
            }, 3000);
        }

        // --- Existing Logic Preserved Below ---
        
        let statusInterval;
        
        // 加载参数
        async function loadParams() {
            try {
                const response = await fetch('/api/params');
                const data = await response.json();
                
                // PID参数
                document.getElementById('kp').value = data.pid.kp;
                document.getElementById('ki').value = data.pid.ki;
                document.getElementById('kd').value = data.pid.kd;
                if (data.pid.kpPost !== undefined) {
                    document.getElementById('kpPost').value = data.pid.kpPost;
                    document.getElementById('kiPost').value = data.pid.kiPost;
                    document.getElementById('kdPost').value = data.pid.kdPost;
                }
                
                // 高级PID参数
                if (data.advanced) {
                    document.getElementById('pidIntRange').value = data.advanced.intRange;
                    document.getElementById('motorDeadband').value = data.advanced.deadband;
                    document.getElementById('pidSmallErrorThres').value = data.advanced.smallErr;
                    document.getElementById('pidKpSmallScale').value = data.advanced.kpScale;
                    document.getElementById('pidKdSmallScale').value = data.advanced.kdScale;
                }
                
                // 物体测量参数
                if (data.object) {
                    document.getElementById('objLengthScale').value = data.object.scale;
                    document.getElementById('objLengthOffset').value = data.object.offset;
                    if (data.object.devCorr !== undefined) {
                        document.getElementById('objDeviationCorrection').value = data.object.devCorr;
                    }
                    if (data.object.threshold !== undefined) {
                        document.getElementById('detectionBaseline').value = data.object.threshold;
                    }
                }
                
                // 传感器权重
                if (data.weights) {
                    for (let i = 0; i < 8; i++) {
                        const el = document.getElementById('weight' + i);
                        if (el) el.value = data.weights[i];
                    }
                }
                
                // 速度参数
                document.getElementById('speedSlow').value = data.speed.slow;
                document.getElementById('speedNormal').value = data.speed.normal;
                document.getElementById('speedFast').value = data.speed.fast;
                document.getElementById('speedTurn').value = data.speed.turn;
                if (data.speed.normalPost !== undefined) {
                    document.getElementById('speedNormalPost').value = data.speed.normalPost;
                    document.getElementById('speedFastPost').value = data.speed.fastPost;
                    document.getElementById('speedTurnPost').value = data.speed.turnPost;
                }
                
                // 避障参数
                if (data.threshold) {
                    document.getElementById('obstacleDetectDist').value = data.threshold.obstacle || 30;
                }
                if (data.avoid) {
                    document.getElementById('avoidForwardDist').value = data.avoid.forward || 500;
                    document.getElementById('avoidParallelDist').value = data.avoid.parallel || 500;
                    document.getElementById('avoidFinalTurnDist').value = data.avoid.finalTurn || 118.0;
                    document.getElementById('avoidTurn1Dist').value = data.avoid.turn1 || 118.0;
                    document.getElementById('avoidTurn2Dist').value = data.avoid.turn2 || 118.0;
                    document.getElementById('avoidTurn3Dist').value = data.avoid.turn3 || 118.0;
                    document.getElementById('avoidSearchDist').value = data.avoid.search || 800;
                    document.getElementById('avoidSpeed').value = data.avoid.speed || 150;
                    document.getElementById('avoidTurnSpeed').value = data.avoid.turnSpeed || 120;
                    document.getElementById('avoidKp').value = data.avoid.kp || 2.0;
                }
                
                if (data.avoidSteps) {
                    document.getElementById('avS1L').value = data.avoidSteps.s1l || 1.0; document.getElementById('avS1R').value = data.avoidSteps.s1r || 1.0;
                    document.getElementById('avS2L').value = data.avoidSteps.s2l || 1.0; document.getElementById('avS2R').value = data.avoidSteps.s2r || 1.0;
                    document.getElementById('avS3L').value = data.avoidSteps.s3l || 1.0; document.getElementById('avS3R').value = data.avoidSteps.s3r || 1.0;
                    document.getElementById('avS4L').value = data.avoidSteps.s4l || 1.0; document.getElementById('avS4R').value = data.avoidSteps.s4r || 1.0;
                    document.getElementById('avS5L').value = data.avoidSteps.s5l || 1.0; document.getElementById('avS5R').value = data.avoidSteps.s5r || 1.0;
                    document.getElementById('avS6L').value = data.avoidSteps.s6l || 1.0; document.getElementById('avS6R').value = data.avoidSteps.s6r || 1.0;
                }
                
                // 车库参数
                if (data.parking) {
                    document.getElementById('pkDistSlow').value = data.parking.distSlow || 60;
                    document.getElementById('pkDistVSlow').value = data.parking.distVSlow || 30;
                    document.getElementById('pkDistStop').value = data.parking.distStop || 10;
                    document.getElementById('pkSpdSlow').value = data.parking.spdSlow || 100;
                    document.getElementById('pkSpdVSlow').value = data.parking.spdVSlow || 60;
                }
                
                // 编码器闭环参数
                if (data.encoder) {
                    document.getElementById('turn90Dist').value = data.encoder.turn90 || 118.0;
                }
                
                // 电机校准系数
                if (data.motorCalib) {
                    document.getElementById('motorLeftCalib').value = data.motorCalib.left.toFixed(2);
                    document.getElementById('motorRightCalib').value = data.motorCalib.right.toFixed(2);
                }
                
                showToast('参数加载成功', 'success');
            } catch (error) {
                showToast('加载失败: ' + error, 'error');
            }
        }
        
        // 保存参数
        async function saveParams() {
            const params = {
                pid: {
                    kp: parseFloat(document.getElementById('kp').value),
                    ki: parseFloat(document.getElementById('ki').value),
                    kd: parseFloat(document.getElementById('kd').value),
                    kpPost: parseFloat(document.getElementById('kpPost').value),
                    kiPost: parseFloat(document.getElementById('kiPost').value),
                    kdPost: parseFloat(document.getElementById('kdPost').value)
                },
                advanced: {
                    intRange: parseInt(document.getElementById('pidIntRange').value),
                    deadband: parseInt(document.getElementById('motorDeadband').value),
                    smallErr: parseInt(document.getElementById('pidSmallErrorThres').value),
                    kpScale: parseFloat(document.getElementById('pidKpSmallScale').value),
                    kdScale: parseFloat(document.getElementById('pidKdSmallScale').value)
                },
                object: {
                    scale: parseFloat(document.getElementById('objLengthScale').value),
                    offset: parseFloat(document.getElementById('objLengthOffset').value),
                    devCorr: parseFloat(document.getElementById('objDeviationCorrection').value)
                },
                speed: {
                    slow: parseInt(document.getElementById('speedSlow').value),
                    normal: parseInt(document.getElementById('speedNormal').value),
                    fast: parseInt(document.getElementById('speedFast').value),
                    turn: parseInt(document.getElementById('speedTurn').value),
                    normalPost: parseInt(document.getElementById('speedNormalPost').value),
                    fastPost: parseInt(document.getElementById('speedFastPost').value),
                    turnPost: parseInt(document.getElementById('speedTurnPost').value)
                },
                threshold: {
                    obstacle: parseInt(document.getElementById('obstacleDetectDist').value)
                },
                avoid: {
                    forward: parseInt(document.getElementById('avoidForwardDist').value),
                    parallel: parseInt(document.getElementById('avoidParallelDist').value),
                    finalTurn: parseFloat(document.getElementById('avoidFinalTurnDist').value),
                    turn1: parseFloat(document.getElementById('avoidTurn1Dist').value),
                    turn2: parseFloat(document.getElementById('avoidTurn2Dist').value),
                    turn3: parseFloat(document.getElementById('avoidTurn3Dist').value),
                    search: parseInt(document.getElementById('avoidSearchDist').value),
                    speed: parseInt(document.getElementById('avoidSpeed').value),
                    turnSpeed: parseInt(document.getElementById('avoidTurnSpeed').value),
                    kp: parseFloat(document.getElementById('avoidKp').value)
                },
                avoidSteps: {
                    s1l: parseFloat(document.getElementById('avS1L').value), s1r: parseFloat(document.getElementById('avS1R').value),
                    s2l: parseFloat(document.getElementById('avS2L').value), s2r: parseFloat(document.getElementById('avS2R').value),
                    s3l: parseFloat(document.getElementById('avS3L').value), s3r: parseFloat(document.getElementById('avS3R').value),
                    s4l: parseFloat(document.getElementById('avS4L').value), s4r: parseFloat(document.getElementById('avS4R').value),
                    s5l: parseFloat(document.getElementById('avS5L').value), s5r: parseFloat(document.getElementById('avS5R').value),
                    s6l: parseFloat(document.getElementById('avS6L').value), s6r: parseFloat(document.getElementById('avS6R').value)
                },
                parking: {
                    distSlow: parseInt(document.getElementById('pkDistSlow').value),
                    distVSlow: parseInt(document.getElementById('pkDistVSlow').value),
                    distStop: parseInt(document.getElementById('pkDistStop').value),
                    spdSlow: parseInt(document.getElementById('pkSpdSlow').value),
                    spdVSlow: parseInt(document.getElementById('pkSpdVSlow').value)
                },
                encoder: {
                    kp: 0, ki: 0, kd: 0,
                    turn90: parseFloat(document.getElementById('turn90Dist').value)
                }
            };
            
            try {
                const response = await fetch('/api/params', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(params)
                });
                
                if (response.ok) {
                    showToast('参数保存成功', 'success');
                } else {
                    showToast('保存失败', 'error');
                }
            } catch (error) {
                showToast('保存失败: ' + error, 'error');
            }
        }
        
        // 重置参数
        async function resetParams() {
            if (!confirm('确定要恢复默认参数吗?')) return;
            try {
                const response = await fetch('/api/reset', { method: 'POST' });
                if (response.ok) {
                    await loadParams();
                    showToast('参数已恢复默认!', 'success');
                }
            } catch (error) {
                showToast('重置失败: ' + error, 'error');
            }
        }
        
        // 更新状态
        async function updateStatus() {
            try {
                const response = await fetch('/api/status');
                const data = await response.json();
                
                // 更新连接状态
                const connStatus = document.getElementById('connectionStatus');
                connStatus.textContent = "SYSTEM ONLINE";
                connStatus.style.borderColor = "var(--success)";
                connStatus.style.color = "var(--success)";
                connStatus.style.background = "rgba(0,255,157,0.1)";

                // 更新当前激光距离显示
                if (data.sensor && data.sensor.laserDist !== undefined) {
                    const laserDist = data.sensor.laserDist;
                    const laserEl = document.getElementById('currentLaserDistance');
                    if (laserEl) {
                        if (laserDist > 2000) {
                            laserEl.textContent = 'OUT OF RANGE';
                            laserEl.style.color = 'var(--danger)';
                        } else {
                            laserEl.textContent = laserDist + ' mm';
                            laserEl.style.color = 'var(--primary)';
                        }
                    }
                }
                
                // 更新传感器状态
                updateSensorStatus(data);
                
            } catch (error) {
                const connStatus = document.getElementById('connectionStatus');
                connStatus.textContent = "OFFLINE";
                connStatus.style.borderColor = "var(--danger)";
                connStatus.style.color = "var(--danger)";
                connStatus.style.background = "rgba(255,42,42,0.1)";
            }
        }
        
        // 更新传感器状态
        function updateSensorStatus(data) {
            if (!data.sensor) return;
            
            // 循迹传感器
            const lineStates = data.sensor.lineStates || 0;
            const dataReady = data.sensor.dataReady || false;
            updateSensorCard('line', dataReady && lineStates !== 0, 
                `0x${lineStates.toString(16).toUpperCase().padStart(2,'0')}`);
            
            // 激光传感器
            const laserDist = data.sensor.laserDist || 0;
            const laserReady = data.sensor.laserReady || false;
            updateSensorCard('laser', laserReady && laserDist > 0, `${laserDist}mm`);
            
            // 超声波传感器
            const ultraDist = data.sensor.ultraDist || 0;
            updateSensorCard('ultra', ultraDist < 500, `${ultraDist.toFixed(1)}cm`);

            // 编码器
            if (data.motor) {
                const distL = Math.abs(data.motor.distL || 0);
                const distR = Math.abs(data.motor.distR || 0);
                const encL = data.motor.encL || 0;
                const encR = data.motor.encR || 0;
                
                updateSensorCard('encoder-l', distL > 0.1, `${distL.toFixed(0)}mm`);
                updateSensorCard('encoder-r', distR > 0.1, `${distR.toFixed(0)}mm`);
                
                const elL = document.getElementById('encoder-l-pulse');
                if(elL) elL.textContent = encL + ' P';
                
                const elR = document.getElementById('encoder-r-pulse');
                if(elR) elR.textContent = encR + ' P';
            }
            
            // 更新物块检测状态
            if (data.detection) {
                if (data.detection.completed && data.detection.valid) {
                    document.getElementById('detectionResult').style.display = 'block';
                    document.getElementById('detectionLength').textContent = data.detection.length.toFixed(1) + ' mm';
                    
                    if (data.detection.rawLength !== undefined) {
                        document.getElementById('detectionRawLength').textContent = data.detection.rawLength.toFixed(1) + ' mm';
                    }
                    
                    if (data.detection.duration !== undefined) {
                        document.getElementById('detectionDuration').textContent = data.detection.duration + ' ms';
                    }
                    
                    document.getElementById('detectionAvgDist').textContent = data.detection.avgDist.toFixed(1) + ' mm';
                    document.getElementById('detectionStatus').textContent = '✅ 完成';
                } else if (data.detection.active) {
                    document.getElementById('detectionStatus').textContent = '🔍 检测中...';
                }
            }
        }
        
        function updateSensorCard(id, isOk, valueText) {
            const card = document.getElementById('card-' + id);
            const valueEl = document.getElementById(id + '-value');
            
            if (!card || !valueEl) return;
            
            valueEl.textContent = valueText;
            
            if (isOk) {
                card.className = 'sensor-box active';
            } else {
                card.className = 'sensor-box'; // Default state
            }
        }
        
        async function testParking() {
            try {
                const response = await fetch('/api/tasks', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ action: 'test_parking' })
                });
                const data = await response.json();
                if (data.status === 'ok') showToast('入库测试已启动', 'success');
                else showToast('启动失败', 'error');
            } catch (error) { showToast('请求错误: ' + error, 'error'); }
        }

        // 手动测试所有传感器
        async function testAllSensors() {
            showToast('开始检测传感器...', 'info');
            
            // 重置所有卡片
            const cards = document.querySelectorAll('.sensor-box');
            cards.forEach(card => {
                card.className = 'sensor-box';
                const value = card.querySelector('.sensor-val');
                if (value) value.textContent = '...';
            });
            
            await new Promise(resolve => setTimeout(resolve, 500));
            
            try {
                const response = await fetch('/api/status');
                const data = await response.json();
                updateSensorStatus(data);
                showToast('传感器状态已更新', 'success');
            } catch (error) {
                showToast('检测失败: ' + error, 'error');
            }
        }
        
        // 运动控制
        let motionInterval = null;
        let currentMotion = null;
        let motionActive = false;
        
        function startMotion(action) {
            if (motionActive && currentMotion === action) return;
            stopMotion();
            currentMotion = action;
            motionActive = true;
            sendMotionCommand(action, false);
            motionInterval = setInterval(() => {
                sendMotionCommand(action, false);
            }, 150);
        }
        
        function stopMotion() {
            if (motionInterval) {
                clearInterval(motionInterval);
                motionInterval = null;
            }
            if (motionActive) {
                sendMotionCommand('stop', false);
                motionActive = false;
                currentMotion = null;
            }
        }
        
        async function sendMotionCommand(action, showMsg) {
            try {
                const response = await fetch('/api/motion', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ action: action, value: 0 })
                });
                if (!response.ok && showMsg) showToast('控制失败', 'error');
            } catch (error) { if (showMsg) console.error('Motion error:', error); }
        }
        
        // 应用传感器权重
        async function applyWeights() {
            const weights = [];
            for (let i = 0; i < 8; i++) {
                weights.push(parseInt(document.getElementById('weight' + i).value));
            }
            try {
                const response = await fetch('/api/weights', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ weights: weights })
                });
                if (response.ok) showToast('权重配置已应用', 'success');
                else showToast('权重设置失败', 'error');
            } catch (error) { showToast('发送失败: ' + error, 'error'); }
        }
        
        // 电机校准功能
        let calibTestInterval = null;
        
        async function startCalibTest() {
            const pwm = parseInt(document.getElementById('calibTestPWM').value);
            const duration = parseInt(document.getElementById('calibTestDuration').value) * 1000;
            
            document.getElementById('calibTestResult').style.display = 'block';
            document.getElementById('calibLeftSpeed').textContent = '...';
            document.getElementById('calibRightSpeed').textContent = '...';
            document.getElementById('calibSuggestion').textContent = 'Testing...';
            
            showToast('开始电机测试...', 'info');
            
            try {
                await fetch('/api/motion', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ action: 'forward', value: pwm })
                });
            } catch (error) {
                showToast('启动失败: ' + error, 'error');
                return;
            }
            
            calibTestInterval = setInterval(async () => {
                try {
                    const response = await fetch('/api/status');
                    const data = await response.json();
                    if (data.motor) {
                        const leftSpeed = Math.abs(data.motor.speedL || 0);
                        const rightSpeed = Math.abs(data.motor.speedR || 0);
                        document.getElementById('calibLeftSpeed').textContent = leftSpeed.toFixed(1);
                        document.getElementById('calibRightSpeed').textContent = rightSpeed.toFixed(1);
                        
                        if (leftSpeed > 10 && rightSpeed > 10) {
                            const ratio = leftSpeed / rightSpeed;
                            let suggestion = '';
                            if (ratio > 1.05) suggestion = `左轮快 (x${ratio.toFixed(2)})`;
                            else if (ratio < 0.95) suggestion = `右轮快 (x${(1/ratio).toFixed(2)})`;
                            else suggestion = '平衡良好';
                            document.getElementById('calibSuggestion').textContent = suggestion;
                        }
                    }
                } catch (error) {}
            }, 200);
            
            setTimeout(() => {
                stopCalibTest();
                showToast('测试完成', 'success');
            }, duration);
        }
        
        async function stopCalibTest() {
            if (calibTestInterval) {
                clearInterval(calibTestInterval);
                calibTestInterval = null;
            }
            try {
                await fetch('/api/motion', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ action: 'stop', value: 0 })
                });
            } catch (error) {}
        }
        
        async function saveCalibration() {
            const leftCalib = parseFloat(document.getElementById('motorLeftCalib').value);
            const rightCalib = parseFloat(document.getElementById('motorRightCalib').value);
            
            try {
                const response = await fetch('/api/calibration', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ leftCalib: leftCalib, rightCalib: rightCalib })
                });
                if (response.ok) showToast('校准系数已保存', 'success');
                else showToast('保存失败', 'error');
            } catch (error) { showToast('保存失败: ' + error, 'error'); }
        }
        
        // 物块检测功能
        async function startObjectDetection() {
            const range = parseInt(document.getElementById('detectionBaseline').value);
            const scale = parseFloat(document.getElementById('objLengthScale').value);
            const offset = parseFloat(document.getElementById('objLengthOffset').value);
            const devCorr = parseFloat(document.getElementById('objDeviationCorrection').value);
            
            try {
                const response = await fetch('/api/detection/start', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ 
                        baseline: 0, threshold: range, filter: 5,
                        scale: scale, offset: offset, devCorr: devCorr
                    })
                });
                if (response.ok) {
                    showToast('物块检测已启动', 'success');
                    document.getElementById('detectionResult').style.display = 'block';
                    document.getElementById('detectionStatus').textContent = '检测中...';
                } else showToast('启动失败', 'error');
            } catch (error) { showToast('启动失败: ' + error, 'error'); }
        }
        
        async function stopDetection() {
            try {
                const response = await fetch('/api/detection/stop', { method: 'POST' });
                if (response.ok) showToast('检测已停止', 'success');
                else showToast('停止失败', 'error');
            } catch (error) { showToast('停止失败: ' + error, 'error'); }
        }
        
        async function testTurn() {
            try {
                const response = await fetch('/api/tasks', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ action: 'test_turn' })
                });
                if (response.ok) showToast('开始测试转弯', 'success');
            } catch (e) { showToast('请求失败', 'error'); }
        }

        async function testStraight() {
            try {
                const response = await fetch('/api/tasks', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ action: 'test_straight' })
                });
                if (response.ok) showToast('开始测试直线', 'success');
            } catch (e) { showToast('请求失败', 'error'); }
        }

        async function testAvoid() {
            try {
                const response = await fetch('/api/tasks', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ action: 'test_avoid' })
                });
                if (response.ok) showToast('开始测试避障', 'success');
            } catch (e) { showToast('请求失败', 'error'); }
        }

        function resetDetection() {
            stopDetection();
            document.getElementById('detectionResult').style.display = 'none';
            document.getElementById('detectionLength').textContent = '--';
            document.getElementById('detectionAvgDist').textContent = '--';
            document.getElementById('detectionStatus').textContent = '--';
        }
        
        // 日志功能
        let autoScroll = true;
        let logInterval;
        
        async function updateLogs() {
            try {
                const response = await fetch('/api/logs');
                const data = await response.json();
                const logDisplay = document.getElementById('logDisplay');
                if (data.logs && data.logs.length > 0) {
                    logDisplay.innerHTML = data.logs.join('');
                    if (autoScroll) logDisplay.scrollTop = logDisplay.scrollHeight;
                }
            } catch (error) {}
        }
        
        async function clearLogs() {
            try {
                const response = await fetch('/api/logs/clear', { method: 'POST' });
                if (response.ok) {
                    document.getElementById('logDisplay').innerHTML = '';
                    showToast('日志已清空', 'success');
                }
            } catch (error) {}
        }
        
        function toggleAutoScroll() {
            autoScroll = !autoScroll;
            const btn = document.getElementById('autoScrollBtn');
            btn.textContent = '滚动: ' + (autoScroll ? 'ON' : 'OFF');
            btn.style.color = autoScroll ? 'var(--primary)' : 'var(--text-dim)';
        }
        
        window.onload = function() {
            loadParams();
            updateStatus();
            testAllSensors();
            updateLogs();
            statusInterval = setInterval(updateStatus, 500);
            logInterval = setInterval(updateLogs, 500);
            
            window.addEventListener('blur', stopMotion);
            document.addEventListener('visibilitychange', function() {
                if (document.hidden) stopMotion();
            });
        };
    </script>
</body>
</html>
)rawliteral";
}

// 添加日志
void WebServerManager::addLog(String message) {
    // 添加时间戳
    unsigned long ms = millis();
    unsigned long seconds = ms / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    
    char timestamp[32];
    snprintf(timestamp, sizeof(timestamp), "[%02lu:%02lu:%02lu.%03lu] ", 
             hours % 24, minutes % 60, seconds % 60, ms % 1000);
    
    // 格式化日志，添加HTML样式
    String formattedLog = String(timestamp);
    
    // 根据内容添加颜色
    if (message.indexOf("✓") >= 0 || message.indexOf("SUCCESS") >= 0) {
        formattedLog += "<span style='color: #4ec9b0;'>" + message + "</span>\n";
    } else if (message.indexOf("⚠") >= 0 || message.indexOf("WARN") >= 0) {
        formattedLog += "<span style='color: #dcdcaa;'>" + message + "</span>\n";
    } else if (message.indexOf("✗") >= 0 || message.indexOf("ERROR") >= 0 || message.indexOf("FAIL") >= 0) {
        formattedLog += "<span style='color: #f48771;'>" + message + "</span>\n";
    } else if (message.indexOf("➡") >= 0 || message.indexOf("->") >= 0) {
        formattedLog += "<span style='color: #569cd6;'>" + message + "</span>\n";
    } else if (message.indexOf("[Detect]") >= 0) {
        formattedLog += "<span style='color: #ce9178;'>" + message + "</span>\n";
    } else {
        formattedLog += message + "\n";
    }
    
    // 线程安全地写入日志
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // 循环缓冲区
        logs[logIndex] = formattedLog;
        logIndex = (logIndex + 1) % MAX_LOGS;
        if (logCount < MAX_LOGS) {
            logCount++;
        }
        xSemaphoreGive(mutex);
    }
    
    // 同时输出到串口
    Serial.print(timestamp);
    Serial.println(message);
}

// 获取日志JSON
String WebServerManager::getLogs() {
    JsonDocument doc;
    JsonArray logsArray = doc["logs"].to<JsonArray>();
    
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        // 按照正确顺序添加日志
        int start = (logCount < MAX_LOGS) ? 0 : logIndex;
        for (int i = 0; i < logCount; i++) {
            int idx = (start + i) % MAX_LOGS;
            logsArray.add(logs[idx]);
        }
        xSemaphoreGive(mutex);
    }
    
    String output;
    serializeJson(doc, output);
    return output;
}

// 清空日志
void WebServerManager::clearLogs() {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        logIndex = 0;
        logCount = 0;
        for (int i = 0; i < MAX_LOGS; i++) {
            logs[i] = "";
        }
        xSemaphoreGive(mutex);
    }
}
