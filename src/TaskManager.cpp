#include "TaskManager.h"
#include <ArduinoJson.h>

TaskManager::TaskManager() {
    currentTaskIndex = 0;
    nextTaskId = 1;
    executing = false;
    taskExecutor = nullptr;
    taskChecker = nullptr;
}

int TaskManager::addTask(TaskType type, TaskParams params, String description) {
    Task task;
    task.id = nextTaskId++;
    task.type = type;
    task.status = TASK_PENDING;
    task.params = params;
    task.startTime = 0;
    task.description = description;
    
    tasks.push_back(task);
    
    Serial.printf("Task added: ID=%d, Type=%d, Desc=%s\n", 
        task.id, task.type, description.c_str());
    
    return task.id;
}

bool TaskManager::removeTask(int taskId) {
    for (auto it = tasks.begin(); it != tasks.end(); ++it) {
        if (it->id == taskId) {
            tasks.erase(it);
            Serial.printf("Task removed: ID=%d\n", taskId);
            return true;
        }
    }
    return false;
}

void TaskManager::clearAllTasks() {
    tasks.clear();
    currentTaskIndex = 0;
    executing = false;
    Serial.println("All tasks cleared");
}

void TaskManager::startExecution() {
    if (tasks.empty()) {
        Serial.println("⚠ No tasks to execute");
        return;
    }
    
    currentTaskIndex = 0;
    executing = true;
    
    Serial.println("=== Task Execution Started ===");
    Serial.printf("Total tasks: %d\n", tasks.size());
}

void TaskManager::pauseExecution() {
    executing = false;
    Serial.println("Task execution paused");
}

void TaskManager::stopExecution() {
    executing = false;
    currentTaskIndex = 0;
    
    // 重置所有任务状态
    for (auto& task : tasks) {
        if (task.status == TASK_RUNNING) {
            task.status = TASK_PENDING;
        }
    }
    
    Serial.println("Task execution stopped");
}

void TaskManager::update() {
    if (!executing || tasks.empty()) {
        return;
    }
    
    if (currentTaskIndex >= tasks.size()) {
        // 所有任务完成
        executing = false;
        Serial.println("=== All Tasks Completed ===");
        return;
    }
    
    Task* currentTask = &tasks[currentTaskIndex];
    
    if (currentTask->status == TASK_PENDING) {
        // 启动新任务
        currentTask->status = TASK_RUNNING;
        currentTask->startTime = millis();
        
        Serial.printf("\n>>> Executing Task %d/%d: %s\n", 
            currentTaskIndex + 1, tasks.size(), currentTask->description.c_str());
        
        // 调用执行器
        if (taskExecutor) {
            bool success = taskExecutor(currentTask);
            if (!success) {
                currentTask->status = TASK_FAILED;
                Serial.println("✗ Task execution failed!");
            }
        }
    }
    else if (currentTask->status == TASK_RUNNING) {
        // 检查任务是否完成
        if (taskChecker) {
            bool completed = taskChecker(currentTask);
            if (completed) {
                currentTask->status = TASK_COMPLETED;
                Serial.printf("✓ Task completed in %lums\n", 
                    millis() - currentTask->startTime);
                
                // 移动到下一个任务
                currentTaskIndex++;
            }
        }
    }
    else if (currentTask->status == TASK_FAILED) {
        // 任务失败，停止执行
        executing = false;
        Serial.println("✗ Task sequence stopped due to failure");
    }
}

Task* TaskManager::getCurrentTask() {
    if (currentTaskIndex < tasks.size()) {
        return &tasks[currentTaskIndex];
    }
    return nullptr;
}

String TaskManager::getTasksJson() {
    JsonDocument doc;
    JsonArray taskArray = doc["tasks"].to<JsonArray>();
    
    for (const auto& task : tasks) {
        JsonObject taskObj = taskArray.add<JsonObject>();
        taskObj["id"] = task.id;
        taskObj["type"] = task.type;
        taskObj["status"] = task.status;
        taskObj["description"] = task.description;
        
        JsonObject params = taskObj["params"].to<JsonObject>();
        params["distance"] = task.params.distance;
        params["angle"] = task.params.angle;
        params["speed"] = task.params.speed;
        params["duration"] = task.params.duration;
        params["laserBaseline"] = task.params.laserBaseline;
        params["laserThreshold"] = task.params.laserThreshold;
    }
    
    doc["currentIndex"] = currentTaskIndex;
    doc["executing"] = executing;
    doc["total"] = tasks.size();
    
    String output;
    serializeJson(doc, output);
    return output;
}

bool TaskManager::loadTasksFromJson(String json) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json);
    
    if (error) {
        Serial.print("JSON parse error: ");
        Serial.println(error.c_str());
        return false;
    }
    
    clearAllTasks();
    
    JsonArray taskArray = doc["tasks"];
    for (JsonObject taskObj : taskArray) {
        TaskParams params;
        params.distance = taskObj["params"]["distance"] | 0.0f;
        params.angle = taskObj["params"]["angle"] | 0.0f;
        params.speed = taskObj["params"]["speed"] | 0;
        params.duration = taskObj["params"]["duration"] | 0;
        params.laserBaseline = taskObj["params"]["laserBaseline"] | 800;
        params.laserThreshold = taskObj["params"]["laserThreshold"] | 100;
        params.customData = taskObj["params"]["customData"] | "";
        
        TaskType type = (TaskType)(taskObj["type"] | 0);
        String desc = taskObj["description"] | "";
        
        addTask(type, params, desc);
    }
    
    Serial.printf("Loaded %d tasks from JSON\n", tasks.size());
    return true;
}

void TaskManager::setTaskExecutor(bool (*executor)(Task* task)) {
    taskExecutor = executor;
}

void TaskManager::setTaskChecker(bool (*checker)(Task* task)) {
    taskChecker = checker;
}
