#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <Arduino.h>
#include <vector>

// 任务类型枚举
enum TaskType {
    TASK_LINE_FOLLOW,      // 循迹
    TASK_MEASURE_OBJECT,   // 测量物块
    TASK_FORWARD,          // 前进指定距离
    TASK_BACKWARD,         // 后退指定距离
    TASK_TURN_LEFT,        // 左转
    TASK_TURN_RIGHT,       // 右转
    TASK_STOP,             // 停止
    TASK_DELAY,            // 延时
    TASK_BEEP,             // 蜂鸣
    TASK_CUSTOM            // 自定义（未来扩展）
};

// 任务状态
enum TaskStatus {
    TASK_PENDING,          // 等待执行
    TASK_RUNNING,          // 正在执行
    TASK_COMPLETED,        // 已完成
    TASK_FAILED            // 失败
};

// 任务参数结构
struct TaskParams {
    float distance;        // 距离参数 (mm)
    float angle;           // 角度参数 (度)
    int speed;             // 速度参数 (PWM)
    unsigned long duration; // 持续时间 (ms)
    uint16_t laserBaseline; // 激光基线距离
    uint16_t laserThreshold; // 激光阈值
    String customData;     // 自定义数据（JSON）
};

// 任务定义
struct Task {
    int id;                // 任务ID
    TaskType type;         // 任务类型
    TaskStatus status;     // 任务状态
    TaskParams params;     // 任务参数
    unsigned long startTime; // 开始时间
    String description;    // 任务描述
};

// 任务管理器
class TaskManager {
public:
    TaskManager();
    
    // 任务队列管理
    int addTask(TaskType type, TaskParams params, String description = "");
    bool removeTask(int taskId);
    void clearAllTasks();
    
    // 任务执行控制
    void startExecution();
    void pauseExecution();
    void stopExecution();
    void update();  // 在主循环中调用
    
    // 任务状态查询
    bool isExecuting() { return executing; }
    bool isCompleted() { return currentTaskIndex >= tasks.size() && !executing; }
    int getCurrentTaskIndex() { return currentTaskIndex; }
    int getTotalTasks() { return tasks.size(); }
    Task* getCurrentTask();
    
    // 获取任务列表
    String getTasksJson();
    
    // 从JSON加载任务
    bool loadTasksFromJson(String json);
    
    // 设置回调函数（用于实际执行任务）
    void setTaskExecutor(bool (*executor)(Task* task));
    void setTaskChecker(bool (*checker)(Task* task));

private:
    std::vector<Task> tasks;
    int currentTaskIndex;
    int nextTaskId;
    bool executing;
    
    bool (*taskExecutor)(Task* task);    // 任务执行回调
    bool (*taskChecker)(Task* task);     // 任务完成检查回调
};

#endif
