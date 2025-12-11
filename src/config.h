#ifndef CONFIG_H
#define CONFIG_H

// ==================== GPIO引脚定义 ====================
// 超声波传感器
#define PIN_ULTRASONIC_TRIG   39
#define PIN_ULTRASONIC_ECHO   3

// 循迹模块 UART
#define PIN_LINE_TX          18
#define PIN_LINE_RX          17

// 声光报警
#define PIN_ALARM            46

// 右电机
#define PIN_MOTOR_R_I1       4
#define PIN_MOTOR_R_I2       5
#define PIN_ENCODER_R_A      10
#define PIN_ENCODER_R_B      9

// 左电机
#define PIN_MOTOR_L_I1       6
#define PIN_MOTOR_L_I2       7
#define PIN_ENCODER_L_A      11
#define PIN_ENCODER_L_B      12

// I2C1 (BMI160 + VL53L0X)
#define PIN_I2C1_SDA         16
#define PIN_I2C1_SCL         15

// I2C2 (OLED)
#define PIN_I2C2_SDA         14
#define PIN_I2C2_SCL         13

// 按键
#define PIN_BUTTON           1

// ==================== 车辆物理参数 ====================
#define CAR_LENGTH_CM        15.0      // 车长 cm
#define CAR_WIDTH_CM         19.18     // 车宽 cm
#define WHEEL_DIAMETER_MM    67.6      // 轮胎直径 mm
#define WHEEL_WIDTH_MM       26.4      // 轮胎宽度 mm
#define WHEEL_BASE_CM        15.0      // 轮距(估算) cm

// 编码器参数
#define ENCODER_PPR          11        // 每圈脉冲数
#define GEAR_RATIO           21.7      // 减速比
#define PULSES_PER_REV       (ENCODER_PPR * GEAR_RATIO * 4)  // AB相4倍频

// 车轮周长(mm)
#define WHEEL_CIRCUMFERENCE  (3.14159 * WHEEL_DIAMETER_MM)

// 脉冲到距离的转换系数 (mm/pulse)
#define MM_PER_PULSE         (WHEEL_CIRCUMFERENCE / PULSES_PER_REV)

// ==================== 传感器参数 ====================
#define LINE_SENSOR_COUNT    8
#define LINE_UART_BAUD       115200

// ==================== 控制参数 ====================
// PWM参数
#define PWM_FREQ             20000     // 20kHz PWM频率
#define PWM_RESOLUTION       8         // 8位分辨率 (0-255)
#define PWM_CHANNEL_R1       4
#define PWM_CHANNEL_R2       5
#define PWM_CHANNEL_L1       6
#define PWM_CHANNEL_L2       7

// 速度参数 (PWM值: 0-255)
#define SPEED_STOP           0
#define SPEED_SLOW           80        // 慢速(入库)
#define SPEED_NORMAL         150       // 正常巡线
#define SPEED_FAST           200       // 快速直线
#define SPEED_TURN           120       // 转弯速度

// PID参数 - 循迹 (电赛标准参数,可通过WebServer调整)
// 注意: 位置范围是 -1000~+1000, 输出差速PWM 0~255
#define KP_LINE              0.20     // 比例系数: 位置误差的响应速度
#define KI_LINE              0.005    // 积分系数: 消除稳态误差
#define KD_LINE              1.5      // 微分系数: 抑制振荡和超调

// 新增优化参数
#define PID_INTEGRAL_RANGE   200      // 积分分离阈值：误差小于此值才进行积分
#define MOTOR_DEADBAND       30       // 电机死区补偿PWM值 (根据电机特性调整)
#define MOTOR_SLEW_RATE      20       // 电机加速度限制 (每周期最大PWM变化量)

// 动态PID参数 (优化直线平顺度)
#define PID_SMALL_ERROR_THRES     150   // 直线判定阈值
#define PID_KP_SMALL_SCALE        0.6   // 直线时Kp缩放系数 (降低响应防抖动)
#define PID_KD_SMALL_SCALE        1.5   // 直线时Kd缩放系数 (增加阻尼防震荡)

// 超声波距离阈值 (cm)
#define OBSTACLE_DETECT_DIST 30        // 障碍物检测距离
#define OBSTACLE_SAFE_DIST   15        // 安全距离

// 避障参数
#define AVOID_TIME_MS        5000      // 避障最大时间 5秒
#define OBSTACLE_WIDTH_CM    30        // 障碍物宽度
#define OBSTACLE_LENGTH_CM   30        // 障碍物长度
#define AVOID_TURN_TIME_MS   1200      // 转向时间 (ms) - 90度转向约需1.2秒
#define AVOID_FORWARD_DIST_MM 500      // 绕行前进距离 (mm)

// 物体测量参数
#define OBJECT_DETECT_DIST   300       // 物体检测距离 (mm)
#define OBJECT_LENGTH_SCALE  1.0f      // 长度计算乘数
#define OBJECT_LENGTH_OFFSET 0.0f      // 长度计算加数

// 车库参数
#define PARKING_WIDTH        400       // 车库宽度 mm
#define PARKING_DEPTH        450       // 车库深度 mm

// ==================== 状态定义 ====================
enum SystemState {
    STATE_IDLE,              // 待机
    STATE_LINE_FOLLOW,       // 循迹
    STATE_OBSTACLE_AVOID,    // 避障中
    STATE_PARKING,           // 入库停车中
    STATE_FINISHED,          // 任务完成
    STATE_TESTING            // 测试模式
};

// ==================== 调试选项 ====================
#define DEBUG_SERIAL         true      // 串口调试输出
#define DEBUG_OLED           true      // OLED显示
#define DEBUG_LINE_SENSOR    false     // 循迹传感器详细输出
#define DEBUG_ENCODER        false     // 编码器输出
#define DEBUG_PID            true      // PID调试输出

// ==================== WiFi配置 ====================
#define WIFI_SSID            "04"      // 要连接的WiFi名称
#define WIFI_PASSWORD        "12345678"  // WiFi密码
#define WIFI_CONNECT_TIMEOUT 3000                   // WiFi连接超时(ms)

#define WIFI_AP_SSID         "SmartCar_AP"
#define WIFI_AP_PASSWORD     "12345678"
#define WEB_SERVER_PORT      80

#endif
