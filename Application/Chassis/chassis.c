/**
  ******************************************************************************
  * @file    chassis.c
  * @brief   麦轮底盘运动控制实现
  ******************************************************************************
  * @description
  * 
  * 功能概述：
  * - 四轮麦轮底盘运动控制
  * - 支持X/Y/YAW三轴独立运动
  * - 使用ADRC控制器实现精确位置控制
  * - 编码器反馈实现闭环控制
  * - CAN总线电机通信
  * 
  * 控制架构：
  * 目标位置 -> ADRC控制器 -> 麦轮运动学解算 -> CAN电机控制 -> 编码器反馈
  *
  ******************************************************************************
  */
/* ==================== 头文件包含 ==================== */
#include "stm32f4xx_hal.h"
#include "chassis.h"
#include "PID.h"
#include "ADRC.h"
#include "Hwt101.h"
#include "Emm_V5_CAN.h"
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_log.h"
#include "BSP_dwt.h"
#include "cmsis_os.h"

/* ==================== 配置开关 ==================== */
#define CHASSIS_DEBUG 0  // 底盘调试开关：1=开启调试 0=关闭调试

/* ==================== 调试宏定义 ==================== */
#if CHASSIS_DEBUG
    #define DEBUG_VAR(var) var
    #define DEBUG_CODE(code) do { code } while(0)
    #define DEBUG_LOG(format, ...) LOGINFO(format, ##__VA_ARGS__)
    #define DEBUG_CALL(func) func
#else
    #define DEBUG_VAR(var)
    #define DEBUG_CODE(code)
    #define DEBUG_LOG(format, ...)
    #define DEBUG_CALL(func) (void)0
#endif

/* ==================== 外部引用 ==================== */
extern float Angle;  // 陀螺仪角度信息（单位：度）

/* ==================== 数学常量 ==================== */
#define PI                    3.14159265358979323846f
#define DEG_TO_RAD(x)         ((x) * PI / 180.0f)
#define RAD_TO_DEG(x)         ((x) * 180.0f / PI)

/* ==================== 物理参数配置 ==================== */
#define CHASSIS_WIDTH         0.435f    // 底盘宽度 (m)
#define CHASSIS_LENGTH        0.51f     // 底盘长度 (m)
#define WHEEL_DIAMETER        0.078f    // 麦轮直径 (m)
#define CHASSIS_WHEEL_RADIUS  (WHEEL_DIAMETER / 2.0f)
#define MECANUM_FACTOR        (CHASSIS_WIDTH + CHASSIS_LENGTH)

/* ==================== 编码器参数 ==================== */
#define ENCODER_COUNTS_PER_REV 65536.0f  // 编码器分辨率 (计数/圈)
#define ENCODER_COUNTS_PER_M  (ENCODER_COUNTS_PER_REV / (WHEEL_DIAMETER * PI))  // 编码器精度 (计数/米)

/* ==================== 控制参数 ==================== */
#define POSITION_TOLERANCE_XY  0.02f   // XY位置控制精度 (m)
#define POSITION_TOLERANCE_YAW 1.5f    // 角度控制精度 (度)
#define CHASSIS_TASK_PERIOD    20      // 底盘控制周期 (ms)
#define ENCODER_TASK_PERIOD    10      // 编码器读取周期 (ms)

/* ==================== 电机CAN ID配置 ==================== */
#define MOTOR_LF_ID 5  // 左前轮电机ID
#define MOTOR_RF_ID 6  // 右前轮电机ID  
#define MOTOR_LB_ID 7  // 左后轮电机ID
#define MOTOR_RB_ID 8  // 右后轮电机ID

/* ==================== 全局变量声明 ==================== */
static int32_t current_encoder[4] = {0};  // 当前编码器累积值

/* ==================== 数据结构定义 ==================== */

// 底盘状态结构体
typedef struct {
    float x;      // X坐标 (m)
    float y;      // Y坐标 (m)  
    float yaw;    // 偏航角 (度)，范围[-180, 180]
} ChassisState_t;

// PID控制器组
typedef struct {
    PIDInstance x;     // X方向PID控制器
    PIDInstance y;     // Y方向PID控制器
    PIDInstance yaw;   // 偏航角PID控制器
} ChassisPID_t;

// ADRC控制器组
typedef struct {
    ADRC_Controller x;     // X方向ADRC控制器
    ADRC_Controller y;     // Y方向ADRC控制器
    ADRC_Controller yaw;   // 偏航角ADRC控制器
} ChassisADRC_t;

/* ==================== 静态变量定义 ==================== */
static ChassisState_t  g_current_pos = {0};           // 当前位置状态
static ChassisState_t  g_target_pos = {0};            // 目标位置状态
static ChassisPID_t    g_pid = {0};                   // PID控制器（备用）
static ChassisADRC_t   g_adrc = {0};                  // ADRC控制器（主用）
static volatile int32_t g_encoder_values[4] = {0};    // 编码器实时值（由编码器任务更新）
static int32_t         prev_encoder[4] = {0};         // 上次编码器值（用于计算增量）
static TaskHandle_t    g_chassis_task_handle = NULL;  // 底盘任务句柄
static bool            g_chassis_task_running = false; // 底盘任务运行标志

/* ==================== 麦轮运动学矩阵 ==================== */
/**
 * 麦轮运动学矩阵：将底盘速度(vx, vy, w)转换为各轮子速度
 * 
 * 轮子布局：
 *   LF ---- RF
 *   |       |
 *   |       |  
 *   LB ---- RB
 * 
 * 矩阵说明：
 * wheel_speed[i] = matrix[i][0]*vx + matrix[i][1]*vy + matrix[i][2]*w
 */
static const float g_mecanum_matrix[4][3] = {
    { 1.0f, -1.0f, -MECANUM_FACTOR},  // 左前轮(LF): vx - vy - w
    { 1.0f,  1.0f,  MECANUM_FACTOR},  // 右前轮(RF): vx + vy + w  
    { 1.0f,  1.0f, -MECANUM_FACTOR},  // 左后轮(LB): vx + vy - w
    { 1.0f, -1.0f,  MECANUM_FACTOR}   // 右后轮(RB): vx - vy + w
};

/* ==================== 函数前向声明 ==================== */
static void Chassis_Task(void *argument);
static void Encoder_Read_Task(void *argument);
static void Chassis_UpdateOdometry(void);
static void Chassis_ControlLoop_Main(void);
static void Chassis_SendMotorCommands(float* wheel_speed);

#if CHASSIS_DEBUG
static void Chassis_DebugOutput(float vy, float* wheel_speed, float error_y, 
                               int32_t* current_encoder, int32_t* actual_encoder_delta);
static void Chassis_MotorDebugOutput(float vy, uint32_t send_start_time, 
                                   bool* send_success, uint32_t motor_debug_count);
#endif

/**
  * @brief  底盘初始化函数
  */
void Chassis_Init(void)
{
    // 定义电机ID数组
    uint8_t motor_ids[4] = {MOTOR_LF_ID, MOTOR_RF_ID, MOTOR_LB_ID, MOTOR_RB_ID};
    
    // 初始化EmmV5_CAN电机驱动
    if (!Emm_V5_CAN_Init(motor_ids, 4)) {
        return;  // 如果初始化失败，直接返回
    }
    // // 配置X、Y方向PID控制器（保留但不使用，用于兼容）
    // PID_Init_Config_s pid_config_xy = {
    //     .Kp = 0.82f,               // 比例系数
    //     .Ki = 0.05f,              // 积分系数
    //     .Kd = 0.0f,               // 微分系数
    //     .MaxOut = 1.0f,           // 最大速度0.5m/s
    //     .DeadBand = 0.00f,        // 1cm死区
    //     .Improve = PID_Integral_Limit |PID_OutputFilter,
    //     .IntegralLimit = 0.35f,    // 积分限幅
    //     .Output_LPF_RC = 0.0f,     // 低通滤波常数
    //     .MaxAccel = 0.0f,
    //     .MaxJerk = 0.0f,
    // };
    // PIDInit(&g_pid.x, &pid_config_xy);
    // PIDInit(&g_pid.y, &pid_config_xy);
    
    // // 配置偏航角PID控制器（保留但不使用，用于兼容）
    // PID_Init_Config_s pid_config_yaw = {
    //     .Kp = 0.0f,               // 比例系数
    //     .Ki = 0.0f,               // 积分系数
    //     .Kd = 0.00f,              // 微分系数
    //     .MaxOut = 10.0f,          // 最大角速度10度/s
    //     .DeadBand = 0.5f,         // 0.5度死区
    //     .Improve = PID_Integral_Limit | PID_OutputFilter | PID_Trapezoid_Intergral|PID_ChangingIntegrationRate,
    //     .IntegralLimit = 0.0f,    // 积分限幅（度）
    //     .Output_LPF_RC = 0.0f     // 低通滤波常数
    // };
    // PIDInit(&g_pid.yaw, &pid_config_yaw)
    
      // 初始化ADRC控制器
    // 创建XY方向ADRC配置结构体
    ADRC_Init_Config_t adrc_config_xy = {
        .r = 0.20f,               // 跟踪速度因子（降低以减缓跟踪速度）
        .h = CHASSIS_TASK_PERIOD/ 1000.0f, // 积分步长
        .b0 = 1.5f,              // 系统增益（降低以减少过冲）
        .max_output = 0.4f,       // 最大输出速度0.5m/s
        .w0 = 0.30f,
        .beta01 = 40,          // ESO 
        .beta02 = 40,
        .beta03 =0.01,
        .beta1 = 0.15f,            // NLSEF参数
        .beta2 = 1.5f,
        .alpha1 = 0.31f,
        .alpha2 = 0.75f,
        .delta = 0.1f
    };
      // 创建偏航角ADRC配置结构体
    ADRC_Init_Config_t adrc_config_yaw = {
        .r = 3.8f,               // 跟踪速度因子（降低以减缓跟踪速度）
        .h = CHASSIS_TASK_PERIOD/ 1000.0f, // 积分步长
        .b0 = 0.05f,              // 系统增益（降低以减少过冲）
        .max_output = 15.0f,       // 最大输出速度0.5m/s
        .w0 = 0.00f,
        .beta01 = 75.0,          // ESO 
        .beta02 = 65.0,
        .beta03 =0.9,
        .beta1 = 1.3f,         // NLSEF参数
        .beta2 = 3.0f,
        .alpha1 = 1.8f,
        .alpha2 = 0.37f,
        .delta = 0.1f
    };
    
    // 初始化控制器
    ADRC_Init(&g_adrc.x, &adrc_config_xy);
    ADRC_Init(&g_adrc.y, &adrc_config_xy);
    ADRC_Init(&g_adrc.yaw, &adrc_config_yaw);
    
    // // 电机初始化：使能、设置模式、清零编码器
    // for (uint8_t i = 0; i < 4; i++) {
    //     uint8_t motor_id = motor_ids[i];
        
    //     // 使能电机
    //     Emm_V5_CAN_En_Control(motor_id, true, 0);
        
    //     // 设置闭环模式
    //     Emm_V5_CAN_Modify_Ctrl_Mode(motor_id, true, 2); 
        
    //     // 清零位置
    //     Emm_V5_CAN_Reset_CurPos_To_Zero(motor_id);
        
    //     // 初始化清除故障
    //     Emm_V5_CAN_Reset_Clog_Pro(motor_id);
    // }    // 重置底盘位置
    Chassis_ResetPosition();
    
    // 直接创建底盘控制任务
    g_chassis_task_running = true;
    xTaskCreate(Chassis_Task, "Chassis", 512, NULL, 5, &g_chassis_task_handle);
    xTaskCreate(Encoder_Read_Task, "Encoder_Read", 256, NULL, 6, NULL);
}

/**
  * @brief  设置底盘目标位置
  * @param  x   目标X坐标，单位m
  * @param  y   目标Y坐标，单位m
  * @param  yaw 目标偏航角，单位度，范围(-180, 180]
  */
void  Chassis_SetTargetPosition(float x, float y, float yaw)
{
    g_target_pos.x = x;
    g_target_pos.y = y;
    g_target_pos.yaw = yaw; // 标准化目标角度
}

/**
  * @brief  获取底盘当前位置
  * @param  x   指向存储X坐标的变量指针，单位m
  * @param  y   指向存储Y坐标的变量指针，单位m
  * @param  yaw 指向存储偏航角的变量指针，单位度，范围(-180, 180]
  */
void Chassis_GetCurrentPosition(float *x, float *y, float *yaw)
{
    if (x != NULL) *x = g_current_pos.x;
    if (y != NULL) *y = g_current_pos.y;
    if (yaw != NULL) *yaw = g_current_pos.yaw;
}

/**
  * @brief  底盘控制任务函数
  */
static void Chassis_Task(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (g_chassis_task_running) {
        // 执行控制循环
        Chassis_Control_Loop();
        
        // 周期性延时
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CHASSIS_TASK_PERIOD));
    }
    
    vTaskDelete(NULL);
}
static void Encoder_Read_Task(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int32_t temp_encoder[4] = {0};
    static uint8_t error_count = 0;
    
    for(;;)
     {
        bool read_success = Emm_V5_CAN_Get_All_Encoders(temp_encoder);
        
        if (read_success) 
        {
            for (int i = 0; i < 4; i++)
            {
                g_encoder_values[i] = temp_encoder[i];
            }
            error_count = 0;  // 重置错误计数
        }
        else
        {
            error_count++;
            // 如果连续多次读取失败，增加延时
            if (error_count > 5) {
                vTaskDelay(pdMS_TO_TICKS(5));  // 额外延时5ms
                error_count = 0;
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ENCODER_TASK_PERIOD));
    }
  }
/* ==================== 主控制循环 ==================== */

/**
  * @brief  底盘控制循环主入口
  * @retval 是否到达目标位置
  */
bool Chassis_Control_Loop(void)
{
    // 1. 更新底盘位置状态
    Chassis_UpdateOdometry();
    
    // 2. 执行控制计算和电机驱动
    Chassis_ControlLoop_Main();
    
    return true; // 这里可以添加到达判断逻辑
}

/**
  * @brief  更新底盘里程计信息
  */
static void Chassis_UpdateOdometry(void)
{
    // 更新陀螺仪角度
    g_current_pos.yaw = Angle;
    
    // 读取当前编码器值
    for (int i = 0; i < 4; i++) {
        current_encoder[i] = g_encoder_values[i];
    }
    
    // 计算编码器增量和轮子位移
    float wheel_delta[4] = {0};
    DEBUG_VAR(static int32_t actual_encoder_delta[4] = {0});
    
    for(int i = 0; i < 4; i++) {
        int32_t steps = current_encoder[i] - prev_encoder[i];
        DEBUG_CODE(actual_encoder_delta[i] = steps);
        wheel_delta[i] = (float)steps / ENCODER_COUNTS_PER_M;
        prev_encoder[i] = current_encoder[i];
    }

    // 计算底盘坐标增量（麦轮逆运动学）
    float delta_x = (-wheel_delta[0] + wheel_delta[1] - wheel_delta[2] + wheel_delta[3]) / 4.0f;
    float delta_y = (wheel_delta[0] + wheel_delta[1] - wheel_delta[2] - wheel_delta[3]) / 4.0f;
     
    // 更新底盘位置（直接累加，不进行坐标系转换）
    g_current_pos.x += delta_x;
    g_current_pos.y += delta_y;
}

/**
  * @brief  底盘控制循环主逻辑
  */
static void Chassis_ControlLoop_Main(void)
{
    // 计算位置误差（变量保留供后续使用）
    // float error_x = g_target_pos.x - g_current_pos.x;
    // float error_y = g_target_pos.y - g_current_pos.y;
    // float error_yaw = g_target_pos.yaw - g_current_pos.yaw;
    
    // ADRC控制器计算控制量
    float vx = ADRC_Compute(&g_adrc.x, g_current_pos.x, g_target_pos.x);
    float vy = ADRC_Compute(&g_adrc.y, g_current_pos.y, g_target_pos.y);
    float vyaw_deg = ADRC_Compute(&g_adrc.yaw, g_target_pos.yaw, g_current_pos.yaw);
    
    // 转换角速度单位
    float vyaw_rad = DEG_TO_RAD(vyaw_deg);
    
    // 麦轮运动学正解：计算轮子速度
    float wheel_speed[4] = {0};
    for (int i = 0; i < 4; i++) {
        wheel_speed[i] = g_mecanum_matrix[i][0] * vx + 
                         g_mecanum_matrix[i][1] * vy + 
                         g_mecanum_matrix[i][2] * vyaw_rad;
    }
    
    // 调试信息输出
    DEBUG_CODE({
        static int32_t actual_encoder_delta[4] = {0};
        for (int i = 0; i < 4; i++) {
            actual_encoder_delta[i] = current_encoder[i] - prev_encoder[i];
        }
        float error_y = g_target_pos.y - g_current_pos.y;
        Chassis_DebugOutput(vy, wheel_speed, error_y, current_encoder, actual_encoder_delta);
    });
    
    // 发送电机控制命令
    Chassis_SendMotorCommands(wheel_speed);
}

/**
  * @brief  发送电机控制命令
  * @param  wheel_speed 四个轮子的速度数组 (m/s)
  */
static void Chassis_SendMotorCommands(float* wheel_speed)
{
    uint8_t motor_ids[4] = {MOTOR_LF_ID, MOTOR_RF_ID, MOTOR_LB_ID, MOTOR_RB_ID};
    uint16_t acc_values[4] = {200, 200, 200, 200}; // 电机加速度配置
    
    // 调试变量定义
    DEBUG_VAR(static uint32_t motor_debug_count = 0);
    DEBUG_VAR(static bool send_success[4]);
    DEBUG_VAR(uint32_t send_start_time);
    
#if CHASSIS_DEBUG
    // 初始化调试变量
    static bool first_call = true;
    if (first_call) {
        for (int i = 0; i < 4; i++) {
            send_success[i] = false;
        }
        first_call = false;
    }
#endif
    
    DEBUG_CODE(motor_debug_count++);
    DEBUG_CODE(send_start_time = DWT_GetTimeline_ms());
    
    // 发送电机控制命令
    for (int i = 0; i < 4; i++) {
        // 转换为电机RPM和方向
        float rpm = wheel_speed[i] * 60.0f / (2.0f * PI * CHASSIS_WHEEL_RADIUS);
        uint8_t dir = (rpm >= 0) ? 0 : 1;  // 0=正转, 1=反转
        float speed = fabsf(rpm);
        uint16_t acc = acc_values[i];

        // 发送CAN控制命令
#if CHASSIS_DEBUG
        send_success[i] = Emm_V5_CAN_Vel_Control(motor_ids[i], dir, speed, acc, 1);
#else
        Emm_V5_CAN_Vel_Control(motor_ids[i], dir, speed, acc, 1);
#endif
        
        // CAN发送间隔延时，避免总线冲突
        if (i < 3) {
            DWT_Delay_ms(0.5f);
        }
    }
    
    // 电机调试信息输出
    DEBUG_CODE({
        float vy_debug = wheel_speed[1]; // 使用第二个轮子的速度作为代表
        Chassis_MotorDebugOutput(vy_debug, send_start_time, send_success, motor_debug_count);
    });
    
    // 发送同步运动命令
    DEBUG_CALL(Emm_V5_CAN_Synchronous_motion(0));
#if !CHASSIS_DEBUG
    Emm_V5_CAN_Synchronous_motion(0);
#endif
}

/* ==================== 底盘控制函数 ==================== */

/**
  * @brief  紧急停止底盘
  */
void Chassis_EmergencyStop(void)
{
    uint8_t motor_ids[4] = {MOTOR_LF_ID, MOTOR_RF_ID, MOTOR_LB_ID, MOTOR_RB_ID};
    
    for (int i = 0; i < 4; i++) {
        Emm_V5_CAN_Stop_Now(motor_ids[i], 0);
    }
}

/**
  * @brief  重置底盘位置为原点
  */
void Chassis_ResetPosition(void)
{
    g_current_pos.x = 0.0f;
    g_current_pos.y = 0.0f;
    g_current_pos.yaw = 0.0f;
}

/* ==================== 控制器参数设置 ==================== */

/**
  * @brief  设置底盘PID控制参数（保留接口，当前使用ADRC）
  */
void Chassis_SetPIDParams(float kp_xy, float ki_xy, float kd_xy, 
                         float kp_yaw, float ki_yaw, float kd_yaw)
{
    // 更新XY方向PID参数
    g_pid.x.Kp = kp_xy;
    g_pid.x.Ki = ki_xy;
    g_pid.x.Kd = kd_xy;
    
    g_pid.y.Kp = kp_xy;
    g_pid.y.Ki = ki_xy;
    g_pid.y.Kd = kd_xy;
    
    // 更新偏航角PID参数
    g_pid.yaw.Kp = kp_yaw;
    g_pid.yaw.Ki = ki_yaw;
    g_pid.yaw.Kd = kd_yaw;
}

/**
  * @brief  设置底盘ADRC控制器基本参数
  * @param  r_xy XY方向跟踪速度因子
  * @param  b0_xy XY方向系统增益
  * @param  r_yaw 偏航角跟踪速度因子
  * @param  b0_yaw 偏航角系统增益
  * @retval 无
  */
void Chassis_SetADRCParams(float r_xy, float b0_xy, float r_yaw, float b0_yaw)
{
    // 更新XY方向ADRC参数
    ADRC_SetTD(&g_adrc.x, r_xy);
    g_adrc.x.b0 = b0_xy;
    
    ADRC_SetTD(&g_adrc.y, r_xy);
    g_adrc.y.b0 = b0_xy;
    
    // 更新偏航角ADRC参数
    ADRC_SetTD(&g_adrc.yaw, r_yaw);
    g_adrc.yaw.b0 = b0_yaw;
}

/**
  * @brief  设置底盘ADRC观测器参数
  * @param  beta01_xy XY方向ESO反馈增益1
  * @param  beta02_xy XY方向ESO反馈增益2
  * @param  beta03_xy XY方向ESO反馈增益3
  * @param  beta01_yaw 偏航角ESO反馈增益1
  * @param  beta02_yaw 偏航角ESO反馈增益2
  * @param  beta03_yaw 偏航角ESO反馈增益3
  * @retval 无
  */
void Chassis_SetADRCESOParams(float beta01_xy, float beta02_xy, float beta03_xy,
                             float beta01_yaw, float beta02_yaw, float beta03_yaw)
{
    // 更新XY方向ESO参数
    ADRC_SetESO(&g_adrc.x, beta01_xy, beta02_xy, beta03_xy);
    ADRC_SetESO(&g_adrc.y, beta01_xy, beta02_xy, beta03_xy);
    
    // 更新偏航角ESO参数
    ADRC_SetESO(&g_adrc.yaw, beta01_yaw, beta02_yaw, beta03_yaw);
}

/**
  * @brief  重置底盘控制器状态
  * @note   清空控制器内部状态，包括积分项、观测器状态等
  * @retval 无
  */
void Chassis_ResetController(void)
{
    // 重置ADRC控制器状态
    ADRC_Reset(&g_adrc.x);
    ADRC_Reset(&g_adrc.y);
    ADRC_Reset(&g_adrc.yaw);
}

/* ==================== 调试函数 ==================== */

#if CHASSIS_DEBUG
/**
 * @brief 底盘调试信息输出
 * @param vy Y方向速度命令
 * @param wheel_speed 轮子速度数组
 * @param error_y Y方向位置误差
 * @param current_encoder 当前编码器值
 * @param actual_encoder_delta 实际编码器增量
 */
static void Chassis_DebugOutput(float vy, float* wheel_speed, float error_y, 
                               int32_t* current_encoder, int32_t* actual_encoder_delta)
{
    static uint32_t debug_count = 0;
    debug_count++;
    
    if (debug_count % 50 == 0) {
        char vy_str[16], lf_str[16], rf_str[16], lb_str[16], rb_str[16];
        char ty_str[16], cy_str[16], ey_str[16];
        
        Float2Str(vy_str, vy);
        Float2Str(lf_str, wheel_speed[0]);
        Float2Str(rf_str, wheel_speed[1]);
        Float2Str(lb_str, wheel_speed[2]);
        Float2Str(rb_str, wheel_speed[3]);
        Float2Str(ty_str, g_target_pos.y);
        Float2Str(cy_str, g_current_pos.y);
        Float2Str(ey_str, error_y);
        
        LOGINFO("[WHEELS] vy=%s, LF=%s, RF=%s, LB=%s, RB=%s", 
               vy_str, lf_str, rf_str, lb_str, rb_str);
        LOGINFO("[POS] target_y=%s, current_y=%s, error_y=%s", 
               ty_str, cy_str, ey_str);
        
        // 只在有运动时显示对称性分析
        if (fabsf(vy) > 0.01f) {
            // 分析期望轮子速度对称性
            float lf_rb_diff = fabsf(wheel_speed[0] - wheel_speed[3]);
            float rf_lb_diff = fabsf(wheel_speed[1] - wheel_speed[2]);
            float max_speed = fmaxf(fmaxf(fabsf(wheel_speed[0]), fabsf(wheel_speed[1])), 
                                   fmaxf(fabsf(wheel_speed[2]), fabsf(wheel_speed[3])));
            
            // 分析实际编码器反馈对称性
            int32_t lf_rb_encoder_diff = abs(current_encoder[0] + current_encoder[3]);
            int32_t rf_lb_encoder_diff = abs(current_encoder[1] + current_encoder[2]);
            
            // 浮点数转字符串输出
            char diff1_str[16], diff2_str[16], max_str[16];
            Float2Str(diff1_str, lf_rb_diff);
            Float2Str(diff2_str, rf_lb_diff);
            Float2Str(max_str, max_speed);
            
            LOGINFO("[SYMMETRY] 期望速度 LF-RB差值: %s, RF-LB差值: %s, 最大速度: %s", 
                   diff1_str, diff2_str, max_str);
            LOGINFO("[ENCODER_ASYM] 实际编码器 LF+RB差值: %d, RF+LB差值: %d", 
                   lf_rb_encoder_diff, rf_lb_encoder_diff);
        }
        
        // 检查编码器反馈的对称性
        LOGINFO("[ENCODER] LF=%d, RF=%d, LB=%d, RB=%d", 
               current_encoder[0], current_encoder[1], current_encoder[2], current_encoder[3]);
        
        // 计算期望编码器值
        float expected_encoder_rate[4];
        for (int i = 0; i < 4; i++) {
            expected_encoder_rate[i] = wheel_speed[i] * ENCODER_COUNTS_PER_M;
        }
        
        float dt = CHASSIS_TASK_PERIOD / 1000.0f;
        int32_t expected_encoder_delta[4];
        for (int i = 0; i < 4; i++) {
            expected_encoder_delta[i] = (int32_t)(expected_encoder_rate[i] * dt);
        }
        
        // 输出期望与实际编码器值对比
        LOGINFO("[EXPECT_ENC] LF=%d, RF=%d, LB=%d, RB=%d", 
               expected_encoder_delta[0], expected_encoder_delta[1], 
               expected_encoder_delta[2], expected_encoder_delta[3]);
        
        LOGINFO("[ACTUAL_ENC] LF=%d, RF=%d, LB=%d, RB=%d", 
               actual_encoder_delta[0], actual_encoder_delta[1], 
               actual_encoder_delta[2], actual_encoder_delta[3]);
        
        // 计算并输出编码器误差
        int32_t encoder_error[4];
        for (int i = 0; i < 4; i++) {
            encoder_error[i] = actual_encoder_delta[i] - expected_encoder_delta[i];
        }
        LOGINFO("[ENC_ERROR] LF=%d, RF=%d, LB=%d, RB=%d", 
               encoder_error[0], encoder_error[1], encoder_error[2], encoder_error[3]);
    }
}

/**
 * @brief 电机控制调试信息输出
 * @param vy Y方向速度命令
 * @param send_start_time 发送开始时间
 * @param send_success 发送成功标志数组
 * @param motor_debug_count 电机调试计数器
 */
static void Chassis_MotorDebugOutput(float vy, uint32_t send_start_time, 
                                   bool* send_success, uint32_t motor_debug_count)
{
    uint32_t send_end_time = DWT_GetTimeline_ms();
    bool sync_success = Emm_V5_CAN_Synchronous_motion(0);
    
    // 限制调试信息输出频率，仅在有运动时输出
    if (motor_debug_count % 100 == 0 && (fabsf(vy) > 0.01f)) {
        char time_str[16];
        Float2Str(time_str, (float)(send_end_time - send_start_time));
        LOGINFO("[MOTOR_SEND] 发送用时: %sms, 成功: LF=%d RF=%d LB=%d RB=%d, 同步=%d", 
               time_str, 
               send_success[0], send_success[1], send_success[2], send_success[3], sync_success);
    }
}
#endif
