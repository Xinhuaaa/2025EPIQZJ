/**
  ******************************************************************************
  * @file    chassis.c
  * @brief   底盘运动控制实现
  ******************************************************************************
  * @attention
  *
  * 描述：简化实现底盘运动控制，接受目标坐标(x,y,yaw)，通过闭环控制使底盘
  *      移动到指定位置并实时消除误差。
  *
  ******************************************************************************
  */
#include "stm32f4xx_hal.h"     
#include "chassis.h"
#include "PID.h"
#include "Hwt101.h"
#include "ZDT_X42_V2.h"  
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"

/* 外部声明 */
extern float Angle;  // 从Hwt101.h中引用角度信息（单位：度）

/* 数学常量与转换宏 */
#define PI                    3.14159265358979323846f
#define DEG_TO_RAD(x)         ((x) * PI / 180.0f)
#define RAD_TO_DEG(x)         ((x) * 180.0f / PI)

/* 底盘物理参数 */
#define CHASSIS_WIDTH         0.19f    // 底盘宽度，单位m
#define CHASSIS_LENGTH        0.2f    // 底盘长度，单位m
#define WHEEL_DIAMETER        0.077f   // 麦轮直径，单位m
#define CHASSIS_WHEEL_RADIUS  (WHEEL_DIAMETER / 2.0f)
#define MECANUM_FACTOR        (CHASSIS_WIDTH + CHASSIS_LENGTH)
#define ENCODER_COUNTS_PER_REV 65536.0f  // 编码器一圈的计数值
#define ENCODER_COUNTS_PER_M  (ENCODER_COUNTS_PER_REV / (WHEEL_DIAMETER * PI))

/* 控制参数 */
#define POSITION_TOLERANCE_XY 0.02f   // 位置允差，单位m
#define POSITION_TOLERANCE_YAW 1.5f   // 角度允差，单位度
#define CHASSIS_TASK_PERIOD   10      // 控制周期，单位ms

// X42电机CAN ID
#define MOTOR_LF_ID 5  // 左前轮ID
#define MOTOR_RF_ID 6  // 右前轮ID
#define MOTOR_LB_ID 7  // 左后轮ID
#define MOTOR_RB_ID 8  // 右后轮ID

int32_t current_encoder[4] = {0};  // 改为int32_t类型以存储累积编码器值

/* 类型定义 */
typedef struct {
    float x;      // X坐标，单位米
    float y;      // Y坐标，单位米
    float yaw;    // 偏航角，单位度，范围[-180, 180]
} ChassisState_t;

typedef struct {
    PIDInstance    x;      // X方向PID控制器
    PIDInstance    y;      // Y方向PID控制器
    PIDInstance    yaw;    // 偏航角PID控制器
} ChassisPID_t;

/* 全局变量 */
static ChassisState_t  g_current_pos = {0};     // 当前位置
static ChassisState_t  g_target_pos = {0};      // 目标位置
static ChassisPID_t    g_pid = {0};             // PID控制器
static uint16_t        g_encoder_values[4] = {0}; // 编码器值
static TaskHandle_t    g_chassis_task_handle = NULL;
static bool            g_chassis_task_running = false;
static int32_t         prev_encoder[4] = {0};   

/* 麦轮运动学矩阵 - 从底盘速度到轮子速度的映射 */
static const float g_mecanum_matrix[4][3] = {
    { 1.0f,  -1.0f, -MECANUM_FACTOR},  // 左前轮
    { 1.0f, 1.0f,  MECANUM_FACTOR},  // 右前轮
    { 1.0f, 1.0f, -MECANUM_FACTOR},  // 左后轮
    { 1.0f,  -1.0f,  MECANUM_FACTOR}   // 右后轮
};

/* 前向声明 */
static void Chassis_Task(void *argument);
static float NormalizeAngleDeg(float angle_deg);

/**
  * @brief  底盘初始化函数
  */
void Chassis_Init(void)
{
    // 定义电机ID数组
    uint8_t motor_ids[4] = {MOTOR_LF_ID, MOTOR_RF_ID, MOTOR_LB_ID, MOTOR_RB_ID};
    
    // 初始化ZDT_X42_V2电机驱动
    if (!ZDT_X42_V2_Init(motor_ids, 4)) {
        printf("X42电机驱动初始化失败！\r\n");
        return;  // 如果初始化失败，直接返回
    }
    
    // 配置X、Y方向PID控制器（相同参数）
    PID_Init_Config_s pid_config_xy = {
        .Kp = 0.5f,               // 比例系数
        .Ki = 0.0f,              // 积分系数
        .Kd = 0.0f,               // 微分系数9
        .MaxOut = 1.0f,           // 最大速度0.5m/s
        .DeadBand = 0.00f,        // 1cm死区
        .Improve = PID_Integral_Limit ,
        .IntegralLimit = 1.0f,    // 积分限幅
        .Output_LPF_RC = 0.0f,     // 低通滤波常数
        .MaxAccel = 0.0f,
        .MaxJerk = 0.0f,
    };
    PIDInit(&g_pid.x, &pid_config_xy);
    PIDInit(&g_pid.y, &pid_config_xy);
    
    // 配置偏航角PID控制器
    PID_Init_Config_s pid_config_yaw = {
        .Kp = 2.0f,               // 比例系数
        .Ki = 0.0f,               // 积分系数
        .Kd = 0.00f,              // 微分系数
        .MaxOut = 10.0f,          // 最大角速度30度/s
        .DeadBand = 0.5f,         // 0.5度死区
        .Improve = PID_Integral_Limit | PID_OutputFilter | PID_Trapezoid_Intergral|PID_ChangingIntegrationRate,
        .IntegralLimit = 0.0f,   // 积分限幅（度）
        .Output_LPF_RC = 0.0f     // 低通滤波常数
    };
    PIDInit(&g_pid.yaw, &pid_config_yaw);
    
    // 电机初始化：使能、设置模式、清零编码器
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t motor_id = motor_ids[i];
        
        // 使能电机
        ZDT_X42_V2_En_Control(motor_id, true, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        
        // 设置闭环模式
        ZDT_X42_V2_Modify_Ctrl_Mode(motor_id, true, 2); // 2 = 闭环模式
        vTaskDelay(pdMS_TO_TICKS(10));
        
        // 清零位置
        ZDT_X42_V2_Reset_CurPos_To_Zero(motor_id);
        vTaskDelay(pdMS_TO_TICKS(10));
        
        // 初始化清除故障
        ZDT_X42_V2_Reset_Clog_Pro(motor_id);
        vTaskDelay(pdMS_TO_TICKS(10));
        
        printf("初始化电机ID: %d\r\n", motor_id);
    }
    
    
    // 重置底盘位置
    Chassis_ResetPosition(); 
    
    // 创建底盘控制任务
    g_chassis_task_running = true;
    xTaskCreate(Chassis_Task, "Chassis", 512, NULL, 5, &g_chassis_task_handle);
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
    g_target_pos.yaw = NormalizeAngleDeg(yaw); // 标准化目标角度
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

/**
  * @brief  底盘控制循环
  * @retval 是否到达目标位置
  */
bool Chassis_Control_Loop(void)
{
    uint8_t motor_ids[4] = {MOTOR_LF_ID, MOTOR_RF_ID, MOTOR_LB_ID, MOTOR_RB_ID};
    
    // 1. 更新底盘位置
    
    // 更新底盘朝向角度（来自陀螺仪）
    g_current_pos.yaw = NormalizeAngleDeg(Angle);
    
    // 读取电机编码器值
    ZDT_X42_V2_Get_All_Encoders(current_encoder);
    
    // 计算每个轮子的位移（米）
    float wheel_delta[4] = {0};    
    for (int i = 0; i < 4; i++) {
        int32_t steps = current_encoder[i] - prev_encoder[i];
        wheel_delta[i] = (float)steps / ENCODER_COUNTS_PER_M;
        prev_encoder[i] = current_encoder[i];
    }
    
    float delta_x = (-wheel_delta[0] + wheel_delta[1] - wheel_delta[2] + wheel_delta[3]) / 4.0f;
    float delta_y = (wheel_delta[0] + wheel_delta[1] - wheel_delta[2] - wheel_delta[3]) / 4.0f;    
    
    // 将局部位移转换到全局坐标系
    float yaw_rad = DEG_TO_RAD(g_current_pos.yaw);
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);
    
    float global_dx = delta_x * cos_yaw - delta_y * sin_yaw;
    float global_dy = delta_x * sin_yaw + delta_y * cos_yaw;
    
    // 更新全局位置
    g_current_pos.x += global_dx;
    g_current_pos.y += global_dy;
    
    // 2. 计算位置误差并控制
    
    // 计算控制误差
    float error_x = g_target_pos.x - g_current_pos.x;
    float error_y = g_target_pos.y - g_current_pos.y;
    float error_yaw = NormalizeAngleDeg(g_target_pos.yaw - g_current_pos.yaw);
    float vx = PIDCalculate(&g_pid.x, g_target_pos.x, g_current_pos.x);
    float vy = PIDCalculate(&g_pid.y, g_target_pos.y, g_current_pos.y);  
    float vyaw_deg = PIDCalculate(&g_pid.yaw, g_current_pos.yaw, g_target_pos.yaw);
    
    // 3. 计算轮子速度
    
    // 将全局速度指令转换到底盘坐标系
    yaw_rad = DEG_TO_RAD(g_current_pos.yaw);
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);
    
    float chassis_vx = vx * cos_yaw + vy * sin_yaw;
    float chassis_vy = -vx * sin_yaw + vy * cos_yaw;
    float vyaw_rad = DEG_TO_RAD(vyaw_deg);
    
    // 使用运动学矩阵计算每个轮子的速度
    float wheel_speed[4] = {0};
    for (int i = 0; i < 4; i++) {
        wheel_speed[i] = g_mecanum_matrix[i][0] * chassis_vx + 
                         g_mecanum_matrix[i][1] * chassis_vy + 
                         g_mecanum_matrix[i][2] * vyaw_rad;
    }
    
    // 4. 控制电机
    
    // 将线速度转换为RPM并控制电机
    for (int i = 0; i < 4; i++) {
        float rpm = wheel_speed[i] * 60.0f / (2.0f * PI * CHASSIS_WHEEL_RADIUS);
        uint8_t dir = (rpm >= 0) ? 0 : 1;  // 0=CW, 1=CCW
        float speed = fabsf(rpm);
        
        // 斜率限制（RPM/s）- 平稳加减速
        uint16_t v_ramp = 1000;
        
        // 设置电机速度，使用ZDT_X42_V2的速度控制接口
        ZDT_X42_V2_Velocity_Control(motor_ids[i], dir, v_ramp, speed, 0);
    }
    
    // 5. 判断是否到达目标位置
    bool reached = (fabs(error_x) < POSITION_TOLERANCE_XY) && 
                   (fabs(error_y) < POSITION_TOLERANCE_XY) && 
                   (fabs(error_yaw) < POSITION_TOLERANCE_YAW);
    
    return reached;
}

/**
  * @brief  紧急停止底盘
  */
void Chassis_EmergencyStop(void)
{
    uint8_t motor_ids[4] = {MOTOR_LF_ID, MOTOR_RF_ID, MOTOR_LB_ID, MOTOR_RB_ID};
    
    for (int i = 0; i < 4; i++) {
        // 使用X42电机的立即停止功能
        ZDT_X42_V2_Stop_Now(motor_ids[i], 0);
    }
}

/**
  * @brief  重置底盘位置为原点
  */
void Chassis_ResetPosition(void)
{
    // 重置位置变量
    g_current_pos.x = 0.0f;
    g_current_pos.y = 0.0f;
    g_current_pos.yaw = 0.0f;
}

/**
  * @brief  设置底盘PID控制参数
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
  * @brief  规范化角度到[-180, 180]度
  * @param  angle_deg 输入角度（度）
  * @retval 规范化后的角度（度）
  */
static float NormalizeAngleDeg(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg <= -180.0f) angle_deg += 360.0f;
    return angle_deg;
}