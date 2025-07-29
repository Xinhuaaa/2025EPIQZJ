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
#include "ADRC.h"  
#include "Hwt101.h"
#include "Emm_V5_CAN.h"  
#include "Encoder.h"
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_log.h"
#include "BSP_dwt.h"
#include "cmsis_os.h"

/* 外部声明 */
extern float Angle;  // 从Hwt101.h中引用角度信息（单位：度）
extern float x_position_units;  // 外部X位置，单位mm
extern float y_position_units;  // 外部Y位置，单位mm

/* 数学常量与转换宏 */
#define PI                    3.14159265358979323846f
#define DEG_TO_RAD(x)         ((x) * PI / 180.0f)
#define RAD_TO_DEG(x)         ((x) * 180.0f / PI)

/* 底盘物理参数 */
#define CHASSIS_WIDTH         0.435f    // 底盘宽度，单位m
#define CHASSIS_LENGTH        0.51f    // 底盘长度，单位m
#define WHEEL_DIAMETER        0.078f   // 麦轮直径，单位m
#define CHASSIS_WHEEL_RADIUS  (WHEEL_DIAMETER / 2.0f)
#define MECANUM_FACTOR        (CHASSIS_WIDTH + CHASSIS_LENGTH)
#define ENCODER_COUNTS_PER_REV 65536.0f  // 编码器一圈的计数值
#define ENCODER_COUNTS_PER_M  (ENCODER_COUNTS_PER_REV / (WHEEL_DIAMETER * PI))

/* 控制参数 */
#define POSITION_TOLERANCE_XY 0.02f   // 位置允差，单位m
#define POSITION_TOLERANCE_YAW 1.5f   // 角度允差，单位度
#define CHASSIS_TASK_PERIOD   15      // 控制周期，单位ms
#define ENCODER_TASK_PERIOD  8  // 5ms周期，比底盘控制任务更快


// X42电机CAN ID
#define MOTOR_LF_ID 5  // 左前轮ID
#define MOTOR_RF_ID 6  // 右前轮ID
#define MOTOR_LB_ID 7  // 左后轮ID
#define MOTOR_RB_ID 8  // 右后轮ID

int32_t current_encoder[4] = {0};  // 改为int32_t类型以存储累积编码器值
float wheel_speed[4] = {0};
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

// 底盘ADRC控制器结构体
typedef struct {
    ADRC_Controller x;     // X方向ADRC控制器
    ADRC_Controller y;     // Y方向ADRC控制器
    ADRC_Controller yaw;   // 偏航角ADRC控制器
} ChassisADRC_t;

/* 全局变量 */
static ChassisState_t  g_current_pos = {0};     // 当前位置
static ChassisState_t  g_target_pos = {0};      // 目标位置
static ChassisPID_t    g_pid = {0};             // PID控制器（保留用于兼容）
static ChassisADRC_t   g_adrc = {0};            // ADRC控制器
static volatile int32_t g_encoder_values[4] = {0}; // 编码器累积值，由编码器任务更新
static TaskHandle_t    g_chassis_task_handle = NULL;
static bool            g_chassis_task_running = false;
static int32_t         prev_encoder[4] = {0};

/* 麦轮运动学矩阵 - 从底盘速度到轮子速度的映射 */
/* 标准麦轮配置：左前右后轮滚轮朝右前，右前左后轮滚轮朝左前 */
/* 定义底盘坐标系：X轴向前，Y轴向左，Z轴向上 */
static const float g_mecanum_matrix[4][3] = {
    { -1.0f,  1.0f, -MECANUM_FACTOR},  // 左前轮 (LF)
    { -1.0f, -1.0f,  MECANUM_FACTOR},  // 右前轮 (RF)
    { -1.0f, -1.0f, -MECANUM_FACTOR},  // 左后轮 (LB) 
    { -1.0f,  1.0f,  MECANUM_FACTOR}   // 右后轮 (RB)
};

/* 前向声明 */
static void Chassis_Task(void *argument);
static void Encoder_Read_Task(void *argument);

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
    // 配置X、Y方向PID控制器（保留但不使用，用于兼容）
    PID_Init_Config_s pid_config_x = {
        .Kp = 0.567f,               // 比例系数
        .Ki = 0.001f,              // 积分系数
        .Kd = 0.000f,               // 微分系数
        .MaxOut = 1.3f,           // 最大速度0.5m/s
        .DeadBand = 0.00f,        // 1cm死区
        .Improve = PID_Integral_Limit |PID_OutputFilter,
        .IntegralLimit = 0.35f,    // 积分限幅
        .Output_LPF_RC = 0.0f,     // 低通滤波常数
        .MaxAccel = 0.0f,
        .MaxJerk = 0.0f,
    };
    PID_Init_Config_s pid_config_y = {
        .Kp = 0.581f,               // 比例系数
        .Ki = 0.01f,              // 积分系数
        .Kd = 0.000f,               // 微分系数
        .MaxOut = 1.0f,           // 最大速度0.5m/s
        .DeadBand = 0.00f,        // 1cm死区
        .Improve = PID_Integral_Limit |PID_OutputFilter,
        .IntegralLimit = 0.35f,    // 积分限幅
        .Output_LPF_RC = 0.0f,     // 低通滤波常数
        .MaxAccel = 0.0f,
        .MaxJerk = 0.0f,
    };
    PIDInit(&g_pid.x, &pid_config_x);
    PIDInit(&g_pid.y, &pid_config_y);
    
    // 配置偏航角PID控制器（保留但不使用，用于兼容）
    PID_Init_Config_s pid_config_yaw = {
        .Kp = 0.573f,               // 比例系数
        .Ki = 0.28f,               // 积分系数
        .Kd = 0.00f,              // 微分系数
        .MaxOut = 30.0f,          // 最大角速度10度/s
        .DeadBand = 0.0f,         // 0.5度死区
        .Improve = PID_Integral_Limit | PID_OutputFilter | PID_Trapezoid_Intergral|PID_ChangingIntegrationRate,
        .IntegralLimit = 0.0f,    // 积分限幅（度）
        .Output_LPF_RC = 0.0f     // 低通滤波常数
    };
    PIDInit(&g_pid.yaw, &pid_config_yaw);
    
      // 初始化ADRC控制器
    // 创建XY方向ADRC配置结构体
    // ADRC_Init_Config_t adrc_config_xy = {
    //     .r = 0.20f,               // 跟踪速度因子（降低以减缓跟踪速度）
    //     .h = CHASSIS_TASK_PERIOD/ 1000.0f, // 积分步长
    //     .b0 = 1.5f,              // 系统增益（降低以减少过冲）
    //     .max_output = 0.4f,       // 最大输出速度0.5m/s
    //     .w0 = 0.30f,
    //     .beta01 = 40,          // ESO 
    //     .beta02 = 40,
    //     .beta03 =0.01,
    //     .beta1 = 0.15f,            // NLSEF参数
    //     .beta2 = 1.5f,
    //     .alpha1 = 0.31f,
    //     .alpha2 = 0.75f,
    //     .delta = 0.1f
    // };
      // 创建偏航角ADRC配置结构体
    // ADRC_Init_Config_t adrc_config_yaw = {
    //     .r = 3.8f,               // 跟踪速度因子（降低以减缓跟踪速度）
    //     .h = CHASSIS_TASK_PERIOD/ 1000.0f, // 积分步长
    //     .b0 = 0.05f,              // 系统增益（降低以减少过冲）
    //     .max_output = 15.0f,       // 最大输出速度0.5m/s
    //     .w0 = 0.00f,
    //     .beta01 = 75.0,          // ESO 
    //     .beta02 = 65.0,
    //     .beta03 =0.9,
    //     .beta1 = 1.3f,         // NLSEF参数
    //     .beta2 = 3.0f,
    //     .alpha1 = 1.8f,
    //     .alpha2 = 0.37f,
    //     .delta = 0.1f
    // };
    
    // 初始化控制器
    // ADRC_Init(&g_adrc.x, &adrc_config_xy);
    // ADRC_Init(&g_adrc.y, &adrc_config_xy);
    // ADRC_Init(&g_adrc.yaw, &adrc_config_yaw);
    
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
    // xTaskCreate(Encoder_Read_Task, "Encoder_Read", 256, NULL, 6, NULL);
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
/**
  * @brief  底盘控制循环
  * @retval 是否到达目标位置
  */
bool Chassis_Control_Loop(void)
{
    uint8_t motor_ids[4] = {MOTOR_LF_ID, MOTOR_RF_ID, MOTOR_LB_ID, MOTOR_RB_ID};
    
    // 1. 更新底盘位置
    // 更新底盘朝向角度（来自陀螺仪）
    g_current_pos.yaw = Angle;
    
    for (int i = 0; i < 4; i++) {
        current_encoder[i] = g_encoder_values[i];
    }
    
    // 计算每个轮子的位移（米）
    float wheel_delta[4] = {0};    
    for(int i = 0; i < 4; i++)
    {
        int32_t steps = current_encoder[i] - prev_encoder[i];
        wheel_delta[i] = (float)steps / ENCODER_COUNTS_PER_M;
        prev_encoder[i] = current_encoder[i];
    }

    // 麦轮运动学正解：从轮速到底盘速度
    // 重新修正麦轮运动学正解，确保与我们定义的坐标系一致
    // 麦轮坐标系：X轴向前，Y轴向左
    float delta_x = (-wheel_delta[0] + wheel_delta[1] - wheel_delta[2] + wheel_delta[3]) / 4.0f;
    float delta_y = (wheel_delta[0] + wheel_delta[1] - wheel_delta[2] - wheel_delta[3]) / 4.0f;    
    
    // 将局部位移转换到全局坐标系
    float yaw_rad = DEG_TO_RAD(g_current_pos.yaw);
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);
    
    // 局部坐标系到全局坐标系的转换
    // 重新修正旋转矩阵，确保方向一致性
    float global_dx = delta_x * cos_yaw + delta_y * sin_yaw;
    float global_dy = delta_x * sin_yaw - delta_y * cos_yaw;

    // 更新全局位置
    // 注释掉原有的两种方式，使用里程计提供的位置，并确保坐标系一致
    // 修正：x和y轴的符号需要与全局坐标系定义一致
    g_current_pos.x = x_position_units/1000.0; // X轴取反，使前进为正
    g_current_pos.y = y_position_units/1000.0; // Y轴也取反，使左移为正
    
    // 2. 计算位置误差并控制
    // 计算控制误差
    float error_x = g_target_pos.x - g_current_pos.x;
    float error_y = g_target_pos.y - g_current_pos.y;
    
    // 使用ADRC控制器计算控制量（全局坐标系）
    float vx_global = PIDCalculate(&g_pid.x, g_current_pos.x, g_target_pos.x);
    float vy_global = PIDCalculate(&g_pid.y, g_current_pos.y, g_target_pos.y);
    float vyaw_deg = PIDCalculate(&g_pid.yaw, g_current_pos.yaw, g_target_pos.yaw);
    // float vx_global = 0;
    // float vy_global = 0;
    // float vyaw_deg = PIDCalculate(&g_pid.yaw, g_current_pos.yaw, g_target_pos.yaw);
    
    // 3. 将全局坐标系速度转换到底盘局部坐标系进行轮速计算
    // 旋转矩阵公式: [cos(θ) sin(θ); -sin(θ) cos(θ)] * [vx_global; vy_global]
    float chassis_vx = vx_global * cos_yaw + vy_global * sin_yaw;
    float chassis_vy = -vx_global * sin_yaw + vy_global * cos_yaw; 
    float vyaw_rad = DEG_TO_RAD(vyaw_deg);

    for (int i = 0; i < 4; i++) {
        wheel_speed[i] = g_mecanum_matrix[i][0] * chassis_vx + 
                         g_mecanum_matrix[i][1] * chassis_vy + 
                         g_mecanum_matrix[i][2] * vyaw_rad;
    }
    
    for (int i = 0; i < 4; i++) {
        float rpm = wheel_speed[i] * 60.0f / (2.0f * PI * CHASSIS_WHEEL_RADIUS);
        uint8_t dir = (rpm >= 0) ? 0 : 1;  // 0=CW, 1=CCW
        float speed = fabsf(rpm);
        uint16_t acc = 100; // 使用数组中的加速度值

        Emm_V5_CAN_Vel_Control(motor_ids[i], dir, speed, acc, 1);
        
        // 在发送之间添加小延时，确保CAN消息不会重叠
        if (i < 3) {
            DWT_Delay_ms(2.0f);  // 增加到2ms延时，确保CAN帧不重叠
        }
    }
    
    Emm_V5_CAN_Synchronous_motion(0);
    
    // 计算偏航角误差
    float error_yaw = g_target_pos.yaw - g_current_pos.yaw;
    while (error_yaw > 180.0f) error_yaw -= 360.0f;
    while (error_yaw < -180.0f) error_yaw += 360.0f;
    
    // 检测x、y、yaw三个方向是否都到达目标位置
    bool reached_target = (fabsf(error_x) < POSITION_TOLERANCE_XY) && 
                         (fabsf(error_y) < POSITION_TOLERANCE_XY) && 
                         (fabsf(error_yaw) < POSITION_TOLERANCE_YAW);
    
    return reached_target;
}

/**
  * @brief  紧急停止底盘
  */
void Chassis_EmergencyStop(void)
{
    uint8_t motor_ids[4] = {MOTOR_LF_ID, MOTOR_RF_ID, MOTOR_LB_ID, MOTOR_RB_ID};
    
    for (int i = 0; i < 4; i++) {
        // 使用X42电机的立即停止功能
        Emm_V5_CAN_Stop_Now(motor_ids[i], 0);
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
  * @brief  设置X轴PID控制参数
  * @param  kp X方向比例系数
  * @param  ki X方向积分系数
  * @param  kd X方向微分系数
  */
void Chassis_SetXPIDParams(float kp, float ki, float kd)
{
    // 更新X方向PID参数
    g_pid.x.Kp = kp;
    g_pid.x.Ki = ki;
    g_pid.x.Kd = kd;
}

/**
  * @brief  设置Y轴PID控制参数
  * @param  kp Y方向比例系数
  * @param  ki Y方向积分系数
  * @param  kd Y方向微分系数
  */
void Chassis_SetYPIDParams(float kp, float ki, float kd)
{
    // 更新Y方向PID参数
    g_pid.y.Kp = kp;
    g_pid.y.Ki = ki;
    g_pid.y.Kd = kd;
}

/**
  * @brief  设置偏航角PID控制参数
  * @param  kp 偏航角比例系数
  * @param  ki 偏航角积分系数
  * @param  kd 偏航角微分系数
  */
void Chassis_SetYawPIDParams(float kp, float ki, float kd)
{
    // 更新偏航角PID参数
    g_pid.yaw.Kp = kp;
    g_pid.yaw.Ki = ki;
    g_pid.yaw.Kd = kd;
}

/**
  * @brief  设置底盘所有PID控制参数(兼容旧接口)
  * @param  kp_x X方向比例系数
  * @param  ki_x X方向积分系数
  * @param  kd_x X方向微分系数
  * @param  kp_y Y方向比例系数
  * @param  ki_y Y方向积分系数
  * @param  kd_y Y方向微分系数
  * @param  kp_yaw 偏航角比例系数
  * @param  ki_yaw 偏航角积分系数
  * @param  kd_yaw 偏航角微分系数
  */
void Chassis_SetPIDParams(float kp_x, float ki_x, float kd_x, 
                         float kp_y, float ki_y, float kd_y,
                         float kp_yaw, float ki_yaw, float kd_yaw)
{
    Chassis_SetXPIDParams(kp_x, ki_x, kd_x);
    Chassis_SetYPIDParams(kp_y, ki_y, kd_y);
    Chassis_SetYawPIDParams(kp_yaw, ki_yaw, kd_yaw);
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

/**
  * @brief  阻塞式移动底盘到指定位置
  * @param  x   目标X坐标，单位m
  * @param  y   目标Y坐标，单位m
  * @param  yaw 目标偏航角，单位度，范围(-180, 180]
  * @param  timeout_ms 超时时间，单位ms，设为0表示无超时限制
  * @return 是否成功到达目标位置（超时返回false）
  */
bool Chassis_MoveToPosition_Blocking(float x, float y, float yaw, uint32_t timeout_ms)
{
    // 设置目标位置
    Chassis_SetTargetPosition(x, y, yaw);
    
    // 用于计算超时的变量
    float start_time = DWT_GetTimeline_ms();
    bool has_timeout = (timeout_ms > 0);
    bool is_timeout = false;
    
    // 等待直到到达目标位置或超时
    while (1) {
        // 检查是否已到达目标位置（只检查位置误差，不执行控制循环）
        // 计算控制误差
        float error_x = g_target_pos.x - g_current_pos.x;
        float error_y = g_target_pos.y - g_current_pos.y;
        float error_yaw = g_target_pos.yaw - g_current_pos.yaw;
        
        // 标准化偏航角误差到[-180, 180]
        while (error_yaw > 180.0f) error_yaw -= 360.0f;
        while (error_yaw < -180.0f) error_yaw += 360.0f;
        
        // 检测x、y、yaw三个方向是否都到达目标位置
        bool reached = (fabsf(error_x) < POSITION_TOLERANCE_XY) && 
                      (fabsf(error_y) < POSITION_TOLERANCE_XY) && 
                      (fabsf(error_yaw) < POSITION_TOLERANCE_YAW);
        
        if (reached) {
            return true;  // 成功到达目标位置
        }
        
        // 检查是否超时
        if (has_timeout) {
            float elapsed = DWT_GetTimeline_ms() - start_time;
            if (elapsed > (float)timeout_ms) {
                is_timeout = true;
                break;  // 超时，退出循环
            }
        }
        
        // 短暂延时，避免过多CPU占用
        osDelay(100);  // 100ms延时，可根据需要调整
    }
}
bool Chassis_MoveToY_Blocking(float y, uint32_t timeout_ms)
{
    // 设置目标位置
    g_target_pos.y = y;
    
    // 用于计算超时的变量
    float start_time = DWT_GetTimeline_ms();
    bool has_timeout = (timeout_ms > 0);
    bool is_timeout = false;
    
    // 等待直到到达目标位置或超时
    while (1) {
        // 检查是否已到达目标位置（只检查位置误差，不执行控制循环）
        // 计算控制误差
        float error_x = g_target_pos.x - g_current_pos.x;
        float error_y = g_target_pos.y - g_current_pos.y;
        float error_yaw = g_target_pos.yaw - g_current_pos.yaw;
        
        // 标准化偏航角误差到[-180, 180]
        while (error_yaw > 180.0f) error_yaw -= 360.0f;
        while (error_yaw < -180.0f) error_yaw += 360.0f;
        
        // 检测x、y、yaw三个方向是否都到达目标位置
        bool reached = (fabsf(error_x) < POSITION_TOLERANCE_XY) && 
                      (fabsf(error_y) < POSITION_TOLERANCE_XY) && 
                      (fabsf(error_yaw) < POSITION_TOLERANCE_YAW);
        
        if (reached) {
            return true;  // 成功到达目标位置
        }
        
        // 检查是否超时
        if (has_timeout) {
            float elapsed = DWT_GetTimeline_ms() - start_time;
            if (elapsed > (float)timeout_ms) {
                is_timeout = true;
                break;  // 超时，退出循环
            }
        }
        
        // 短暂延时，避免过多CPU占用
        osDelay(100);  // 100ms延时，可根据需要调整
    }
    

    return !is_timeout;  // 如果不是因为超时退出，则返回成功
}
