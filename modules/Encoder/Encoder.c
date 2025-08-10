/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include <stdint.h>
#include <stdlib.h> // 添加stdlib.h以使用labs()函数
#define _USE_MATH_DEFINES
#include <math.h> // for sinf, cosf functions
#include "Localization/ekf_localization.h"

// 如果 M_PI 仍然没有定义，手动定义它
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ====================== 用户参数宏定义 ====================== */
#define ENCODER_TICKS_PER_UNIT  53.4f // 假设单位是毫米 (mm)，请根据实际测量值调整！
//X轴往前走时,x轴轮式里程计减小
//Y轴往左走时,Y轴轮式里程计增大
// Y轴里程计相对于车辆中心的X轴偏移量 (毫米)
// 如果Y轴里程计在车辆中心X轴正方向偏移，则为正值
// 例如：如果Y轴里程计在车辆中心右侧 50mm 处
#define Y_ODOMETER_X_OFFSET_MM  0.0f // 请根据实际测量值进行调整！

// X轴里程计相对于车辆中心的Y轴偏移量 (毫米)
// 如果X轴里程计在车辆中心Y轴正方向偏移（即在中心轴线前方），则为正值
// 例如：如果X轴里程计在车辆中心前方 30mm 处
#define X_ODOMETER_Y_OFFSET_MM  0.0f // 请根据实际测量值进行调整！

/* ====================== EKF 噪声调参（x/y 分通道） ====================== */
// 基线参数（通用场景）
static const float EKF_QX_K_BASE   = 1.2e-3f; // x 每 mm 位移过程噪声系数
static const float EKF_QY_K_BASE   = 6.0e-4f; // y 更“紧”，抑制侧漂
static const float EKF_QX_MIN_BASE = 1.0e-4f;
static const float EKF_QY_MIN_BASE = 5.0e-5f;

// 沿 x 直行场景（进一步降低 y 的过程噪声）
static const float EKF_QX_K_XDOM   = 1.5e-3f;
static const float EKF_QY_K_XDOM   = 3.0e-4f;
static const float EKF_QX_MIN_XDOM = 1.0e-4f;
static const float EKF_QY_MIN_XDOM = 3.0e-5f;

/* ====================== 外部全局变量声明 ====================== */
// 声明陀螺仪的全局角度增量变量
// 假设 Angle 是以度为单位，代表从上电以来的总角度增量。
// 确保这个变量在您的项目中已经被定义并实时更新。
extern float Angle; 

/* ====================== 函数声明 ====================== */
void EncoderTask(void *argument);

/* ====================== 编码器里程变量 ====================== */
// 用于计算增量，存储上一次循环时硬件计数器的值
static uint16_t prev_x_hw_count = 0; // 上次X轴里程计硬件计数器值
static uint16_t prev_y_hw_count = 0; // 上次Y轴里程计硬件计数器值

// 用于计算 Angle 增量的上一次值
static float prev_angle_deg = 0.0f; 

// 当前周期的增量位移（单位：毫米）
float delta_x_car_mm = 0.0f; // 车辆在自身X轴方向的增量位移
float delta_y_car_mm = 0.0f; // 车辆在自身Y轴方向的增量位移

// 车辆全局位姿（单位：毫米，角度：弧度）
float global_x_pos = 0.0f;
float global_y_pos = 0.0f;
float global_angle = 0.0f; // 车辆当前Z轴角度，以弧度表示，从0开始累加

// EKF 导出变量（可供其他模块读取）
float ekf_x_mm = 0.0f;
float ekf_y_mm = 0.0f;
float ekf_theta_rad = 0.0f;

// 导出给其他模块使用的位置变量（单位：毫米）
float x_position_units = 0.0f;
float y_position_units = 0.0f;
int32_t delta_ticks_y = 0;
int32_t delta_ticks_x = 0;
/* ====================== 编码器初始化函数 ====================== */

/**
 * @brief  编码器初始化函数
 * @param  None
 * @retval None
 */
void EncoderInit(void)
{
    /* 创建编码器任务 */
    const osThreadAttr_t encoder_task_attributes = {
        .name = "EncoderTask",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityNormal,
    };
    osThreadNew(EncoderTask, NULL, &encoder_task_attributes);

    // 初始化全局位姿
    global_x_pos = 0.0f;
    global_y_pos = 0.0f;
    global_angle = 0.0f; // 角度增量，从0开始累加

    // 初始化 prev_angle_deg 为当前 Angle 的值，确保第一次计算 delta_angle_rad 为 0
    prev_angle_deg = Angle; 

    // 初始化 EKF（以当前原点为 0,0,0）
    ekf_init(0.0f, 0.0f, 0.0f);
    // 可按需调整过程/观测噪声（经验初值）
    // 平移过程噪声与运动量成正比，角度过程噪声与角速度成正比
    ekf_set_process_noise(1e-3f, 2e-3f, 1e-4f, 1e-4f);
    // 启用 x/y 分通道过程噪声（基线）
    ekf_set_process_noise_xy(EKF_QX_K_BASE, EKF_QY_K_BASE,
                             EKF_QX_MIN_BASE, EKF_QY_MIN_BASE);
    // 陀螺仪测量噪声方差（rad^2），数值越小表示越相信陀螺仪角度
    ekf_set_measurement_noise(0.01f);
}

/* ====================== 编码器任务实现 ====================== */

void EncoderTask(void *argument)
{
    /* 启动定时器编码器接口模式 */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // 假设 htim3 对应 Y轴里程计
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // 假设 htim4 对应 X轴里程计

    // 在第一次循环前获取初始计数，避免第一次计算Delta时出现大跳变
    prev_x_hw_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4); // X轴里程计初始计数
    prev_y_hw_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3); // Y轴里程计初始计数


    for(;;)
    {
        /* 读取当前硬件计数器值 */
        uint16_t current_y_hw_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3); // Y轴里程计当前计数
        uint16_t current_x_hw_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4); // X轴里程计当前计数

        /* 计算当前周期内的脉冲变化量，处理16位计数器环绕问题 */
        int32_t raw_delta_y = (int32_t)current_y_hw_count - (int32_t)prev_y_hw_count;
        int32_t raw_delta_x = (int32_t)current_x_hw_count - (int32_t)prev_x_hw_count;
        
        // 处理环绕情况
        if (raw_delta_y > 32767) raw_delta_y -= 65536;
        else if (raw_delta_y < -32768) raw_delta_y += 65536;
        
        if (raw_delta_x > 32767) raw_delta_x -= 65536;
        else if (raw_delta_x < -32768) raw_delta_x += 65536;
        
        delta_ticks_y = (int32_t)raw_delta_y; // Y轴里程计增量
        delta_ticks_x = (int32_t)raw_delta_x; // X轴里程计增量

        // 环绕问题已在前面处理完毕，这段代码不再需要
        // 如果有异常大的增量值，可以在此处增加滤波或限幅代码
        const int32_t MAX_REASONABLE_DELTA = 10000; // 设置一个合理的增量上限
        if (labs((long)delta_ticks_y) > MAX_REASONABLE_DELTA) {
            // 可能是异常值，记录日志或采取其他措施
            delta_ticks_y = 0; // 或设为上一个有效值
        }
        if (labs((long)delta_ticks_x) > MAX_REASONABLE_DELTA) {
            // 可能是异常值，记录日志或采取其他措施
            delta_ticks_x = 0; // 或设为上一个有效值
        }
        
        // 更新上一次的硬件计数器值，为下一次循环做准备
        prev_x_hw_count = current_x_hw_count;
        prev_y_hw_count = current_y_hw_count;

        /* 获取陀螺仪Z轴角度增量 */
        // Angle 已经是总的增量，直接计算本次循环的增量
        float current_angle_deg = Angle;
        float delta_angle_deg = current_angle_deg - prev_angle_deg;
        prev_angle_deg = current_angle_deg; // 更新上一次的 Angle 值
        
        float delta_angle_rad = delta_angle_deg * (M_PI / 180.0f); // 转换为弧度

        /* 计算车辆在自身坐标系下的增量位移 */
        // X轴里程计需要补偿旋转引起的位移
        // 如果 X_ODOMETER_Y_OFFSET_MM 为正（X里程计在车体Y轴正方向，即前方）
        // 且车辆逆时针旋转 (delta_angle_rad > 0) 会导致 X里程计向负X方向移动，所以需要加上补偿
        delta_x_car_mm = ((float)delta_ticks_x / ENCODER_TICKS_PER_UNIT) + (X_ODOMETER_Y_OFFSET_MM * delta_angle_rad);

        // Y轴里程计需要补偿旋转引起的位移
        // 如果 Y_ODOMETER_X_OFFSET_MM 为正（Y里程计在车体X轴正方向，即右侧）
        // 且车辆逆时针旋转 (delta_angle_rad > 0) 会导致 Y里程计向正Y方向移动，所以需要从读数中减去
        delta_y_car_mm = ((float)delta_ticks_y / ENCODER_TICKS_PER_UNIT) - (Y_ODOMETER_X_OFFSET_MM * delta_angle_rad);

        /* 根据运动形态自适应 x/y 噪声：沿 x 直行时，收紧 y 通道噪声 */
        {
            float ax = fabsf(delta_x_car_mm);
            float ay = fabsf(delta_y_car_mm);
            float ath = fabsf(delta_angle_rad);
            // 判定“沿 x 直行”：x 位移显著大于 y，且转角很小
            if (ax > (3.0f * ay) && ath < 0.02f) { // 约 <1.15°
                ekf_set_process_noise_xy(EKF_QX_K_XDOM, EKF_QY_K_XDOM,
                                         EKF_QX_MIN_XDOM, EKF_QY_MIN_XDOM);
            } else {
                ekf_set_process_noise_xy(EKF_QX_K_BASE, EKF_QY_K_BASE,
                                         EKF_QX_MIN_BASE, EKF_QY_MIN_BASE);
            }
        }

        /* EKF 预测：使用车体坐标系位移与航向增量
           注意：项目约定“前进时 X 里程计减小”，为与标准机体系一致，这里对 dx 取反 */
        ekf_predict(-delta_x_car_mm, delta_y_car_mm, delta_angle_rad);

        /* EKF 观测：使用陀螺仪角度（将 Angle(度) 转为弧度）*/
        float yaw_meas_rad = Angle * (M_PI / 180.0f);
        // 将测量包裹到 [-pi, pi]，避免累积角导致数值过大
        if (yaw_meas_rad > M_PI || yaw_meas_rad < -M_PI) {
            float two_pi = 2.0f * (float)M_PI;
            yaw_meas_rad = fmodf(yaw_meas_rad + (float)M_PI, two_pi);
            if (yaw_meas_rad < 0) yaw_meas_rad += two_pi;
            yaw_meas_rad -= (float)M_PI;
        }
        ekf_update_yaw(yaw_meas_rad);

        /* 读取滤波结果并导出 */
        ekf_get_state(&ekf_x_mm, &ekf_y_mm, &ekf_theta_rad);

        global_x_pos = ekf_x_mm;
        global_y_pos = ekf_y_mm;
        global_angle = ekf_theta_rad;

        x_position_units = ekf_x_mm;
        y_position_units = ekf_y_mm;

        /* 轮询周期 */
        osDelay(6); // 10ms 周期，即 100Hz 更新频率
    }
}