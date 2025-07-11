/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include <stdint.h>

/* ====================== 用户参数宏定义 ====================== */
#define ENCODER_WRAP_THRESHOLD  10000
#define ENCODER_TICKS_PER_UNIT  53.4f

/* ====================== 函数声明 ====================== */
void EncoderTask(void *argument);

/* ====================== 编码器里程变量 ====================== */
int32_t encoder1_raw_count = 0;
int32_t encoder2_raw_count = 0;

int32_t encoder1_base_count = 0;
int32_t encoder2_base_count = 0;

int32_t encoder1_wrap_count = 0;
int32_t encoder2_wrap_count = 0;

int32_t x_total_ticks = 0;
int32_t y_total_ticks = 0;
int32_t x_total_ticks_shadow = 0;

float x_position_units = 0.0f;
float y_position_units = 0.0f;

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
}

/* ====================== 编码器任务实现 ====================== */

void EncoderTask(void *argument)
{
    /* 启动定时器编码器接口模式 */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    for(;;)
    {
        /* 读取当前CNT并做short环绕处理 */
        int16_t cnt1 = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
        int16_t cnt2 = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

        /* 叠加到基数（防止溢出） */
        encoder1_raw_count = cnt1 + encoder1_base_count;
        encoder2_raw_count = cnt2 + encoder2_base_count;

        /* 处理正溢出 */
        if (encoder1_raw_count >= ENCODER_WRAP_THRESHOLD)
        {
            encoder1_base_count = encoder1_raw_count % ENCODER_WRAP_THRESHOLD;
            encoder1_wrap_count++;
            __HAL_TIM_SET_COUNTER(&htim3, 0);
        }
        else if (encoder1_raw_count <= -ENCODER_WRAP_THRESHOLD)
        {
            encoder1_base_count = encoder1_raw_count % ENCODER_WRAP_THRESHOLD;
            encoder1_wrap_count--;
            __HAL_TIM_SET_COUNTER(&htim3, 0);
        }

        if (encoder2_raw_count >= ENCODER_WRAP_THRESHOLD)
        {
            encoder2_base_count = encoder2_raw_count % ENCODER_WRAP_THRESHOLD;
            encoder2_wrap_count++;
            __HAL_TIM_SET_COUNTER(&htim4, 0);
        }
        else if (encoder2_raw_count <= -ENCODER_WRAP_THRESHOLD)
        {
            encoder2_base_count = encoder2_raw_count % ENCODER_WRAP_THRESHOLD;
            encoder2_wrap_count--;
            __HAL_TIM_SET_COUNTER(&htim4, 0);
        }

        /* 计算总累计脉冲数 */
        x_total_ticks = encoder2_wrap_count * ENCODER_WRAP_THRESHOLD + encoder2_raw_count;
        y_total_ticks = encoder1_wrap_count * ENCODER_WRAP_THRESHOLD + encoder1_raw_count;
        x_total_ticks_shadow = x_total_ticks;  // 备份

        /* 换算成物理单位（给PID控制） */
        x_position_units = x_total_ticks / ENCODER_TICKS_PER_UNIT;
        y_position_units = y_total_ticks / ENCODER_TICKS_PER_UNIT;



        /* 轮询周期 */
        osDelay(10);
    }
}
