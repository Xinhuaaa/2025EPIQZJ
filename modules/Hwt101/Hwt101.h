#ifndef __HWT101_H
#define __HWT101_H

#include "main.h"
#include "stdio.h"
#include "string.h"
/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define MAXSIZE 30

/* 初始化相关参数 */
#define INIT_SAMPLE_COUNT 10  // 初始化采样次数
#define MAX_ANGLE_DEVIATION 30.0f  // 最大角度偏差阈值(度)

extern UART_HandleTypeDef huart2;


extern uint8_t DMA_buf[30];
/* FreeRTOS信号量和任务句柄 */
extern SemaphoreHandle_t HWT101_DataReadySemaphore;
extern TaskHandle_t HWT101_TaskHandle;

/* 角度数据 */
extern float Angle;        // Z轴累计角度
extern float PrevRawAngle; // 上一次的原始角度值
extern float TotalAngle;   // 累计角度值

/* 函数声明 */
void HWT101_Init(void);
void HWT101_Data_Proc(void);
void HWT101_Task(void *argument);
void HWT101_TaskInit(void);
void HWT101_UART_IDLECallback(void); 

#endif
