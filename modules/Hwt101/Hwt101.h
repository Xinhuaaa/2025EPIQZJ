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

extern UART_HandleTypeDef huart2;


extern uint8_t DMA_buf[30];
/* FreeRTOS信号量和任务句柄 */
extern SemaphoreHandle_t HWT101_DataReadySemaphore;
extern TaskHandle_t HWT101_TaskHandle;

/* 角度数据 */
extern float Angle;

/* 函数声明 */
void HWT101_Init(void);
void HWT101_Data_Proc(void);
void HWT101_Task(void *argument);
void HWT101_TaskInit(void);
void HWT101_UART_IDLECallback(void); 

#endif
