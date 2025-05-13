#include "hwt101.h"
#include "main.h"
#include "usart.h"
#include "stdio.h"
#include "math.h"
#include "cmsis_os.h"

#define Debug 0

// FreeRTOS对象
SemaphoreHandle_t HWT101_DataReadySemaphore = NULL;
TaskHandle_t HWT101_TaskHandle = NULL;

uint8_t DMA_buf[30] = {0};
float Angle = 0;  // Z轴角度

// Z轴角度归零命令
uint8_t CALIYAW[] = {0xFF,0xAA,0x76,0x00,0x00};

/**
 * @brief  HWT101初始化
 * @param  None
 * @retval None
 */
void HWT101_Init(void)
{
    
    // 创建信号量
    HWT101_DataReadySemaphore = xSemaphoreCreateBinary();
    
    // 确保信号量创建成功
    configASSERT(HWT101_DataReadySemaphore != NULL);
    
    // 使能IDLE中断
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    
    // 先执行Z轴归零
    HAL_UART_Transmit(&huart2, CALIYAW, sizeof(CALIYAW), HAL_MAX_DELAY);
    // 发送两次以确保命令被接收
    HAL_UART_Transmit(&huart2, CALIYAW, sizeof(CALIYAW), HAL_MAX_DELAY);
    // 等待足够长的时间确保归零操作完成
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 归零完成后，再开启DMA接收
    HAL_UART_Receive_DMA(&huart2, DMA_buf, MAXSIZE);
}

/**
 * @brief  HWT101数据处理
 */
void HWT101_Data_Proc(void)
{
    // 计算角度，将原始数据转换为角度值
    Angle = (float)((DMA_buf[18]<<8)|DMA_buf[17])/32768*180;
    // 将角度范围转换为-180到180度
    if (Angle > 180.0f)
    {
        Angle = Angle - 360.0f;
    }
    // 校验和
    if(DMA_buf[21] != (uint8_t)(0x55+0x53+DMA_buf[17]+DMA_buf[18]+DMA_buf[19]+DMA_buf[20]))
    {
        #if Debug
        printf("HWT101校验错误!\r\n");
        #endif
    }
    
#if Debug
    printf("角度: %f\r\n", Angle);
#endif

    // 重新开启DMA接收(直接覆盖原缓冲区)
    HAL_UART_Receive_DMA(&huart2, DMA_buf, MAXSIZE);
}

/**
 * @brief  HWT101传感器任务函数
 * @param  argument: 未使用
 * @retval None
 */
void HWT101_Task(void *argument)
{
    // 初始化HWT101
    HWT101_Init();
    
    // 任务主循环
    for(;;)
    {
        // 等待数据就绪信号量
        if(xSemaphoreTake(HWT101_DataReadySemaphore, portMAX_DELAY) == pdTRUE)
        {
            // 处理接收到的数据
            HWT101_Data_Proc();
        }
    }
}

/**
 * @brief  创建HWT101任务
 * @param  None
 * @retval None
 */
void HWT101_TaskInit(void)
{
    // 任务参数
    const osThreadAttr_t HWT101_attributes = {
        .name = "HWT101Task",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityNormal,
    };
    
    // 创建任务
    HWT101_TaskHandle = osThreadNew(HWT101_Task, NULL, &HWT101_attributes);
    
    // 确保任务创建成功
    configASSERT(HWT101_TaskHandle != NULL);
}

/**
 * @brief  UART IDLE中断回调函数 - 需在stm32f4xx_it.c文件中实现
 * @note   此函数需要在UART5的IDLE中断处理函数中调用
 * @param  None
 * @retval None
 */
void HWT101_UART_IDLECallback(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // 给出信号量，通知任务处理数据
    xSemaphoreGiveFromISR(HWT101_DataReadySemaphore, &xHigherPriorityTaskWoken);
    
    // 如果释放信号量导致更高优先级的任务就绪，则请求上下文切换
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

