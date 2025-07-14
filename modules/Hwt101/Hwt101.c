#include "hwt101.h"
#include "main.h"
#include "usart.h"
#include "stdio.h"
#include "math.h"
#include "cmsis_os.h"

#define Debug 0
#define INIT_SAMPLE_COUNT 10  // 初始化采样次数
#define MAX_ANGLE_DEVIATION 30.0f  // 最大角度偏差阈值(度)

// FreeRTOS对象
SemaphoreHandle_t HWT101_DataReadySemaphore = NULL;
TaskHandle_t HWT101_TaskHandle = NULL;

uint8_t DMA_buf[30] = {0};
float Angle = 0;  // Z轴累计角度
float PrevRawAngle = 0; // 上一次的原始角度值
float TotalAngle = 0;   // 累计角度值

// Z轴角度归零命令
uint8_t CALIYAW[] = {0xFF,0xAA,0x76,0x00,0x00};

// 初始化相关变量
static uint8_t init_complete = 0;  // 初始化完成标志

/**
 * @brief  获取原始角度值
 * @param  None
 * @retval 原始角度值(度)
 */
static float HWT101_GetRawAngle(void)
{
    return (float)((DMA_buf[18]<<8)|DMA_buf[17])/32768*180;
}

/**
 * @brief  角度标准化到[-180, 180]范围
 * @param  angle: 输入角度
 * @retval 标准化后的角度
 */
static float HWT101_NormalizeAngle(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

/**
 * @brief  计算角度数组的平均值(去除异常值)
 * @param  angles: 角度数组
 * @param  count: 数组长度
 * @retval 平均角度值
 */
static float HWT101_CalculateFilteredAverage(float *angles, uint8_t count)
{
    if(count == 0) return 0.0f;
    
    // 先计算初始平均值
    float sum = 0.0f;
    for(uint8_t i = 0; i < count; i++) {
        sum += angles[i];
    }
    float initial_avg = sum / count;
    
    // 去除偏差过大的值，重新计算平均值
    sum = 0.0f;
    uint8_t valid_count = 0;
    
    for(uint8_t i = 0; i < count; i++) {
        float deviation = fabs(angles[i] - initial_avg);
        // 处理角度穿越问题
        if(deviation > 180.0f) {
            deviation = 360.0f - deviation;
        }
        
        if(deviation <= MAX_ANGLE_DEVIATION) {
            sum += angles[i];
            valid_count++;
        }
    }
    
    // 如果有效值太少，使用所有值
    if(valid_count < count / 2) {
        sum = 0.0f;
        for(uint8_t i = 0; i < count; i++) {
            sum += angles[i];
        }
        return sum / count;
    }
    
    return sum / valid_count;
}

/**
 * @brief  初始化时进行多次采样并计算基准角度
 * @param  None
 * @retval 基准角度值
 */
static float HWT101_InitSampling(void)
{
    float angles[INIT_SAMPLE_COUNT];
    uint8_t sample_count = 0;
    uint32_t timeout_count = 0;
    const uint32_t max_timeout = 1000; // 最大超时次数
    
    #if Debug
    printf("开始HWT101初始化采样...\r\n");
    #endif
    
    while(sample_count < INIT_SAMPLE_COUNT && timeout_count < max_timeout) {
        // 等待数据就绪信号量，超时时间设为100ms
        if(xSemaphoreTake(HWT101_DataReadySemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
            // 校验数据有效性
            if(DMA_buf[21] == (uint8_t)(0x55+0x53+DMA_buf[17]+DMA_buf[18]+DMA_buf[19]+DMA_buf[20])) {
                float raw_angle = HWT101_GetRawAngle();
                angles[sample_count] = HWT101_NormalizeAngle(raw_angle);
                sample_count++;
                
                #if Debug
                printf("采样 %d: %f 度\r\n", sample_count, angles[sample_count-1]);
                #endif
            }
            // 重新启动DMA接收
            HAL_UART_Receive_DMA(&huart2, DMA_buf, MAXSIZE);
        }
        timeout_count++;
    }
    
    if(sample_count == 0) {
        #if Debug
        printf("初始化采样失败，使用默认值0\r\n");
        #endif
        return 0.0f;
    }
    
    // 计算过滤后的平均值
    float base_angle = HWT101_CalculateFilteredAverage(angles, sample_count);
    
    #if Debug
    printf("初始化完成，基准角度: %f 度 (有效采样: %d/%d)\r\n", 
           base_angle, sample_count, INIT_SAMPLE_COUNT);
    #endif
    
    return base_angle;
}

/**
 * @brief  HWT101硬件初始化（在任务内调用）
 * @param  None
 * @retval None
 */
void HWT101_Init(void)
{
    // 初始化角度相关变量
    Angle = 0.0f;
    PrevRawAngle = 0.0f;
    TotalAngle = 0.0f;
    init_complete = 0;
    
    // 使能IDLE中断
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    
    // 先执行Z轴归零
    HAL_UART_Transmit(&huart2, CALIYAW, sizeof(CALIYAW), HAL_MAX_DELAY);
    // 发送两次以确保命令被接收
    HAL_UART_Transmit(&huart2, CALIYAW, sizeof(CALIYAW), HAL_MAX_DELAY);
    // 等待足够长的时间确保归零操作完成
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // 归零完成后，开启DMA接收
    HAL_UART_Receive_DMA(&huart2, DMA_buf, MAXSIZE);
    
    // 进行初始化采样
    float base_angle = HWT101_InitSampling();
    
    // 设置基准角度
    PrevRawAngle = base_angle;
    TotalAngle = 0.0f;  // 从0开始计算累计角度
    Angle = 0.0f;
    
    // 标记初始化完成
    init_complete = 1;
    
    #if Debug
    printf("HWT101初始化完成，基准角度: %f 度\r\n", base_angle);
    #endif
}

/**
 * @brief  HWT101数据处理
 */
void HWT101_Data_Proc(void)
{
    // 如果初始化未完成，不处理数据
    if(!init_complete) {
        HAL_UART_Receive_DMA(&huart2, DMA_buf, MAXSIZE);
        return;
    }
    
    // 校验和检查
    if(DMA_buf[21] != (uint8_t)(0x55+0x53+DMA_buf[17]+DMA_buf[18]+DMA_buf[19]+DMA_buf[20]))
    {
        #if Debug
        printf("HWT101校验错误!\r\n");
        #endif
        // 重新开启DMA接收
        HAL_UART_Receive_DMA(&huart2, DMA_buf, MAXSIZE);
        return;
    }
    
    // 计算当前原始角度
    float currentRawAngle = HWT101_GetRawAngle();
    currentRawAngle = HWT101_NormalizeAngle(currentRawAngle);
    
    // 计算角度变化值
    float deltaAngle = currentRawAngle - PrevRawAngle;
    
    // 处理角度穿越问题（例如从179度到-179度的变化）
    if (deltaAngle > 180.0f) {
        deltaAngle -= 360.0f;
    } else if (deltaAngle < -180.0f) {
        deltaAngle += 360.0f;
    }
    
    // 更新累计角度
    TotalAngle += deltaAngle;
    
    // 更新Angle和PrevRawAngle变量
    Angle = TotalAngle;
    PrevRawAngle = currentRawAngle;
    
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
    // 先创建信号量
    HWT101_DataReadySemaphore = xSemaphoreCreateBinary();
    
    // 确保信号量创建成功
    configASSERT(HWT101_DataReadySemaphore != NULL);
    
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
 * @note   此函数需要在UART2的IDLE中断处理函数中调用
 * @param  None
 * @retval None
 */
void HWT101_UART_IDLECallback(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // 确保信号量已创建
    if(HWT101_DataReadySemaphore != NULL)
    {
        // 给出信号量，通知任务处理数据
        xSemaphoreGiveFromISR(HWT101_DataReadySemaphore, &xHigherPriorityTaskWoken);
        
        // 如果释放信号量导致更高优先级的任务就绪，则请求上下文切换
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

