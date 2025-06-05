/**
 * @file Crawl.c
 * @brief 爬行抓取控制实现 动作组1抓取 动作组2闭合
 * @author Generated
 * @date 2024
 */

#include "Crawl.h"
#include "lift.h"
#include "servo.h"
#include <stdbool.h>
#include "Emm_v5.h"
#include "FreeRTOS.h"
#include "task.h"
#include "servo.h"

#define CRAWL_STEPPER_LEFT_ID 1
#define CRAWL_STEPPER_RIGHT_ID 2 

/* 全局变量定义 */
Crawl_Status_t crawl_status = {0};
osThreadId_t crawlTaskHandle = NULL;
uint64_t maichong = 2000; 
uint64_t action = 1; 
/* 前向声明 */
void CrawlTask(void *argument);

/**
 * @brief 抓取系统初始化
 * @retval 0: 成功, -1: 失败
 */
int Crawl_Init(void)
{
    // 初始化状态结构体
    crawl_status.state = CRAWL_STATE_IDLE;
    crawl_status.action = CRAWL_ACTION_NONE;
    crawl_status.is_busy = false;
    crawl_status.is_enabled = true;
    crawl_status.target_height = 0.0f;
    crawl_status.start_time = 0;
    crawl_status.timeout = CRAWL_DEFAULT_TIMEOUT;
    crawl_status.error_code = 0;
      printf("抓取系统初始化完成\r\n");
    
    // 直接创建抓取控制任务
    const osThreadAttr_t crawlTask_attributes = {
        .name = "CrawlTask",
        .stack_size = 512 * 4,
        .priority = (osPriority_t) osPriorityNormal,
    };

    crawlTaskHandle = osThreadNew(CrawlTask, NULL, &crawlTask_attributes);
    if (crawlTaskHandle == NULL)
    {
        printf("抓取任务创建失败!\r\n");
        return -1;
    }
    
    return 0;
}

/**
 * @brief 抓取控制任务函数
 * @param argument 任务参数
 */
void CrawlTask(void *argument)
{
    // runActionGroup(1,1);
    // osDelay(pdMS_TO_TICKS(2000)); // 等待1秒钟，确保动作组运行完成
    // runActionGroup(2,1);

    while (1)
    {
        // 根据当前状态执行相应动作
        switch (crawl_status.state)
        {

            case CRAWL_STATE_IDLE:

            
        break;
                
            case CRAWL_STATE_LIFTING:
            
        break;
                
            case CRAWL_STATE_EXTENDING:
            Emm_V5_Pos_Control(1,0,600,30,maichong,1,0);
            osDelay(50);
            Emm_V5_Pos_Control(2,1,600,30,maichong,1,0);
            osDelay(50);
            crawl_status.state=CRAWL_STATE_IDLE;
        break;
                
            case CRAWL_STATE_GRABBING:
            runActionGroup(1,1);
            osDelay(50);
            crawl_status.state=CRAWL_STATE_IDLE;

        break;
                
            case CRAWL_STATE_RETRACTING:

                // 收回中状态处理
        break;
                
            case CRAWL_STATE_ERROR:
                // 错误状态处理
        break;
                
            default:
                crawl_status.state = CRAWL_STATE_IDLE;
        break;
        }
        
        // 周期性延时
        osDelay(pdMS_TO_TICKS(CRAWL_TASK_PERIOD));
    }
}