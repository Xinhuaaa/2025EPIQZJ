/**
 * @file Crawl.c
 * @brief 爬行抓取控制实现 动作组1抓取
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
#include "lift.h"

#define CRAWL_STEPPER_LEFT_ID 1
#define CRAWL_STEPPER_RIGHT_ID 2 

/* 全局变量定义 */
Crawl_Status_t crawl_status = {0};
osThreadId_t crawlTaskHandle = NULL;
uint64_t maichong = 10000; 
uint64_t action = 1; 
int64_t num=0;
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
    // runActionGroup(3,1);
    // osDelay(1500);
    // runActionGroup(4,1);

    // lift_status.target_displacement=110;
    // osDelay(pdMS_TO_TICKS(3000));  
    // Emm_V5_Pos_Control(1,1,600,30,50000,0,0);
    // osDelay(pdMS_TO_TICKS(100)); 
    // Emm_V5_Pos_Control(2,0,600,30,50000,0,0);
    // osDelay(pdMS_TO_TICKS(7000)); 
    // lift_status.target_displacement=30;
    // osDelay(pdMS_TO_TICKS(7000)); 
    // runActionGroup(1,1);
    // osDelay(pdMS_TO_TICKS(2000)); 
    // lift_status.target_displacement=230;
    // osDelay(pdMS_TO_TICKS(1000));  
    // Emm_V5_Pos_Control(1,0,600,30,50000,0,0);
    // osDelay(pdMS_TO_TICKS(100)); 
    // Emm_V5_Pos_Control(2,1,600,30,50000,0,0);
    // osDelay(pdMS_TO_TICKS(20000));  
    // lift_status.target_displacement=30;

    while (1)
    {
        Emm_V5_Pos_Control(1,0,600,0,num,1,0);
        osDelay(10);
        Emm_V5_Pos_Control(2,1,600,0,num,1,0);
        osDelay(100);

        // 根据当前状态执行相应动作
        switch (crawl_status.state)
        {

            case CRAWL_STATE_IDLE:
        runActionGroup(3,1);//前面舵机放置
        osDelay(550);

        runActionGroup(2,1);//后面舵机放置
        osDelay(550);

            
        break;
                
            case CRAWL_1:
        osDelay(50);
        runActionGroup(4,1); //舵机抓取
        osDelay(1000);
        Lift_To_High1(); //升高到1层高度
        osDelay(50);
        Emm_V5_Pos_Control(1,1,800,100,55500,1,0); //移动伸缩到1号货架
        osDelay(50);
        Emm_V5_Pos_Control(2,0,800,100,55500,1,0);
        osDelay(5550);
        runActionGroup(3,1); //舵机放置
        osDelay(2000);
        Emm_V5_Pos_Control(1,0,1000,100,0,1,0); //移动伸缩到初始位置
        osDelay(50);
        Emm_V5_Pos_Control(2,1,1000,100,0,1,0);
        osDelay(5550);
        crawl_status.state=CRAWL_STATE_IDLE; //恢复默认状态


            
        break;
                
            case CRAWL_2:
        osDelay(50);
        runActionGroup(4,1); //舵机抓取
        osDelay(1000);
        Lift_To_High1(); //升高到1层高度
        osDelay(50);
        Emm_V5_Pos_Control(1,1,800,100,37000,1,0); //移动伸缩到2号货架
        osDelay(50);
        Emm_V5_Pos_Control(2,0,800,100,37000,1,0);
        osDelay(5550);
        runActionGroup(3,1); //舵机放置
        osDelay(2000);
        Emm_V5_Pos_Control(1,0,1000,100,0,1,0); //移动伸缩到初始位置
        osDelay(50);
        Emm_V5_Pos_Control(2,1,1000,100,0,1,0);
        osDelay(5550);
        crawl_status.state=CRAWL_STATE_IDLE; //恢复默认状态
        break;
                
       
            case CRAWL_3:
        osDelay(50);
        runActionGroup(4,1); //舵机抓取
        osDelay(1000);
        Lift_To_High1(); //升高到1层高度
        osDelay(50);
        Emm_V5_Pos_Control(1,1,800,100,18000,1,0); //移动伸缩到3号货架
        osDelay(50);
        Emm_V5_Pos_Control(2,0,800,100,18000,1,0);
        osDelay(5550);
        runActionGroup(3,1); //舵机放置
        osDelay(2000);
        Emm_V5_Pos_Control(1,0,1000,100,0,1,0); //移动伸缩到初始位置
        osDelay(50);
        Emm_V5_Pos_Control(2,1,1000,100,0,1,0);
        osDelay(5550);
        crawl_status.state=CRAWL_STATE_IDLE; //恢复默认状态
        break;
                
            case CRAWL_4:
        osDelay(50);
        runActionGroup(4,1); //舵机抓取
        osDelay(1000);
        Lift_To_High2(); //升高到2层高度
        osDelay(50);
        Emm_V5_Pos_Control(1,1,800,100,55500,1,0); //移动伸缩到4号货架
        osDelay(50);
        Emm_V5_Pos_Control(2,0,800,100,55500,1,0);
        osDelay(5550);
        runActionGroup(3,1); //舵机放置
        osDelay(2000);
        Emm_V5_Pos_Control(1,0,1000,100,0,1,0); //移动伸缩到初始位置
        osDelay(50);
        Emm_V5_Pos_Control(2,1,1000,100,0,1,0);
        osDelay(5550);
        crawl_status.state=CRAWL_STATE_IDLE; //恢复默认状态
        break;
                
            case CRAWL_5:
        osDelay(50);
        runActionGroup(4,1); //舵机抓取
        osDelay(1000);
        Lift_To_High2(); //升高到2层高度
        osDelay(50);
        Emm_V5_Pos_Control(1,1,800,100,37000,1,0); //移动伸缩到5号货架
        osDelay(50);
        Emm_V5_Pos_Control(2,0,800,100,37000,1,0);
        osDelay(5550);
        runActionGroup(3,1); //舵机放置
        osDelay(2000);
        Emm_V5_Pos_Control(1,0,1000,100,0,1,0); //移动伸缩到初始位置
        osDelay(50);
        Emm_V5_Pos_Control(2,1,1000,100,0,1,0);
        osDelay(5550);
        crawl_status.state=CRAWL_STATE_IDLE; //恢复默认状态
        break;
                
            case CRAWL_6:
        osDelay(50);
        runActionGroup(4,1); //舵机抓取
        osDelay(1000);
        Lift_To_High2(); //升高到2层高度
        osDelay(50);
        Emm_V5_Pos_Control(1,1,800,100,18000,1,0); //移动伸缩到6号货架
        osDelay(50);
        Emm_V5_Pos_Control(2,0,800,100,18000,1,0);
        osDelay(5550);
        runActionGroup(3,1); //舵机放置
        osDelay(2000);
        Emm_V5_Pos_Control(1,0,1000,100,0,1,0); //移动伸缩到初始位置
        osDelay(50);
        Emm_V5_Pos_Control(2,1,1000,100,0,1,0);
        osDelay(5550);
        crawl_status.state=CRAWL_STATE_IDLE; //恢复默认状态
        break;
            case Test:
        Emm_V5_Pos_Control(2,0,1600,200,55000,1,0);
        osDelay(5550);
        Emm_V5_Pos_Control(2,1,1000,200,0,1,0);
        osDelay(5550);
        crawl_status.state=CRAWL_STATE_IDLE; //恢复默认状态
        break;
 
        }
        
        // 周期性延时
        osDelay(pdMS_TO_TICKS(CRAWL_TASK_PERIOD));
    }
}