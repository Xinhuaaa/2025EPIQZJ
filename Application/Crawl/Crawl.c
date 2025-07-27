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
#include "bsp_log.h"

#define CRAWL_SPEED 800 // 默认超时时间5秒
#define CRAWL_ACC 250 // 默认超时时间5秒
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

    // crawlTaskHandle = osThreadNew(CrawlTask, NULL, &crawlTask_attributes);

    
    return 0;
}

// /**
//  * @brief 抓取控制任务函数
//  * @param argument 任务参数
//  */
void CrawlTask(void *argument)
{
    while (1)
    {
        Emm_V5_Pos_Control(1,1,600,0,num,1,0);
        osDelay(10);
        Emm_V5_Pos_Control(2,0,600,0,num,1,0);
        osDelay(2000);

        osDelay(100);  // 适当的延时
    }
}
//     runActionGroup(2,1);
//     osDelay(500);
//     runActionGroup(3,1);
//     osDelay(4500);
//     Lift_To_StartHeight();

//     while (1)
//     {
//         // 根据当前状态执行相应动作
//         switch (crawl_status.state)
//         {

//             case CRAWL_STATE_IDLE:

//         runActionGroup(5,1);//前面舵机放置
//         osDelay(550);

//         runActionGroup(6,1);//后面舵机放置
//         osDelay(550);

            
//         break;
                
//             case CRAWL_1:
//         runActionGroup(3,1);
//         //移动伸缩到抓取长度
//         Emm_V5_Pos_Control(1,0,600,255,17400,1,0); 
//         osDelay(50);
//         Emm_V5_Pos_Control(2,1,600,255,17400,1,0);
//         osDelay(1500);
//         //升高到抓取一层抓取高度
//         Lift_To_HighA();   
//         osDelay(500);
//         //舵机抓取
//         runActionGroup(4,1); //舵机抓取
//         osDelay(1000);
//         //升高到一层运动高度
//         Lift_To_High1(); //升高到1层高度
//         osDelay(50);
//         //移动伸缩到车内一号位长度
//         Emm_V5_Pos_Control(1,1,600,255,55500,1,0); 
//         osDelay(50);
//         Emm_V5_Pos_Control(2,0,600,255,55500,1,0);
//         osDelay(5550);
//         //舵机放置
//         runActionGroup(3,1); //舵机放置
//         osDelay(2000);
//         //伸缩回程到初始位置
//         Emm_V5_Pos_Control(1,0,600,255,0,1,0); 
//         osDelay(50);
//         Emm_V5_Pos_Control(2,1,600,255,0,1,0);
//         osDelay(5550);
//         crawl_status.state=CRAWL_STATE_IDLE; //恢复默认状态
//         break;
                
//             case CRAWL_2:
//         runActionGroup(3,1);
//         //移动伸缩到抓取长度
//         Emm_V5_Pos_Control(1,0,2000,255,17400,1,0); 
//         osDelay(50);
//         Emm_V5_Pos_Control(2,1,2000,255,17400,1,0);
//         osDelay(1500);
//         //升高到抓取一层高度
//         Lift_To_HighA();     
//         osDelay(50);
//         //舵机抓取
//         runActionGroup(4,1); 
//         osDelay(1000);
//         //升高到一层运动高度
//         Lift_To_High1(); 
//         osDelay(50);
//         //移动伸缩到车内二号位长度
//         Emm_V5_Pos_Control(1,1,600,255,37000,1,0); 
//         osDelay(50);
//         Emm_V5_Pos_Control(2,0,600,255,37000,1,0);
//         osDelay(5550);
//         //舵机放置
//         runActionGroup(3,1); //舵机放置
//         osDelay(2000);
//         //伸缩回程到初始位置
//         Emm_V5_Pos_Control(1,0,1000,255,0,1,0); 
//         osDelay(50);
//         Emm_V5_Pos_Control(2,1,1000,255,0,1,0);
//         osDelay(5550);
//         crawl_status.state=CRAWL_STATE_IDLE; 
//         break;
                
//             case CRAWL_3:
//         runActionGroup(3,1);
//         //移动伸缩到抓取长度
//         Emm_V5_Pos_Control(1,0,2000,255,17400,1,0); 
//         osDelay(50);
//         Emm_V5_Pos_Control(2,1,2000,255,17400,1,0);
//         osDelay(1500);
//         //升高到抓取一层高度
//         Lift_To_HighA();     
//         osDelay(50);
//         //舵机抓取
//         runActionGroup(4,1); 
//         osDelay(1000);
//         //升高到一层运动高度
//         Lift_To_High1(); 
//         osDelay(50);
//         //移动伸缩到车内三号位长度
//         Emm_V5_Pos_Control(1,1,600,255,18000,1,0); 
//         osDelay(50);
//         Emm_V5_Pos_Control(2,0,600,255,18000,1,0);
//         osDelay(5550);
//         //舵机放置
//         runActionGroup(3,1); 
//         osDelay(2000);
//         //伸缩回程到初始位置
//         Emm_V5_Pos_Control(1,0,1000,255,0,1,0); 
//         osDelay(50);
//         Emm_V5_Pos_Control(2,1,1000,255,0,1,0);
//         osDelay(5550);
//         crawl_status.state=CRAWL_STATE_IDLE; //恢复默认状态
//         break;
                
//             case CRAWL_4:
//         runActionGroup(3,1);
//         //升高到抓取二层高度
//         Lift_To_HighB(); 
//         //移动伸缩到抓取长度
//         Emm_V5_Pos_Control(1,0,2000,255,17400,1,0); 
//         osDelay(50);
//         Emm_V5_Pos_Control(2,1,2000,255,17400,1,0);
//         osDelay(1500);
//         //舵机抓取
//         runActionGroup(4,1); 
//         osDelay(1000);
//         osDelay(50);
//         //移动伸缩到车内一号位长度
//         Emm_V5_Pos_Control(1,1,600,255,55500,1,0); 
//         osDelay(50);
//         Emm_V5_Pos_Control(2,0,600,255,55500,1,0);
//         osDelay(5550);
//         //降低到2层放置高度
//         Lift_To_High2(); 
//         //舵机放置
//         runActionGroup(3,1); 
//         osDelay(2000);
//         //伸缩回程到初始位置
//         Emm_V5_Pos_Control(1,0,1000,255,0,1,0); 
//         osDelay(50);
//         Emm_V5_Pos_Control(2,1,1000,255,0,1,0);
//         osDelay(5550);
//         crawl_status.state=CRAWL_STATE_IDLE; //恢复默认状态
//         break;
                
//             case CRAWL_5:
//         runActionGroup(3,1);
//         //升高到抓取二层高度
//         Lift_To_HighB(); 
//         //移动伸缩到抓取长度
//         Emm_V5_Pos_Control(1,0,2000,255,17400,1,0); 
//         osDelay(50);
//         Emm_V5_Pos_Control(2,1,2000,255,17400,1,0);
//         osDelay(1500);
//         //舵机抓取
//         runActionGroup(4,1); //舵机抓取
//         osDelay(1000);
//         //伸缩移动到车内二号位放置长度
//         Emm_V5_Pos_Control(1,1,600,255,37000,1,0); 
//         osDelay(50);
//         Emm_V5_Pos_Control(2,0,600,255,37000,1,0);
//         osDelay(5550);
//         //降低到2层放置高度
//         Lift_To_High2(); 
//         //舵机放置
//         runActionGroup(3,1); //舵机放置
//         osDelay(2000);
//         //伸缩回程到初始位置
//         Emm_V5_Pos_Control(1,0,1000,255,0,1,0); 
//         osDelay(50);
//         Emm_V5_Pos_Control(2,1,1000,255,0,1,0);
//         osDelay(5550);
//         crawl_status.state=CRAWL_STATE_IDLE; 
//         break;
                
//             case CRAWL_6:
//         runActionGroup(3,1);
//         //升高到抓取二层高度
//         Lift_To_HighB(); 
//         //移动伸缩到抓取长度
//         Emm_V5_Pos_Control(1,0,2000,255,17400,1,0); 
//         osDelay(50);
//         Emm_V5_Pos_Control(2,1,2000,255,17400,1,0);
//         osDelay(1500);
//         //舵机抓取
//         runActionGroup(4,1); //舵机抓取
//         osDelay(1000);
//         osDelay(50);
//         //移动伸缩到车内三号位长度
//         Emm_V5_Pos_Control(1,1,600,255,18000,1,0); 
//         osDelay(50);
//         Emm_V5_Pos_Control(2,0,600,255,18000,1,0);
//         osDelay(5550);
//         //降低到2层放置高度
//         Lift_To_High2(); //升高到2层高度
//         //舵机放置
//         runActionGroup(3,1); //舵机放置
//         osDelay(2000);
//         //伸缩回程到初始位置
//         Emm_V5_Pos_Control(1,0,1000,255,0,1,0); 
//         osDelay(50);
//         Emm_V5_Pos_Control(2,1,1000,255,0,1,0);
//         osDelay(5550);
//         crawl_status.state=CRAWL_STATE_IDLE; 
//         break;
        
//         case Test:
//         //测试舵机抓取动作
//         runActionGroup(4,1); //舵机抓取
//         osDelay(1000);    
//         //测试舵机放置动作
//         runActionGroup(1,1); //舵机抓取
//         osDelay(1000);    
//         break;
        
//         case PUT_1:
//         //准备放置动作
//         runActionGroup(3,1);
//         osDelay(500);
//         runActionGroup(2,1);
//         //移动伸缩到车内一号位长度
//         Emm_V5_Pos_Control(1,1,600,255,18000,1,0);
//         osDelay(10);
//         Emm_V5_Pos_Control(2,0,600,255,18000,1,0);
//         osDelay(2000);
//         //升高到二层抓取高度
//         Lift_To_PUT2HIGH(); 
//         //舵机抓取
//         runActionGroup(1,1); 
//         osDelay(800);
//         //升高到二层运动高度
//         Lift_To_High2();
//         //移动伸缩到货架放置长度
//         Emm_V5_Pos_Control(1,1,600,255,45500,1,0);
//         osDelay(10);
//         Emm_V5_Pos_Control(2,0,600,255,45000,1,0);
//         osDelay(2000);
//         //降低到货架放置高度
//         Lift_To_PUTDown(); 
//         //舵机放置
//         runActionGroup(3,1);
//         osDelay(500);
//         runActionGroup(2,1);
//         osDelay(2000);
//         //升高到运动高度
//         Lift_To_PUT2HIGH(); 
//         //伸缩回程到初始位置
//         Emm_V5_Pos_Control(1,0,600,255,0,1,0); 
//         osDelay(10);
//         Emm_V5_Pos_Control(2,1,600,255,0,1,0);
//         osDelay(2000);
//         Lift_To_PUT2HIGH();
//         crawl_status.state=CRAWL_STATE_IDLE; //恢复默认状态
//         break;
        
//         case PUT_2:
//         //准备放置动作
//         runActionGroup(3,1);
//         osDelay(500);
//         runActionGroup(2,1);
//         //移动伸缩到车内二号位长度
//         Emm_V5_Pos_Control(1,1,600,255,0,1,0);
//         osDelay(10);
//         Emm_V5_Pos_Control(2,0,600,255,0,1,0);
//         osDelay(2000);
//         //升高到二层抓取高度
//         Lift_To_PUT2HIGH(); 
//         //舵机抓取
//         runActionGroup(1,1); 
//         osDelay(800);
//         //升高到二层运动高度
//         Lift_To_High2(); 
//         //移动伸缩到货架放置长度
//         Emm_V5_Pos_Control(1,1,600,255,45500,1,0);
//         osDelay(10);
//         Emm_V5_Pos_Control(2,0,600,255,45000,1,0);
//         osDelay(2000);
//         //降低到货架放置高度
//         Lift_To_PUTDown(); 
//         //舵机放置
//         runActionGroup(3,1);
//         osDelay(500);
//         runActionGroup(2,1);
//         osDelay(2000);
//         //升高到运动高度
//         Lift_To_PUT2HIGH(); 
//         //伸缩回程到初始位置
//         Emm_V5_Pos_Control(1,0,600,255,0,1,0);
//         osDelay(10);
//         Emm_V5_Pos_Control(2,1,600,255,0,1,0);
//         osDelay(2000);
//         Lift_To_PUT2HIGH();
//         crawl_status.state=CRAWL_STATE_IDLE;
//         break;
        
//         case PUT_3:
//         //准备放置动作
//         runActionGroup(3,1);
//         osDelay(500);
//         runActionGroup(2,1);
//         //移动伸缩到车内三号位长度
//         Emm_V5_Pos_Control(1,0,600,255,18000,1,0);
//         osDelay(10);
//         Emm_V5_Pos_Control(2,1,600,255,18000,1,0);
//         osDelay(2000);
//         //升高到二层抓取高度
//         Lift_To_PUT2HIGH(); 
//         //舵机抓取
//         runActionGroup(1,1); 
//         osDelay(800);
//         //升高到二层运动高度
//         Lift_To_High2(); 
//         //移动伸缩到货架放置长度
//         Emm_V5_Pos_Control(1,1,600,255,45500,1,0);
//         osDelay(10);
//         Emm_V5_Pos_Control(2,0,600,255,45000,1,0);
//         osDelay(2000);
//         //降低到货架放置高度
//         Lift_To_PUTDown(); 
//         //舵机放置
//         runActionGroup(3,1);
//         osDelay(500);
//         runActionGroup(2,1);
//         osDelay(2000);
//         //升高到运动高度
//         Lift_To_PUT2HIGH(); 
//         //伸缩回程到初始位置
//         Emm_V5_Pos_Control(1,0,600,255,0,1,0); 
//         osDelay(10);
//         Emm_V5_Pos_Control(2,1,600,255,0,1,0);
//         osDelay(2000);
//         Lift_To_PUT2HIGH();
//         crawl_status.state=CRAWL_STATE_IDLE; //恢复默认状态
//         break;
        
//         case PUT_4:
//         //准备放置动作
//         runActionGroup(3,1);
//         osDelay(500);
//         runActionGroup(2,1);
//         //移动伸缩到车内一号位长度
//         Emm_V5_Pos_Control(1,1,600,255,18000,1,0);
//         osDelay(10);
//         Emm_V5_Pos_Control(2,0,600,255,18000,1,0);
//         osDelay(2000);
//         //升高到一层抓取高度
//         Lift_To_StartHeight(); 
//         //舵机抓取
//         runActionGroup(1,1); 
//         osDelay(800);
//         //升高到一层运动高度
//         Lift_To_High1(); 
//         //移动伸缩到货架放置长度
//         Emm_V5_Pos_Control(1,1,600,255,45500,1,0);
//         osDelay(10);
//         Emm_V5_Pos_Control(2,0,600,255,45000,1,0);
//         osDelay(2000);
//         //降低到货架放置高度
//         Lift_To_PUTDown(); 
//         //舵机放置
//         runActionGroup(3,1);
//         osDelay(500);
//         runActionGroup(2,1);
//         osDelay(2000);
//         //升高到运动高度
//         Lift_To_StartHeight(); 
//         //伸缩回程到初始位置
//         Emm_V5_Pos_Control(1,0,600,255,0,1,0); 
//         osDelay(10);
//         Emm_V5_Pos_Control(2,1,600,255,0,1,0);
//         osDelay(2000);
//         crawl_status.state=CRAWL_STATE_IDLE; 
//         break;
        
//         case PUT_5:
//         //准备放置动作
//         runActionGroup(3,1);
//         osDelay(500);
//         runActionGroup(2,1);
//         //移动伸缩到车内二号位长度
//         Emm_V5_Pos_Control(1,1,600,255,0,1,0);
//         osDelay(10);
//         Emm_V5_Pos_Control(2,0,600,255,0,1,0);
//         osDelay(2000);
//         //升高到一层抓取高度
//         Lift_To_StartHeight(); 
//         //舵机抓取
//         runActionGroup(1,1); 
//         osDelay(800);
//         //升高到一层运动高度
//         Lift_To_High1(); 
//         //移动伸缩到货架放置长度
//         Emm_V5_Pos_Control(1,1,600,255,45500,1,0);
//         osDelay(10);
//         Emm_V5_Pos_Control(2,0,600,255,45000,1,0);
//         osDelay(2000);
//         //降低到货架放置高度
//         Lift_To_PUTDown(); 
//         //舵机放置
//         runActionGroup(3,1);
//         osDelay(500);
//         runActionGroup(2,1);
//         osDelay(2000);
//         //升高到运动高度
//         Lift_To_StartHeight(); 
//         //伸缩回程到初始位置
//         Emm_V5_Pos_Control(1,0,600,255,0,1,0); 
//         osDelay(10);
//         Emm_V5_Pos_Control(2,1,600,255,0,1,0);
//         osDelay(2000);
//         crawl_status.state=CRAWL_STATE_IDLE; 
//         break;
        
//         case PUT_6:
//         //准备放置动作
//         runActionGroup(3,1);
//         osDelay(500);
//         runActionGroup(2,1);
//         //移动伸缩到车内三号位长度
//         Emm_V5_Pos_Control(1,0,600,255,18000,1,0);
//         osDelay(10);
//         Emm_V5_Pos_Control(2,1,600,255,18000,1,0);
//         osDelay(2000);
//         //升高到一层抓取高度
//         Lift_To_StartHeight(); 
//         //舵机抓取
//         runActionGroup(1,1); 
//         osDelay(800);
//         //升高到一层运动高度
//         Lift_To_High1(); 
//         //移动伸缩到货架放置长度
//         Emm_V5_Pos_Control(1,1,600,255,45500,1,0);
//         osDelay(10);
//         Emm_V5_Pos_Control(2,0,600,255,45000,1,0);
//         osDelay(2000);
//         //降低到货架放置高度
//         Lift_To_PUTDown(); 
//         //舵机放置
//         runActionGroup(3,1);
//         osDelay(500);
//         runActionGroup(2,1);
//         osDelay(2000);
//         //升高到运动高度
//         Lift_To_StartHeight(); 
//         //伸缩回程到初始位置
//         Emm_V5_Pos_Control(1,0,600,255,0,1,0); 
//         osDelay(10);
//         Emm_V5_Pos_Control(2,1,600,255,0,1,0);
//         osDelay(2000);
//         crawl_status.state=CRAWL_STATE_IDLE; 
//         break;
//         }
        
//         // 周期性延时
//         osDelay(pdMS_TO_TICKS(CRAWL_TASK_PERIOD));
//     }
// }

/**
 * @brief 抓取箱子接口函数
 * @param box_number 箱子编号 (1-6)
 *                   1-3: 上层箱子
 *                   4-6: 下层箱子
 *                   排列方式：
 *                   1 2 3
 *                   4 5 6
 * @param box_count 当前已放置的箱子数量 (0-5)
 * @retval 0: 成功, -1: 失败
 */
int Crawl_GrabBox(int box_number, int box_count)
{
    // 参数检查
    if (box_number < 1 || box_number > 6) {
        printf("无效的箱子编号: %d\r\n", box_number);
        return -1;
    }
    
    if (box_count < 0 || box_count > 5) {
        printf("无效的箱子数量: %d\r\n", box_count);
        return -1;
    }
    
    // 检查系统是否忙碌
    if (crawl_status.is_busy) {
        printf("抓取系统忙碌中\r\n");
        return -1;
    }
    // 设置忙碌状态
    crawl_status.is_busy = true;
    
    // 准备抓取动作
    runActionGroup(3, 1);
    // 根据箱子编号判断层数并执行相应动作
    if (box_number >= 1 && box_number <= 3) {
        // 上层箱子 (1-3)
        // 升高到第二层抓取高度
        Lift_To_HighB();
        Emm_V5_Pos_Control(1, 0,CRAWL_SPEED,CRAWL_ACC, 17400, 1, 0);
        osDelay(50);
        Emm_V5_Pos_Control(2, 1,CRAWL_SPEED,CRAWL_ACC, 17400, 1, 0);
        osDelay(1500); //最优
        // 舵机抓取
        runActionGroup(4, 1);
        Lift_To_HighBMove();
        osDelay(1000);
        
    } else if (box_number >= 4 && box_number <= 6) {
        // 下层箱子 (4-6)
        // 移动伸缩到抓取长度
        Lift_To_StartHeight();
        Emm_V5_Pos_Control(1, 0,CRAWL_SPEED,CRAWL_ACC, 17400, 1, 0);
        osDelay(50);
        Emm_V5_Pos_Control(2, 1,CRAWL_SPEED,CRAWL_ACC, 17400, 1, 0);
        // 升高到第一层抓取高度
        osDelay(1800);
        Lift_To_HighA();
        runActionGroup(4, 1);
        osDelay(100);

        // 升高到第一层运动高度
        Lift_To_High1();
        osDelay(50);
    }
    
    // 根据已放置的箱子数量确定车内位置
    uint32_t target_position;
    
    switch (box_count) {
        case 0:
            // 第一个箱子，放到一号位下层
            target_position = 55500;
            Emm_V5_Pos_Control(1, 1,CRAWL_SPEED,CRAWL_ACC, target_position, 1, 0);
            osDelay(50);
            Emm_V5_Pos_Control(2, 0,CRAWL_SPEED,CRAWL_ACC, target_position, 1, 0);
            osDelay(2550);
            Lift_To_High1();

            break;
        case 1:
            // 第二个箱子，放到二号位下层
            target_position = 37000;
            Emm_V5_Pos_Control(1, 1,CRAWL_SPEED,CRAWL_ACC, target_position, 1, 0);
            osDelay(50);
            Emm_V5_Pos_Control(2, 0,CRAWL_SPEED,CRAWL_ACC, target_position, 1, 0);
            osDelay(2050);
            Lift_To_High1();
            break;
        case 2:
            // 第三个箱子，放到三号位下层
            target_position = 18000;
            Emm_V5_Pos_Control(1, 1,CRAWL_SPEED,CRAWL_ACC, target_position, 1, 0);
            osDelay(50);
            Emm_V5_Pos_Control(2, 0,CRAWL_SPEED,CRAWL_ACC, target_position, 1, 0);
            osDelay(1550);
            Lift_To_High1();
            break;
        case 3:
            // 第四个箱子，放到一号位上层
            Emm_V5_Pos_Control(1, 1,CRAWL_SPEED,CRAWL_ACC, 0, 1, 0);
            osDelay(50);
            Emm_V5_Pos_Control(2, 0,CRAWL_SPEED,CRAWL_ACC, 0, 1, 0);
            osDelay(550);
            Lift_To_High2();
            target_position = 55500;
            Emm_V5_Pos_Control(1, 1,CRAWL_SPEED,CRAWL_ACC, target_position, 1, 0);
            osDelay(50);
            Emm_V5_Pos_Control(2, 0,CRAWL_SPEED,CRAWL_ACC, target_position, 1, 0);
            osDelay(2550);
            break;
        case 4:
            // 第五个箱子，放到二号位上层
            Emm_V5_Pos_Control(1, 1,CRAWL_SPEED,CRAWL_ACC, 0, 1, 0);
            osDelay(50);
            Emm_V5_Pos_Control(2, 0,CRAWL_SPEED,CRAWL_ACC, 0, 1, 0);
            osDelay(550);
            Lift_To_High2();
            target_position = 37000;
            Emm_V5_Pos_Control(1, 1,CRAWL_SPEED,CRAWL_ACC, target_position, 1, 0);
            osDelay(50);
            Emm_V5_Pos_Control(2, 0,CRAWL_SPEED,CRAWL_ACC, target_position, 1, 0);
            osDelay(2050);
            break;
        case 5:
            // 第六个箱子，放到三号位上层
            Emm_V5_Pos_Control(1, 1,CRAWL_SPEED,CRAWL_ACC, 0, 1, 0);
            osDelay(50);
            Emm_V5_Pos_Control(2, 0,CRAWL_SPEED,CRAWL_ACC, 0, 1, 0);
            osDelay(550);
            Lift_To_High2();
            target_position = 18000;
            Emm_V5_Pos_Control(1, 1,CRAWL_SPEED,CRAWL_ACC, target_position, 1, 0);
            osDelay(50);
            Emm_V5_Pos_Control(2, 0,CRAWL_SPEED,CRAWL_ACC, target_position, 1, 0);
            osDelay(1550);
            break;
        default:
            crawl_status.is_busy = false;
            return -1;
    }

    // 舵机归位
    runActionGroup(3, 1);    
    osDelay(500);

    // 清除忙碌状态
    crawl_status.is_busy = false;
    
    return 0;
}

/**
 * @brief 放置箱子接口函数
 * @param box_count 当前已放置的箱子数量 (0-5)
 * @param special_box 特殊箱子编号 (0表示无特殊箱子)
 * @param is_reverse_place 是否为反向放置 (0:正向放置, 1:反向放置)
 * @retval 0: 成功, -1: 失败
 */
int Crawl_PlaceBox(int box_count, int special_box, int is_reverse_place)
{
    // 参数检查
    if (box_count < 0 || box_count > 5) {
        printf("无效的箱子数量: %d\r\n", box_count);
        return -1;
    }
    
    // 检查系统是否忙碌
    if (crawl_status.is_busy) {
        return -1;
    }
    
    // 设置忙碌状态
    crawl_status.is_busy = true;
    
    // 准备放置动作 - 先将爪子搞到放松状态
    runActionGroup(3, 1);
    runActionGroup(2, 1);
    
    // 根据box_count确定车内位置和移动参数
    uint32_t grab_position;
    uint8_t direction1, direction2;
    
    // 根据box_count确定车内位置（0-2从上层抓取，3-5从下层抓取）
//从车内抓取箱子：
    if (is_reverse_place) {
         switch (box_count) {
        case 0:
        case 3:
            // 一号位
            grab_position = 56000;
            direction1 = 1;
            direction2 = 0;
        Emm_V5_Pos_Control(1, direction1,CRAWL_SPEED,CRAWL_ACC, grab_position, 1, 0);
        osDelay(10);
        Emm_V5_Pos_Control(2, direction2,CRAWL_SPEED,CRAWL_ACC, grab_position, 1, 0);
        osDelay(1000);
            break;
        case 1:
        case 4:
            // 二号位
            grab_position = 37000;
            direction1 = 1;
            direction2 = 0;
        Emm_V5_Pos_Control(1, direction1,CRAWL_SPEED,CRAWL_ACC, grab_position, 1, 0);
        osDelay(10);
        Emm_V5_Pos_Control(2, direction2,CRAWL_SPEED,CRAWL_ACC, grab_position, 1, 0);
        osDelay(1000);
            break;
        case 2:
        case 5:
            // 三号位
            grab_position = 18000;
            direction1 = 1;
            direction2 = 0;
        Emm_V5_Pos_Control(1, direction1,CRAWL_SPEED,CRAWL_ACC, grab_position, 1, 0);
        osDelay(10);
        Emm_V5_Pos_Control(2, direction2,CRAWL_SPEED,CRAWL_ACC, grab_position, 1, 0);
        osDelay(1000);
            break;
        default:
            grab_position = 0;
            direction1 = 1;
            direction2 = 0;
        Emm_V5_Pos_Control(1, direction1,CRAWL_SPEED,CRAWL_ACC, grab_position, 1, 0);
        osDelay(10);
        Emm_V5_Pos_Control(2, direction2,CRAWL_SPEED,CRAWL_ACC, grab_position, 1, 0);
        osDelay(1000);
            break;
    }

        // 根据box_count确定抓取高度
        if (box_count >= 0 && box_count <= 2) {
            // 抓取上层箱子
            Lift_To_PUT2HIGH();
        } else {
            // 抓取下层箱子
            Lift_To_StartHeight();
        }
        
        // 使用后方舵机组抓取箱子
        runActionGroup(4, 1); // 使用反向舵机组抓取
        
        // 升高到运动高度
        if (box_count >= 0 && box_count <= 2) {
            Lift_To_High2();
        } else {
            Lift_To_High1();
        }
    } else {
        // 常规放置逻辑
        // 移动伸缩到车内对应位置
         switch (box_count) {
        case 0:
        case 3:
            // 一号位
            grab_position = 18000;
            direction1 = 1;
            direction2 = 0;
            break;
        case 1:
        case 4:
            // 二号位
            grab_position = 0;
            direction1 = 1;
            direction2 = 0;
            break;
        case 2:
        case 5:
            // 三号位
            grab_position = 18000;
            direction1 = 0;
            direction2 = 1;
            break;
        default:
            grab_position = 0;
            direction1 = 1;
            direction2 = 0;
            break;
    }
        Emm_V5_Pos_Control(1, direction1,CRAWL_SPEED,CRAWL_ACC, grab_position, 1, 0);
        osDelay(10);
        Emm_V5_Pos_Control(2, direction2,CRAWL_SPEED,CRAWL_ACC, grab_position, 1, 0);
        osDelay(1000);
        
        // 根据box_count确定抓取高度
        if (box_count >= 0 && box_count <= 2) {
            // 抓取上层箱子
            Lift_To_PUT2HIGH();
        } else {
            // 抓取下层箱子
            Lift_To_StartHeight();
        }
        
        // 舵机抓取箱子
        runActionGroup(1, 1); // 使用常规舵机组抓取
        
        // 升高到运动高度
        if (box_count >= 0 && box_count <= 2) {
            Lift_To_High2();
        } else {
            Lift_To_High1();
        }
    }
    //抓取完后放置
    if (is_reverse_place) {
        Lift_To_High2();
        // 反向放置逻辑
        if (special_box == box_count) {
        // 特殊箱子使用叠放高度
        Lift_To_PUTspecialUP();
        Emm_V5_Pos_Control(1, 0,CRAWL_SPEED,CRAWL_ACC, 17400, 1, 0);
        osDelay(10);
        Emm_V5_Pos_Control(2, 1,CRAWL_SPEED,CRAWL_ACC, 17400, 1, 0);
        osDelay(1000);
        Lift_To_PUTspecialDown();
        runActionGroup(3, 1); 
        runActionGroup(6, 1);
        osDelay(1000);
        Lift_To_PUTspecialUUP();

        } else {
            // 普通箱子使用普通放置高度
        Emm_V5_Pos_Control(1, 0,CRAWL_SPEED,CRAWL_ACC, 17400, 1, 0);
        osDelay(10);
        Emm_V5_Pos_Control(2, 1,CRAWL_SPEED,CRAWL_ACC, 17400, 1, 0);
        osDelay(1000);
        Lift_To_PUTDown();
         runActionGroup(3, 1); // 使用反向舵机组放置
        runActionGroup(6, 1);
        osDelay(1000);
        Lift_To_PUTspecialUUP();
        }
    } else {
        // 正常放置逻辑
        if (special_box == box_count) {
            // 特殊箱子使用叠放高度
        Lift_To_PUTspecialUP();
        Emm_V5_Pos_Control(1, 1,CRAWL_SPEED,CRAWL_ACC, 49100, 1, 0);
        osDelay(10);
        Emm_V5_Pos_Control(2, 0,CRAWL_SPEED,CRAWL_ACC, 49100, 1, 0);
        osDelay(1000);
        Lift_To_PUTspecialDown();
        runActionGroup(3, 1);
        runActionGroup(2, 1);
        osDelay(1000);
        Lift_To_PUTspecialUUP();


        } else {
            // 普通箱子使用普通放置高度
        Emm_V5_Pos_Control(1, 1,CRAWL_SPEED,CRAWL_ACC, 49100, 1, 0);
        osDelay(10);
        Emm_V5_Pos_Control(2, 0,CRAWL_SPEED,CRAWL_ACC, 49100, 1, 0);
        osDelay(1000);
        Lift_To_PUTDown();
        runActionGroup(3, 1);
        runActionGroup(2, 1);
        osDelay(1000);
        }
        
        // 舵机放置（松开爪子）

    }
    
    // 升高到运动高度
    if (box_count >= 0 && box_count <= 2) {
        Lift_To_PUT2HIGH();
    } else {
        Lift_To_StartHeight();
    }
    
    // 伸缩回程到初始位置
    Emm_V5_Pos_Control(1, 0,CRAWL_SPEED,CRAWL_ACC, 0, 1, 0);
    osDelay(10);
    Emm_V5_Pos_Control(2, 1,CRAWL_SPEED,CRAWL_ACC, 0, 1, 0);
    osDelay(1000);
    
    // 清除忙碌状态
    crawl_status.is_busy = false;
    return 0;
}
//电机运动到1号位的延时
void osDelay_1()
{
    osDelay(2000);
}
//电机运动到2号位的延时
void osDelay_2()
{
    osDelay(2000);
}
//电机运动到3号位的延时
void osDelay_3()
{
    osDelay(2000);
}