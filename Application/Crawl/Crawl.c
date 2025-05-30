/**
 * @file Crawl.c
 * @brief 爬行抓取控制实现 - 简单顺序执行版本
 * @author Generated
 * @date 2024
 */

#include "Crawl.h"
#include "lift.h"
#include "servo.h"
#include <stdbool.h>
#include "Emm_v5.h"

#define CRAWL_STEPPER_LEFT_ID 1
#define CRAWL_STEPPER_RIGHT_ID 2 

void Crawl_SimpleGrab()
{
    Emm_V5_Pos_Control(1, 1, 500, 50, 6000, 0, 0);
    osDelay(100);
    Emm_V5_Pos_Control(2, 1, 500, 50, 6000, 0, 0);

    // 简单延时等待前伸完成
    osDelay(10);
    
    // 步骤3: 舵机抓取
    runActionGroup(1, 1);
    
    // 简单延时等待抓取完成
    osDelay(1000);
    
    return 0;
}
