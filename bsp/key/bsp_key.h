#ifndef __KEY_H
#define __KEY_H

#include "cmsis_os2.h"
#include <stdint.h>
#include <stdbool.h>

// 按键事件类型
typedef enum {
    KEY_EVENT_PRESSED,
    KEY_EVENT_RELEASED,
    KEY_EVENT_LONG_PRESS,
    KEY_EVENT_DOUBLE_CLICK
} key_event_type_t;

// 按键事件结构体
typedef struct {
    key_event_type_t event;
    uint8_t key_id;
    uint32_t timestamp;
} key_event_t;

// 初始化函数
void Key_Init(void);

// 队列、信号量、定时器对象需要定义为外部引用（如果你希望其他模块也访问）
// 若只在 key.c 使用，可以省略
extern osMessageQueueId_t key_event_queue;
extern osSemaphoreId_t key_mutex;

#endif
