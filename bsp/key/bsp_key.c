#include "cmsis_os2.h"
#include "main.h"
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

// 按键状态结构体
typedef struct {
    bool current_state;
    bool last_state;
    bool is_pressed;
    uint32_t press_time;
    uint32_t release_time;
    uint8_t click_count;
    bool long_press_sent;
} key_status_t;

// 宏定义
#define KEY_QUEUE_SIZE         10
#define KEY_LONG_PRESS_TIME    1000   // ms
#define KEY_DOUBLE_CLICK_TIME  300    // ms
#define KEY_SCAN_PERIOD        10     // ms
#define KEY_DEBOUNCE_TIME      20     // ms

// CMSIS RTOS 对象
osMessageQueueId_t key_event_queue = NULL;
osMutexId_t key_mutex = NULL;
osTimerId_t key_scan_timer = NULL;
key_status_t key_status = {0};

// 读取按键状态（已设置为上拉输入）
bool read_key_pin(void) {
    return HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);  // 释放=高，按下=低
}

// 定时器回调（CMSIS 风格带 void *参数）
void key_scan_timer_callback(void *argument) {
    key_event_t event;
    static key_status_t *key = &key_status;

    if (osMutexAcquire(key_mutex, 0) == osOK) {
        bool raw_state = read_key_pin();
        uint32_t now = osKernelGetTickCount();

        if (raw_state != key->last_state) {
            osDelay(KEY_DEBOUNCE_TIME);
            raw_state = read_key_pin();  // 再次读取确认
        }

        key->current_state = raw_state;

        if (key->current_state != key->last_state) {
            key->last_state = key->current_state;

            if (!key->current_state) {  // 按下
                key->is_pressed = true;
                key->press_time = now;
                key->click_count++;

                event.event = KEY_EVENT_PRESSED;
                event.key_id = 1;
                event.timestamp = now;
                osMessageQueuePut(key_event_queue, &event, 0, 0);
            } else {  // 释放
                key->is_pressed = false;
                key->release_time = now;
                key->long_press_sent = false;

                event.event = KEY_EVENT_RELEASED;
                event.key_id = 1;
                event.timestamp = now;
                osMessageQueuePut(key_event_queue, &event, 0, 0);
            }
        }

        // 长按检测
        if (key->is_pressed &&
            !key->long_press_sent &&
            (now - key->press_time) >= KEY_LONG_PRESS_TIME) {

            event.event = KEY_EVENT_LONG_PRESS;
            event.key_id = 1;
            event.timestamp = now;
            osMessageQueuePut(key_event_queue, &event, 0, 0);
            key->long_press_sent = true;
        }

        // 双击检测
        if (!key->is_pressed && key->click_count >= 2) {
            if ((now - key->release_time) >= KEY_DOUBLE_CLICK_TIME) {
                if (key->click_count == 2) {
                    event.event = KEY_EVENT_DOUBLE_CLICK;
                    event.key_id = 1;
                    event.timestamp = now;
                    osMessageQueuePut(key_event_queue, &event, 0, 0);
                }
                key->click_count = 0;
            }
        }

        osMutexRelease(key_mutex);
    }
}

// 初始化函数
void Key_Init(void) {
    key_event_queue = osMessageQueueNew(KEY_QUEUE_SIZE, sizeof(key_event_t), NULL);
    key_mutex = osMutexNew(NULL);

    const osTimerAttr_t timer_attr = {
        .name = "keyScanTimer"
    };
    key_scan_timer = osTimerNew(key_scan_timer_callback, osTimerPeriodic, NULL, &timer_attr);

    if (key_scan_timer != NULL) {
        osTimerStart(key_scan_timer, KEY_SCAN_PERIOD);
    }
}
