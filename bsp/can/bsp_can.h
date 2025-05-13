#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include <stdint.h>
#include "cmsis_os2.h"  
#include "stdbool.h"
#define CAN_MX_REGISTER_CNT 16     // 最大支持的 CAN 设备实例数
#define MX_CAN_FILTER_CNT (2 * 14) // STM32F4 系列最多可用过滤器数量
#define DEVICE_CAN_CNT 1           // 当前设备支持的 CAN 控制器数量（F407: 2，F334: 1）

/* ------------------------- CAN Instance 结构体 ------------------------- */
#pragma pack(1)
typedef struct _CANInstance
{
    CAN_HandleTypeDef *can_handle; // CAN 句柄
    CAN_TxHeaderTypeDef txconf;    // 发送配置
    uint32_t tx_id;                // 发送 ID
    uint32_t tx_mailbox;           // 邮箱号
    uint8_t tx_buff[8];            // 发送缓存
    uint8_t rx_buff[8];            // 接收缓存
    uint32_t rx_id;                // 接收 ID
    uint8_t rx_len;                // 接收数据长度
    uint8_t use_ext_id;            // 是否使用扩展ID (0: 标准帧, 1: 扩展帧)

    void (*can_module_callback)(struct _CANInstance *); // 模块回调函数
    void *id;                 // 模块 ID 指针（可选）

    osEventFlagsId_t rx_event;    // RTOS事件组，用于多任务同步
    uint32_t rx_counter;           // 接收计数器，每次接收数据递增
} CANInstance;
#pragma pack()

// CAN接收请求结构体，用于多任务同时等待接收
typedef struct {
    uint8_t addr;               // 期望接收的地址
    uint8_t* rx_data;           // 接收数据缓冲区
    uint8_t* rx_len;            // 接收数据长度
    CANInstance* can_inst;      // 关联的CAN实例
    uint32_t last_counter;      // 上次接收计数
    bool received;              // 是否已接收到数据
} CANRxRequest_t;

/* ---------------------- 初始化结构体 ---------------------- */
typedef struct
{
    CAN_HandleTypeDef *can_handle;
    uint32_t tx_id;
    uint32_t rx_id;
    uint8_t use_ext_id;            // 是否使用扩展ID (0: 标准帧, 1: 扩展帧)
    void (*can_module_callback)(CANInstance *);
    void *id;
} CAN_Init_Config_s;

/* 全局变量声明 */
extern CANInstance *can_instance[CAN_MX_REGISTER_CNT];
extern uint8_t idx;

/* ---------------------- API 接口函数声明 ---------------------- */

/**
 * @brief 注册一个 CAN 实例（必须先调用）
 * @param config 初始化结构体
 * @return CANInstance* 指向 CAN 实例的指针
 */
CANInstance *CANRegister(CAN_Init_Config_s *config);

/**
 * @brief 设置发送数据长度
 * @param _instance CAN 实例
 * @param length    发送长度（1~8）
 */
void CANSetDLC(CANInstance *_instance, uint8_t length);

/**
 * @brief 通过 CAN 发送数据，数据应提前写入 tx_buff
 * @param _instance CAN 实例
 * @param timeout 超时时间（毫秒）
 * @return 发送是否成功
 */
uint8_t CANTransmit(CANInstance *_instance, float timeout);

/**
 * @brief 启动 CAN 接收任务（需在系统启动阶段调用一次）
 */
void StartCANRxTask(void);

/**
 * @brief 用于调试的辅助函数，打印CAN实例状态
 * @param _instance CAN 实例
 */
void CANPrintInstanceStatus(CANInstance *_instance);

/**
 * @brief 等待特定CAN实例接收到数据
 * @param _instance CAN 实例
 * @param timeout_ms 超时时间（毫秒）
 * @return bool 是否成功接收数据
 */
bool CANWaitForData(CANInstance *_instance, uint32_t timeout_ms);

#endif // BSP_CAN_H
