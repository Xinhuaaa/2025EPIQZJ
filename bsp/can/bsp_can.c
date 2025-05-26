#include "can.h"
#include "bsp_can.h"
#include "main.h"
#include "memory.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "cmsis_os2.h"

#define CAN_RX_QUEUE_SIZE 8
#define CAN_MX_REGISTER_CNT 16
/* CAN 接收消息结构 */
typedef struct {
    CAN_HandleTypeDef *hcan;
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
} CANRxMsg_t;

CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {NULL};
uint8_t idx;
osMessageQueueId_t can_rx_queue;
osMutexId_t can_tx_mutex;

static void CANAddFilter(CANInstance *_instance)
{
    CAN_FilterTypeDef can_filter_conf;
    static uint8_t can1_filter_idx = 0, can2_filter_idx = 14;

    can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;
    
if (_instance->use_ext_id)
{
    can_filter_conf.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_conf.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_conf.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter_conf.SlaveStartFilterBank = 14;

    // 当我们只需要特定的电机地址，使用模式2 - 仅匹配高8位
    uint32_t addr = (_instance->rx_id >> 8) & 0xFF;
    can_filter_conf.FilterIdHigh = addr << (5 + 8);       // 先左移8位对齐，再移5位
    can_filter_conf.FilterIdLow  = CAN_ID_EXT;            // IDE=1(扩展帧), RTR=0
    can_filter_conf.FilterMaskIdHigh = 0xFF << (5 + 8);   // 只匹配高8位地址
    can_filter_conf.FilterMaskIdLow  = CAN_ID_EXT;        // IDE 必须为1（扩展帧）
    
    // 调试信息
    // LOGINFO("[CAN] 配置扩展帧过滤器, 仅匹配高8位地址: 0x%02X, ID=0x%08X", addr, _instance->rx_id);
    
    can_filter_conf.FilterActivation = ENABLE;
}
    else
    {
        // 标准帧过滤器配置（保持原样）
        can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;
        can_filter_conf.FilterFIFOAssignment = (_instance->tx_id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
        can_filter_conf.SlaveStartFilterBank = 14;
        can_filter_conf.FilterIdLow = _instance->rx_id << 5;
    }
    
    can_filter_conf.FilterBank = _instance->can_handle == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++);
    can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;

    HAL_CAN_ConfigFilter(_instance->can_handle, &can_filter_conf);
}

static void CANServiceInit()
{
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    can_rx_queue = osMessageQueueNew(CAN_RX_QUEUE_SIZE, sizeof(CANRxMsg_t), NULL);
    can_tx_mutex = osMutexNew(NULL);
}

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (!idx)
    {
        CANServiceInit();
        // LOGINFO("[bsp_can] CAN Service Init");
    }
    if (idx >= CAN_MX_REGISTER_CNT)
    {
        
         #ifdef CAN_DEBUG
         while (1)
            LOGERROR("[bsp_can] CAN instance exceeded MAX num");
            #endif
    }
    for (size_t i = 0; i < idx; i++)
    {
        if (can_instance[i]->rx_id == config->rx_id && can_instance[i]->can_handle == config->can_handle)
        {
            
             #ifdef CAN_DEBUG
             while (1)
                LOGERROR("[bsp_can] CAN id crash, tx [%d] or rx [%d] already registered", config->tx_id, config->rx_id);
                #endif
        }
    }    CANInstance *instance = (CANInstance *)malloc(sizeof(CANInstance));
    memset(instance, 0, sizeof(CANInstance));

    instance->txconf.StdId = config->tx_id;
    instance->txconf.ExtId = config->tx_id; // 添加扩展ID支持
    instance->txconf.IDE = config->use_ext_id ? CAN_ID_EXT : CAN_ID_STD; // 根据配置选择使用标准帧或扩展帧
    instance->txconf.RTR = CAN_RTR_DATA;
    instance->txconf.DLC = 0x08;
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id;
    instance->rx_id = config->rx_id;    
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;
    instance->use_ext_id = config->use_ext_id; // 保存扩展ID标志
    
    // 确保rx_buff和tx_buff被正确初始化（虽然memset已经将它们置零，这里为了明确）
    memset(instance->rx_buff, 0, sizeof(instance->rx_buff));
    memset(instance->tx_buff, 0, sizeof(instance->tx_buff));
    
    // 创建事件标志组
    instance->rx_event = osEventFlagsNew(NULL);
    instance->rx_counter = 0; // 初始化接收计数器
    
    // 打印实例初始化信息
    #ifdef CAN_DEBUG
    LOGINFO("[bsp_can] 注册CAN实例: rx_id=0x%08X, tx_id=0x%08X, use_ext_id=%d", 
          instance->rx_id, instance->tx_id, instance->use_ext_id);
    #endif
    CANAddFilter(instance);
    can_instance[idx++] = instance;

    return instance;
}

uint8_t CANTransmit(CANInstance *_instance, float timeout)
{
    static uint32_t busy_count;
    float dwt_start = DWT_GetTimeline_ms();
    osMutexAcquire(can_tx_mutex, osWaitForever);

    while (HAL_CAN_GetTxMailboxesFreeLevel(_instance->can_handle) == 0)
    {
        if (DWT_GetTimeline_ms() - dwt_start > timeout)
        {
            #ifdef CAN_DEBUG
            LOGWARNING("[bsp_can] CAN MAILbox full! Cnt [%d]", busy_count);
            #endif
            busy_count++;
            osMutexRelease(can_tx_mutex);
            return 0;
        }
    }

    if (HAL_CAN_AddTxMessage(_instance->can_handle, &_instance->txconf, _instance->tx_buff, &_instance->tx_mailbox))
    {
        #ifdef CAN_DEBUG
        LOGWARNING("[bsp_can] CAN bus BUS! cnt:%d", busy_count);
        #endif
        busy_count++;
        osMutexRelease(can_tx_mutex);
        return 0;
    }

    osMutexRelease(can_tx_mutex);
    return 1;
}

void CANSetDLC(CANInstance *_instance, uint8_t length)
{
    if (length > 8 || length == 0)
        while (1)
            LOGERROR("[bsp_can] CAN DLC error!");
    _instance->txconf.DLC = length;
}

static void CANFIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox)
{
    static CAN_RxHeaderTypeDef rxconf; // 同上
    uint8_t can_rx_buff[8];
    while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff); // 从FIFO中获取数据
        for (size_t i = 0; i < idx; ++i)
        {
            bool match = false;
            
            if (_hcan == can_instance[i]->can_handle)
            {
                if (rxconf.IDE == CAN_ID_EXT && can_instance[i]->use_ext_id)
                {
                    match = (rxconf.ExtId == can_instance[i]->rx_id);
                    #ifdef CAN_DEBUG
                    if (match) {
                        LOGINFO("[CAN] 匹配到扩展帧: ID=0x%08X, 实际数据:", rxconf.ExtId);
                        for (int j = 0; j < rxconf.DLC; j++) {
                            LOGINFO("  can_rx_buff[%d]=0x%02X", j, can_rx_buff[j]);
                        }
                    }
                    #endif
                }
                else if (rxconf.IDE == CAN_ID_STD && !can_instance[i]->use_ext_id) 
                {
                    // 标准帧匹配
                    match = (rxconf.StdId == can_instance[i]->rx_id);
                }
            }
              if (match)
            {
                can_instance[i]->rx_len = rxconf.DLC;                      // 保存接收到的数据长度
                memcpy(can_instance[i]->rx_buff, can_rx_buff, rxconf.DLC); // 消息拷贝到对应实例                    
                // 更新接收计数器和事件标志
                can_instance[i]->rx_counter++;
                if (can_instance[i]->rx_event) {
                    osEventFlagsSet(can_instance[i]->rx_event, 0x01);
                }
                
                if (can_instance[i]->can_module_callback != NULL) // 回调函数不为空就调用
                {
                    can_instance[i]->can_module_callback(can_instance[i]); // 调用回调函数
                }
                return;
            }
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO1);
}
