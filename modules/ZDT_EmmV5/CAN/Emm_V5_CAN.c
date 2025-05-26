#include "bsp_can.h"
#include "cmsis_os2.h"
#include "string.h"
#include "bsp_log.h"  
#include "Emm_V5_CAN.h"
#include "stdbool.h"
#include "bsp_dwt.h"  

#define CAN_SEND_DELAY_MS   3
#define CAN_TX_TIMEOUT_MS   10.0f
#define MOTOR_LF_ID 5  // 左前轮ID
#define MOTOR_RF_ID 6  // 右前轮ID
#define MOTOR_LB_ID 7  // 左后轮ID
#define MOTOR_RB_ID 8  // 右后轮ID
// 编码器相关常量
#define ENCODER_MAX     65536  // 编码器最大值
#define ENCODER_HALF    32768  // 编码器最大值的一半
#define CAN_CMD_LOG_LEVEL 0
// 编码器累计值计算相关变量
static int32_t last_raw_encoder[8] = {0};     // 上一次读取的原始编码器值
static int32_t accumulated_encoder[8] = {0};  // 累积编码器值
static int32_t encoder_zero_offset[8] = {0};  // 编码器零点偏移值
static bool encoder_initialized[8] = {false}; // 是否已初始化编码器值

extern CANInstance *can_instance[];
extern uint8_t idx;

/**
 * @brief 注册X42电机CAN实例
 * @param motor_addr 电机地址(1-8)
 * @param can_handle CAN句柄
 * @return CANInstance* 注册的实例指针，NULL表示失败
 */
CANInstance* Emm_V5_CAN_RegisterMotor(uint8_t motor_addr, CAN_HandleTypeDef *can_handle)
{
    if (motor_addr < 1 || motor_addr > 8) {
        LOGERROR("电机地址无效: %d (有效范围: 1-8)", motor_addr);
        return NULL;
    }
    
    uint32_t tx_id = (motor_addr << 8); // 发送ID: 高8位是电机地址
    uint32_t rx_id = (motor_addr << 8); // 接收ID: 高8位是电机地址
      // 创建CAN配置
    CAN_Init_Config_s can_config = {
        .can_handle = can_handle,
        .tx_id = tx_id,
        .rx_id = rx_id,
        .use_ext_id = 1, // 使用扩展帧
        .can_module_callback = NULL // 暂时不使用回调函数
    };
      // 注册CAN实例
    CANInstance *instance = CANRegister(&can_config);
    
    if (instance == NULL) {
#if CAN_CMD_LOG_LEVEL >= 1
        LOGERROR("注册CAN实例失败，地址: 0x%02X", motor_addr);
#endif
        return NULL;
    }
    
#if CAN_CMD_LOG_LEVEL >= 2
    LOGINFO("成功注册CAN实例，地址: 0x%02X, TX_ID: 0x%03X, RX_ID: 0x%03X", 
            motor_addr, tx_id, rx_id);
#endif
    
    return instance;
}

/**
 * @brief X42电机驱动初始化
 * @param motor_ids 电机ID数组
 * @param motor_count 电机数量
 * @return bool 是否初始化成功
 */
bool Emm_V5_CAN_Init(uint8_t *motor_ids, uint8_t motor_count)
{
    bool all_registered = true;
    
#if CAN_CMD_LOG_LEVEL >= 2
    LOGINFO("初始化X42电机驱动...");
#endif
    
    // 注册所有电机的CAN实例
    for (uint8_t i = 0; i < motor_count; i++) {
        uint8_t motor_addr = motor_ids[i];
        if (Emm_V5_CAN_RegisterMotor(motor_addr, &hcan1) == NULL) {
            all_registered = false;
        }
    }
    
    if (!all_registered) {
#if CAN_CMD_LOG_LEVEL >= 1
        LOGERROR("部分电机CAN实例注册失败");
#endif
    } else {
#if CAN_CMD_LOG_LEVEL >= 2
        LOGINFO("所有电机CAN实例注册成功");
#endif
    }
        return all_registered;
}

/**
 * @brief  通用发送 CAN 命令（根据cmd[0]地址匹配CANInstance）
 * @param  cmd     命令数据，cmd[0] 必须是电机地址
 * @param  len     命令长度
 */
void EmmV5_CAN_SendCmd(uint8_t *cmd, uint16_t len)
{
    if (len == 0 || len > 64) {
#if CAN_CMD_LOG_LEVEL >= 1
        LOGERROR("[CAN] 命令长度无效: %d", len);
#endif
        return;
    }

    uint8_t addr = cmd[0];
    CANInstance *target = NULL;    // 查找目标CAN实例
    for (size_t i = 0; i < idx; ++i)
    {
        if (can_instance[i]->use_ext_id)
        {
            // 扩展帧模式下，比较高8位地址
            if ((can_instance[i]->tx_id >> 8) == addr)
            {
                target = can_instance[i];
                break;
            }
        }
        else
        {
            // 标准帧模式下的匹配（保持向后兼容）
            if ((can_instance[i]->txconf.StdId >> 8) == addr)
            {
                target = can_instance[i];
                break;
            }
        }
    }

    // 如果未找到对应的CAN实例，输出错误日志并返回
    if (!target)
    {
#if CAN_CMD_LOG_LEVEL >= 1
        LOGERROR("[CAN] 未找到电机地址为0x%02X的CAN实例", addr);
#endif
        return;
    }

#if CAN_CMD_LOG_LEVEL >= 2
    {
        char log_buffer[128] = {0};
        int offset = 0;
        
        // 打印命令基本信息
        offset += snprintf(log_buffer + offset, sizeof(log_buffer) - offset, 
                     "[CAN] 发送命令 (ID:0x%02X): ", addr);
        
        // 打印命令数据内容
        for (uint16_t i = 0; i < len && offset < sizeof(log_buffer) - 5; i++) {
            offset += snprintf(log_buffer + offset, sizeof(log_buffer) - offset, "%02X ", cmd[i]);
        }
        LOGINFO("%s", log_buffer);
        
        // 特殊识别编码器读取命令
        if (len >= 2 && cmd[1] == 0x31) {
            LOGINFO("[CAN] 正在请求编码器值，电机地址:0x%02X", addr);
        }
    }
#endif

    uint8_t packet_index = 0;
    uint16_t data_offset = 1;  // 跳过cmd[0]（地址），后续数据从cmd[1]开始

    while (data_offset < len)
    {
        uint8_t frame_len = (len - data_offset > 8) ? 8 : (len - data_offset);
        uint32_t can_id = (addr << 8) + ((len - 1 > 8) ? packet_index : 0);

        memcpy(target->tx_buff, &cmd[data_offset], frame_len);
        
        // 设置CAN ID (根据use_ext_id设置StdId或ExtId)
        if (target->use_ext_id) {
            target->txconf.ExtId = can_id;
        } else {
            target->txconf.StdId = can_id;
        }
        
        CANSetDLC(target, frame_len);

        if (!CANTransmit(target, CAN_TX_TIMEOUT_MS))
        {
#if CAN_CMD_LOG_LEVEL >= 1
            LOGERROR("[CAN] 帧%d发送失败，ID:0x%02X", packet_index, addr);
#endif
            return;
        }

        data_offset += frame_len;
        packet_index++;

        if (len - 1 > 8)
            osDelay(CAN_SEND_DELAY_MS);
    }

#if CAN_CMD_LOG_LEVEL >= 2
    LOGINFO("[CAN] 命令发送成功，ID:0x%02X", addr);
#endif
}
/**
  * @brief    将当前位置清零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_CAN_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0A;                       // 功能码
  cmd[2] =  0x6D;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  EmmV5_CAN_SendCmd(cmd, 4);
}

/**
  * @brief    解除堵转保护
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_CAN_Reset_Clog_Pro(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0E;                       // 功能码
  cmd[2] =  0x52;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  EmmV5_CAN_SendCmd(cmd, 4);
}

/**
  * @brief    读取系统参数
  * @param    addr  ：电机地址
  * @param    s     ：系统参数类型
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_CAN_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[i] = addr; ++i;                   // 地址

  switch(s)                             // 功能码
  {
    case S_VER  : cmd[i] = 0x1F; ++i; break;
    case S_RL   : cmd[i] = 0x20; ++i; break;
    case S_PID  : cmd[i] = 0x21; ++i; break;
    case S_VBUS : cmd[i] = 0x24; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_ORG  : cmd[i] = 0x3B; ++i; break;
    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // 校验字节
  
  // 发送命令
  EmmV5_CAN_SendCmd(cmd, i);
}

/**
  * @brief    修改开环/闭环控制模式
  * @param    addr     ：电机地址
  * @param    svF      ：是否存储标志，false为不存储，true为存储
  * @param    ctrl_mode：控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_CAN_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x46;                       // 功能码
  cmd[2] =  0x69;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  ctrl_mode;                  // 控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  EmmV5_CAN_SendCmd(cmd, 6);
}

/**
  * @brief    使能信号控制
  * @param    addr  ：电机地址
  * @param    state ：使能状态     ，true为使能电机，false为关闭电机
  * @param    snF   ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_CAN_En_Control(uint8_t addr, bool state, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF3;                       // 功能码
  cmd[2] =  0xAB;                       // 辅助码
  cmd[3] =  (uint8_t)state;             // 使能状态
  cmd[4] =  snF;                        // 多机同步运动标志
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  EmmV5_CAN_SendCmd(cmd, 6);
}

/**
  * @brief    速度模式
  * @param    addr：电机地址
  * @param    dir ：方向       ，0为CW，其余值为CCW
  * @param    vel ：速度       ，范围0 - 5000RPM
  * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
  * @param    snF ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_CAN_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF6;                       // 功能码
  cmd[2] =  dir;                        // 方向
  cmd[3] =  (uint8_t)(vel >> 8);        // 速度(RPM)高8位字节
  cmd[4] =  (uint8_t)(vel >> 0);        // 速度(RPM)低8位字节
  cmd[5] =  acc;                        // 加速度，注意：0是直接启动
  cmd[6] =  snF;                        // 多机同步运动标志
  cmd[7] =  0x6B;                       // 校验字节

  // 发送命令
  EmmV5_CAN_SendCmd(cmd, 8);
}

/**
  * @brief    位置模式
  * @param    addr：电机地址
  * @param    dir ：方向        ，0为CW，其余值为CCW
  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_CAN_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 功能码
  cmd[2]  =  dir;                       // 方向
  cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
  cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节 
  cmd[5]  =  acc;                       // 加速度，注意：0是直接启动
  cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
  cmd[10] =  raF;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
  cmd[11] =  snF;                       // 多机同步运动标志，false为不启用，true为启用
  cmd[12] =  0x6B;                      // 校验字节
  
  // 发送命令
  EmmV5_CAN_SendCmd(cmd, 13);
}

/**
  * @brief    立即停止（所有控制模式都通用）
  * @param    addr  ：电机地址
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_CAN_Stop_Now(uint8_t addr, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFE;                       // 功能码
  cmd[2] =  0x98;                       // 辅助码
  cmd[3] =  snF;                        // 多机同步运动标志
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  EmmV5_CAN_SendCmd(cmd, 5);
}

/**
  * @brief    多机同步运动
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_CAN_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFF;                       // 功能码
  cmd[2] =  0x66;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  EmmV5_CAN_SendCmd(cmd, 4);
}

/**
  * @brief    设置单圈回零的零点位置
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志，false为不存储，true为存储
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_CAN_Origin_Set_O(uint8_t addr, bool svF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x93;                       // 功能码
  cmd[2] =  0x88;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  EmmV5_CAN_SendCmd(cmd, 5);
}

/**
  * @brief    修改回零参数
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志，false为不存储，true为存储
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    o_dir  ：回零方向，0为CW，其余值为CCW
  * @param    o_vel  ：回零速度，单位：RPM（转/分钟）
  * @param    o_tm   ：回零超时时间，单位：毫秒
  * @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
  * @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
  * @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
  * @param    potF   ：上电自动触发回零，false为不使能，true为使能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_CAN_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t cmd[32] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x4C;                       // 功能码
  cmd[2] =  0xAE;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[5] =  o_dir;                      // 回零方向
  cmd[6]  =  (uint8_t)(o_vel >> 8);     // 回零速度(RPM)高8位字节
  cmd[7]  =  (uint8_t)(o_vel >> 0);     // 回零速度(RPM)低8位字节 
  cmd[8]  =  (uint8_t)(o_tm >> 24);     // 回零超时时间(bit24 - bit31)
  cmd[9]  =  (uint8_t)(o_tm >> 16);     // 回零超时时间(bit16 - bit23)
  cmd[10] =  (uint8_t)(o_tm >> 8);      // 回零超时时间(bit8  - bit15)
  cmd[11] =  (uint8_t)(o_tm >> 0);      // 回零超时时间(bit0  - bit7 )
  cmd[12] =  (uint8_t)(sl_vel >> 8);    // 无限位碰撞回零检测转速(RPM)高8位字节
  cmd[13] =  (uint8_t)(sl_vel >> 0);    // 无限位碰撞回零检测转速(RPM)低8位字节 
  cmd[14] =  (uint8_t)(sl_ma >> 8);     // 无限位碰撞回零检测电流(Ma)高8位字节
  cmd[15] =  (uint8_t)(sl_ma >> 0);     // 无限位碰撞回零检测电流(Ma)低8位字节 
  cmd[16] =  (uint8_t)(sl_ms >> 8);     // 无限位碰撞回零检测时间(Ms)高8位字节
  cmd[17] =  (uint8_t)(sl_ms >> 0);     // 无限位碰撞回零检测时间(Ms)低8位字节
  cmd[18] =  potF;                      // 上电自动触发回零，false为不使能，true为使能
  cmd[19] =  0x6B;                      // 校验字节
  
  // 发送命令
  EmmV5_CAN_SendCmd(cmd, 20);
}

/**
  * @brief    触发回零
  * @param    addr   ：电机地址
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_CAN_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9A;                       // 功能码
  cmd[2] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[3] =  snF;                        // 多机同步运动标志，false为不启用，true为启用
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  EmmV5_CAN_SendCmd(cmd, 5);
}

/**
  * @brief    强制中断并退出回零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_CAN_Origin_Interrupt(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9C;                       // 功能码
  cmd[2] =  0x48;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  EmmV5_CAN_SendCmd(cmd, 4);
}
/**
  * @brief    读取单个电机编码器值
  * @param    addr  ：电机地址
  * @retval   编码器累计值，失败返回-1
  */
int32_t Emm_V5_CAN_Read_Encoder(uint8_t addr)
{
    int32_t encoder_value = -1;
    int32_t motor_index = addr - 1;  // 将电机地址映射为数组索引
    
    if (motor_index < 0 || motor_index >= 8) {
        // LOGERROR("[CAN] 电机索引超出范围: %d", motor_index);
        return -1;
    }
    
    // 查找该电机地址对应的CAN实例
    CANInstance *target = NULL;
    for (size_t i = 0; i < idx; ++i)
    {
        uint8_t can_addr = (can_instance[i]->rx_id >> 8) & 0xFF;
        if (can_addr == addr)
        {
            target = can_instance[i];
            break;
        }
    }
    
    if (target == NULL)
    {
#if CAN_CMD_LOG_LEVEL >= 1
        LOGERROR("[CAN] 未找到电机地址为0x%02X的CAN实例", addr);
#endif
        return -1;
    }
    
    // 发送读取编码器命令
    Emm_V5_CAN_Read_Sys_Params(addr, S_ENCL);
    
    // 等待一段时间，让电机有时间响应
    osDelay(5);
    
    // 检查接收到的数据是否是编码器值
    if (target->rx_buff[0] == 0x31) // 0x31是读取编码器值的功能码
    {
        // 解析编码器值（两个字节）
        uint16_t raw_value = (target->rx_buff[1] << 8) | target->rx_buff[2];
        
        // 初始化编码器初始值（如果未初始化）
        if (!encoder_initialized[motor_index]) {
            encoder_zero_offset[motor_index] = raw_value;
            last_raw_encoder[motor_index] = raw_value;
            accumulated_encoder[motor_index] = 0;
            encoder_initialized[motor_index] = true;
            LOGINFO("[CAN] 电机地址0x%02X 编码器初始值设置为: %d", addr, raw_value);
            return 0; // 初始位置为0
        }
        
        // 计算差值，考虑编码器回绕
        int32_t diff = raw_value - last_raw_encoder[motor_index];
        
        // 检测跳变
        if (diff > ENCODER_HALF) {
            // 从最大值跳到最小值
            diff -= ENCODER_MAX;
        } else if (diff < -ENCODER_HALF) {
            // 从最小值跳到最大值
            diff += ENCODER_MAX;
        }
        
        // 更新累积值
        accumulated_encoder[motor_index] += diff;
        
        // 保存当前值作为下一次的上一次值
        last_raw_encoder[motor_index] = raw_value;
        
        // 返回累计编码器值
        encoder_value = accumulated_encoder[motor_index];
        
#if CAN_CMD_LOG_LEVEL >= 2
        LOGINFO("[CAN] 电机地址0x%02X 原始编码器值: %d, 累计编码器值: %d", 
                addr, raw_value, encoder_value);
#endif
    }
    else
    {
#if CAN_CMD_LOG_LEVEL >= 1
        LOGERROR("[CAN] 电机地址0x%02X 读取编码器值失败，接收到的功能码: 0x%02X", 
                 addr, target->rx_buff[0]);
#endif
    }
    
    return encoder_value;
}

/**
  * @brief    读取所有电机编码器值
  * @param    encoders  ：存储编码器值的数组，大小应至少为4
  * @retval   是否成功读取全部编码器
  */
bool Emm_V5_CAN_Get_All_Encoders(int32_t *encoders)
{
    if (encoders == NULL) {
        return false;
    }
    
    uint8_t motor_ids[4] = {MOTOR_LF_ID, MOTOR_RF_ID, MOTOR_LB_ID, MOTOR_RB_ID};
    bool all_success = true;
    
    for (uint8_t i = 0; i < 4; i++)
    {
        int32_t value = Emm_V5_CAN_Read_Encoder(motor_ids[i]);
        if (value == -1)
        {
            all_success = false;
        }
        else
        {
            // 将读取到的值存入数组
            encoders[i] = value;
        }
    }
    
    return all_success;
}

/**
  * @brief    重置编码器累计值
  * @param    addr  ：电机地址，如果为0则重置所有电机
  * @retval   无
  */
void Emm_V5_CAN_Reset_Encoder_Count(uint8_t addr)
{
    if (addr == 0) {
        // 重置所有电机的累计值
        for (int i = 0; i < 8; i++) {
            accumulated_encoder[i] = 0;
            // 不重置初始值和上次值，只重置累计值
        }
#if CAN_CMD_LOG_LEVEL >= 2
        LOGINFO("[CAN] 已重置所有电机编码器累计值");
#endif
    } else {
        // 重置特定电机的累计值
        int32_t motor_index = addr - 1;
        if (motor_index >= 0 && motor_index < 8) {
            accumulated_encoder[motor_index] = 0;
#if CAN_CMD_LOG_LEVEL >= 2
            LOGINFO("[CAN] 已重置电机地址0x%02X的编码器累计值", addr);
#endif
        }
    }
}

/**
  * @brief    设置编码器零点偏移
  * @param    addr  ：电机地址
  * @param    offset：零点偏移值
  * @retval   无
  */
void Emm_V5_CAN_Set_Encoder_Zero(uint8_t addr, int32_t offset)
{
    int32_t motor_index = addr - 1;
    if (motor_index >= 0 && motor_index < 8) {
        encoder_zero_offset[motor_index] = offset;
#if CAN_CMD_LOG_LEVEL >= 2
        LOGINFO("[CAN] 已设置电机地址0x%02X的编码器零点偏移为: %d", addr, offset);
#endif
    }
}
