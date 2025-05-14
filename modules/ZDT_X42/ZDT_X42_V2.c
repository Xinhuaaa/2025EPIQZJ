#include "bsp_can.h"
#include "cmsis_os2.h"
#include "string.h"
#include "bsp_log.h"  
#include "ZDT_X42_V2.h"
#include "stdbool.h"
#include "bsp_dwt.h"  

#define CAN_SEND_DELAY_MS   1
#define CAN_TX_TIMEOUT_MS   10.0f
#define MOTOR_LF_ID 5  // 左前轮ID
#define MOTOR_RF_ID 6  // 右前轮ID
#define MOTOR_LB_ID 7  // 左后轮ID
#define MOTOR_RB_ID 8  // 右后轮ID
// 编码器相关常量
#define ENCODER_MAX     65536  // 编码器最大值
#define ENCODER_HALF    32768  // 编码器最大值的一半

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
CANInstance* ZDT_X42_V2_RegisterMotor(uint8_t motor_addr, CAN_HandleTypeDef *can_handle)
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
        .can_module_callback = 1 // 可以根据需要设置回调函数
    };
    
    // 注册CAN实例
    CANInstance *instance = CANRegister(&can_config);
    
    if (instance == NULL) {
        LOGERROR("注册CAN实例失败，地址: 0x%02X", motor_addr);
        return NULL;
    }
    
    LOGINFO("成功注册CAN实例，地址: 0x%02X, TX_ID: 0x%03X, RX_ID: 0x%03X", 
            motor_addr, tx_id, rx_id);
    
    return instance;
}

/**
 * @brief X42电机驱动初始化
 * @param motor_ids 电机ID数组
 * @param motor_count 电机数量
 * @return bool 是否初始化成功
 */
bool ZDT_X42_V2_Init(uint8_t *motor_ids, uint8_t motor_count)
{
    bool all_registered = true;
    
    LOGINFO("初始化X42电机驱动...");
    
    // 注册所有电机的CAN实例
    for (uint8_t i = 0; i < motor_count; i++) {
        uint8_t motor_addr = motor_ids[i];
        if (ZDT_X42_V2_RegisterMotor(motor_addr, &hcan1) == NULL) {
            all_registered = false;
        }
    }
    
    if (!all_registered) {
        LOGERROR("部分电机CAN实例注册失败");
    } else {
        LOGINFO("所有电机CAN实例注册成功");
    }
        return all_registered;
}

/**
 * @brief  通用发送 CAN 命令（根据cmd[0]地址匹配CANInstance）
 * @param  cmd     命令数据，cmd[0] 必须是电机地址
 * @param  len     命令长度
 */
void can_SendCmd(uint8_t *cmd, uint16_t len)
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
void ZDT_X42_V2_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0A;                       // 功能码
  cmd[2] =  0x6D;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 4);
}

/**
  * @brief    解除堵转保护
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Reset_Clog_Pro(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0E;                       // 功能码
  cmd[2] =  0x52;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 4);
}

/**
  * @brief    读取系统参数
  * @param    addr  ：电机地址
  * @param    s     ：系统参数类型
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址

  switch(s)                             // 功能码
  {
    case S_VER   : cmd[1] = 0x1F; break;                  /* 读取固件版本和对应的硬件版本 */
    case S_RL    : cmd[1] = 0x20; break;                  /* 读取读取相电阻和相电感 */
    case S_PID   : cmd[1] = 0x21; break;                  /* 读取PID参数 */
    case S_ORG   : cmd[1] = 0x22; break;                  /* 读取回零参数 */
    case S_VBUS  : cmd[1] = 0x24; break;                  /* 读取总线电压 */
    case S_CBUS  : cmd[1] = 0x26; break;                  /* 读取总线电流 */
    case S_CPHA  : cmd[1] = 0x27; break;                  /* 读取相电流 */
    case S_ENC   : cmd[1] = 0x29; break;                  /* 读取编码器原始值 */
    case S_CPUL  : cmd[1] = 0x30; break;                  /* 读取实时脉冲数（根据实时位置计算得到的脉冲数） */
    case S_ENCL  : cmd[1] = 0x31; break;                  /* 读取经过线性化校准后的编码器值 */
    case S_TPUL  : cmd[1] = 0x32; break;                  /* 读取输入脉冲数 */
    case S_TPOS  : cmd[1] = 0x33; break;                  /* 读取电机目标位置 */
    case S_OPOS  : cmd[1] = 0x34; break;                  /* 读取电机实时设定的目标位置（开环模式的实时位置） */
    case S_VEL   : cmd[1] = 0x35; break;                  /* 读取电机实时转速 */
    case S_CPOS  : cmd[1] = 0x36; break;                  /* 读取电机实时位置（基于角度编码器累加的电机实时位置） */
    case S_PERR  : cmd[1] = 0x37; break;                  /* 读取电机位置误差 */
    case S_TEMP  : cmd[1] = 0x39; break;                  /* 读取电机实时温度 */
    case S_SFLAG : cmd[1] = 0x3A; break;                  /* 读取状态标志位 */
    case S_OFLAG : cmd[1] = 0x3B; break;                  /* 读取回零状态标志位 */
    case S_Conf  : cmd[1] = 0x42; cmd[2] = 0x6C; break;   /* 读取驱动参数 */
    case S_State : cmd[1] = 0x43; cmd[2] = 0x7A; break;   /* 读取系统状态参数 */
    default: break;
  }
  
  // 记录最后发送的命令类型，用于后续匹配
  if (s == S_ENCL) {
      // 查找该电机地址对应的实例，并设置一个标志表示正在请求编码器值
      for (size_t i = 0; i < idx; ++i) {
          uint8_t can_addr = (can_instance[i]->rx_id >> 8) & 0xFF;
          if (can_addr == addr) {
              // 标记正在读取编码器
              LOGINFO("[SYS_PARAMS] 电机ID:0x%02X 正在请求编码器值", addr);
              break;
          }
      }
  }

  // 发送命令
  if(s >= S_Conf)
  {
    cmd[3] = 0x6B; 
    LOGINFO("[SYS_PARAMS] 发送命令: 地址=0x%02X, 功能=0x%02X 0x%02X", cmd[0], cmd[1], cmd[2]);
    can_SendCmd(cmd, 4);
  }
  else
  {
    cmd[2] = 0x6B; 
    LOGINFO("[SYS_PARAMS] 发送命令: 地址=0x%02X, 功能=0x%02X", cmd[0], cmd[1]);
    can_SendCmd(cmd, 3);
  }
}

/**
  * @brief    修改开环/闭环控制模式
  * @param    addr     ：电机地址
  * @param    svF      ：是否存储标志，false为不存储，true为存储
  * @param    ctrl_mode：控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x46;                       // 功能码
  cmd[2] =  0x69;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  ctrl_mode;                  // 控制模式（对应屏幕上的Ctrl_Mode菜单），0是开环模式，1是FOC矢量闭环模式
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 6);
}

/**
  * @brief    使能信号控制
  * @param    addr  ：电机地址
  * @param    state ：使能状态     ，true为使能电机，false为关闭电机
  * @param    snF   ：多机同步标志 ，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_En_Control(uint8_t addr, bool state, uint8_t snF)
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
  can_SendCmd(cmd, 6);
}

/**
  * @brief    力矩模式
  * @param    addr  ：电机地址
  * @param    sign  ：符号         ，0为正，其余值为负
  * @param    t_ramp：斜率(Ma/s)   ，范围0 - 65535Ma/s
  * @param    torque：力矩(Ma)     ，范围0 - 4000Ma
  * @param    snF   ：多机同步标志 ，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Torque_Control(uint8_t addr, uint8_t sign, uint16_t t_ramp, uint16_t torque, uint8_t snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF5;                       // 功能码
  cmd[2] =  sign;                       // 符号（方向）
  cmd[3] =  (uint8_t)(t_ramp >> 8);     // 力矩斜率(Ma/s)高8位字节
  cmd[4] =  (uint8_t)(t_ramp >> 0);     // 力矩斜率(Ma/s)低8位字节
  cmd[5] =  (uint8_t)(torque >> 8);     // 力矩(Ma)高8位字节
  cmd[6] =  (uint8_t)(torque >> 0);     // 力矩(Ma)低8位字节
  cmd[7] =  snF;                        // 多机同步运动标志
  cmd[8] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 9);
}

/**
  * @brief    速度模式
  * @param    addr  	：电机地址
  * @param    dir     ：方向         ，0为CW，其余值为CCW
  * @param    v_ramp  ：斜率(RPM/s)  ，范围0 - 65535RPM/s
  * @param    velocity：速度(RPM)    ，范围0.0 - 4000.0RPM
  * @param    snF     ：多机同步标志 ，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Velocity_Control(uint8_t addr, uint8_t dir, uint16_t v_ramp, float velocity, uint8_t snF)
{
  uint8_t cmd[16] = {0}; uint16_t vel = 0;

  // 将速度放大10倍发送过去
  vel = (uint16_t)ABS(velocity * 10.0f);

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF6;                       // 功能码
  cmd[2] =  dir;                        // 符号（方向）
  cmd[3] =  (uint8_t)(v_ramp >> 8);     // 速度斜率(RPM/s)高8位字节
  cmd[4] =  (uint8_t)(v_ramp >> 0);     // 速度斜率(RPM/s)低8位字节
  cmd[5] =  (uint8_t)(vel >> 8);        // 速度(RPM)高8位字节
  cmd[6] =  (uint8_t)(vel >> 0);        // 速度(RPM)低8位字节
  cmd[7] =  snF;                        // 多机同步运动标志
  cmd[8] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 9);
}

/**
  * @brief    直通限速位置模式
  * @param    addr  	：电机地址
  * @param    dir     ：方向										，0为CW，其余值为CCW
  * @param    velocity：最大速度(RPM)					，范围0.0 - 4000.0RPM
  * @param    position：位置(°)								，范围0.0°- (2^32 - 1)°
  * @param    raf     ：相位位置/绝对位置标志	，0为相对位置，其余值为绝对位置
  * @param    snF     ：多机同步标志						，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Bypass_Position_LV_Control(uint8_t addr, uint8_t dir, float velocity, float position, uint8_t raf, uint8_t snF)
{
  uint8_t cmd[16] = {0}; uint16_t vel = 0; uint32_t pos = 0;

  // 将速度和位置放大10倍发送过去
  vel = (uint16_t)ABS(velocity * 10.0f); pos = (uint32_t)ABS(position * 10.0f);

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFB;                      // 功能码
  cmd[2]  =  dir;                       // 符号（方向）
  cmd[3]  =  (uint8_t)(vel >> 8);       // 最大速度(RPM)高8位字节
  cmd[4]  =  (uint8_t)(vel >> 0);       // 最大速度(RPM)低8位字节 
  cmd[5]  =  (uint8_t)(pos >> 24);      // 位置(bit24 - bit31)
  cmd[6]  =  (uint8_t)(pos >> 16);      // 位置(bit16 - bit23)
  cmd[7]  =  (uint8_t)(pos >> 8);       // 位置(bit8  - bit15)
  cmd[8]  =  (uint8_t)(pos >> 0);       // 位置(bit0  - bit7 )
  cmd[9]  =  raf;                       // 相位位置/绝对位置标志
  cmd[10] =  snF;                       // 多机同步运动标志
  cmd[11] =  0x6B;                      // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 12);
}

/**
  * @brief    梯形曲线位置模式
  * @param    addr  	：电机地址
  * @param    dir     ：方向										，0为CW，其余值为CCW
  * @param    acc     ：加速加速度(RPM/s)			，0为CW，其余值为CCW
  * @param    dec     ：减速加速度(RPM/s)			，0为CW，其余值为CCW
  * @param    velocity：最大速度(RPM)					，范围0.0 - 4000.0RPM
  * @param    position：位置(°)								，范围0.0°- (2^32 - 1)°
  * @param    raf     ：相位位置/绝对位置标志	，0为相对位置，其余值为绝对位置
  * @param    snF     ：多机同步标志						，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Traj_Position_Control(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float velocity, float position, uint8_t raf, uint8_t snF)
{
  uint8_t cmd[32] = {0}; uint16_t vel = 0; uint32_t pos = 0;

  // 将速度和位置放大10倍发送过去
  vel = (uint16_t)ABS(velocity * 10.0f); pos = (uint32_t)ABS(position * 10.0f);

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 功能码
  cmd[2]  =  dir;                       // 符号（方向）
  cmd[3]  =  (uint8_t)(acc >> 8);       // 加速加速度(RPM/s)高8位字节
  cmd[4]  =  (uint8_t)(acc >> 0);       // 加速加速度(RPM/s)低8位字节  
  cmd[5]  =  (uint8_t)(dec >> 8);       // 减速加速度(RPM/s)高8位字节
  cmd[6]  =  (uint8_t)(dec >> 0);       // 减速加速度(RPM/s)低8位字节  
  cmd[7]  =  (uint8_t)(vel >> 8);       // 最大速度(RPM)高8位字节
  cmd[8]  =  (uint8_t)(vel >> 0);       // 最大速度(RPM)低8位字节 
  cmd[9]  =  (uint8_t)(pos >> 24);      // 位置(bit24 - bit31)
  cmd[10] =  (uint8_t)(pos >> 16);      // 位置(bit16 - bit23)
  cmd[11] =  (uint8_t)(pos >> 8);       // 位置(bit8  - bit15)
  cmd[12] =  (uint8_t)(pos >> 0);       // 位置(bit0  - bit7 )
  cmd[13] =  raf;                       // 相位位置/绝对位置标志
  cmd[14] =  snF;                       // 多机同步运动标志
  cmd[15] =  0x6B;                      // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 16);
}

/**
  * @brief    立即停止（所有控制模式都通用）
  * @param    addr  ：电机地址
  * @param    snF   ：多机同步标志，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Stop_Now(uint8_t addr, uint8_t snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFE;                       // 功能码
  cmd[2] =  0x98;                       // 辅助码
  cmd[3] =  snF;                        // 多机同步运动标志
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 5);
}

/**
  * @brief    多机同步运动
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFF;                       // 功能码
  cmd[2] =  0x66;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 4);
}

/**
  * @brief    设置单圈回零的零点位置
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志，false为不存储，true为存储
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Origin_Set_O(uint8_t addr, bool svF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x93;                       // 功能码
  cmd[2] =  0x88;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 5);
}

/**
  * @brief    修改回零参数
  * @param    addr   ：电机地址
  * @param    svF    ：是否存储标志，false为不存储，true为存储
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
void ZDT_X42_V2_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
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
  can_SendCmd(cmd, 20);
}

/**
  * @brief    触发回零
  * @param    addr   ：电机地址
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    snF    ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9A;                       // 功能码
  cmd[2] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  cmd[3] =  snF;                        // 多机同步运动标志，false为不启用，true为启用
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  can_SendCmd(cmd, 5);
}

/**
  * @brief    强制中断并退出回零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Origin_Interrupt(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9C;                       // 功能码
  cmd[2] =  0x48;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节    // 发送命令
  can_SendCmd(cmd, 4);
}

/**
  * @brief    读取单个电机编码器值
  * @param    addr  ：电机地址
  * @retval   编码器累计值，失败返回-1
  */
int32_t ZDT_X42_V2_Read_Encoder(uint8_t addr)
{
    int32_t encoder_value = -1;
    int32_t motor_index = addr - 1;  // 将电机地址映射为数组索引
    
    if (motor_index < 0 || motor_index >= 8) {
        LOGERROR("[CAN] 电机索引超出范围: %d", motor_index);
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
    ZDT_X42_V2_Read_Sys_Params(addr, S_ENCL);
    
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
bool ZDT_X42_V2_Get_All_Encoders(int32_t *encoders)
{
    if (encoders == NULL) {
        return false;
    }
    
    uint8_t motor_ids[4] = {MOTOR_LF_ID, MOTOR_RF_ID, MOTOR_LB_ID, MOTOR_RB_ID};
    bool all_success = true;
    
    for (uint8_t i = 0; i < 4; i++)
    {
        int32_t value = ZDT_X42_V2_Read_Encoder(motor_ids[i]);
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
void ZDT_X42_V2_Reset_Encoder_Count(uint8_t addr)
{
    if (addr == 0) {
        // 重置所有电机的累计值
        for (int i = 0; i < 8; i++) {
            accumulated_encoder[i] = 0;
            // 不重置初始值和上次值，只重置累计值
        }
        LOGINFO("[CAN] 已重置所有电机编码器累计值");
    } else {
        // 重置特定电机的累计值
        int32_t motor_index = addr - 1;
        if (motor_index >= 0 && motor_index < 8) {
            accumulated_encoder[motor_index] = 0;
            LOGINFO("[CAN] 已重置电机地址0x%02X的编码器累计值", addr);
        }
    }
}

/**
  * @brief    设置编码器零点偏移
  * @param    addr  ：电机地址
  * @param    offset：零点偏移值
  * @retval   无
  */
void ZDT_X42_V2_Set_Encoder_Zero(uint8_t addr, int32_t offset)
{
    int32_t motor_index = addr - 1;
    if (motor_index >= 0 && motor_index < 8) {
        encoder_zero_offset[motor_index] = offset;
        LOGINFO("[CAN] 已设置电机地址0x%02X的编码器零点偏移为: %d", addr, offset);
    }
}
