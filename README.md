# 2025年起重机国三电控系统

> 基于STM32F407VET6的移动抓取机器人控制系统

## 📖 项目概述

本项目是一个先进的嵌入式机器人控制系统，专为2025年起重机竞赛设计。系统采用STM32F407VET6微控制器作为主控，实现了具有**移动平台操控能力**的机器人，支持精确的三自由度运动控制和复杂的物体抓取操作。

### 🎯 主要功能

- **移动底盘**：3自由度运动控制（X, Y, Yaw），采用ADRC控制算法
- **升降结构**：基于4个M2006电机的垂直运动系统（18cm轮径，36:1减速比）
- **抓取机构**：多阶段操作系统（升降→伸展→抓取→收缩→降低）

## 🔧 硬件配置

### 主控制器
- **MCU**: 嘉立创天空星STM32F407VET6 (ARM Cortex-M4F, 168MHz, 支持FPU)
- **开发环境**: VSCode + EIDE扩展
- **RTOS**: FreeRTOS with CMSIS-RTOS V2

### 核心外设

| 设备类型 | 型号 | 通信方式 | 功能描述 |
|---------|------|----------|----------|
| 电机控制 | 大疆M2006 | CAN总线 | 升降结构驱动 |
| 步进电机 | 张大头EMM57 | CAN通信 | 精密运动控制 |
| 步进电机 | 张大头EMM42 | UART通信 | 辅助运动控制 |
| IMU传感器 | HWT101陀螺仪 | UART | 姿态检测 |
| 编码器 | 轮式里程计 | - | 位置反馈 |
| 激光测距 | STP23 | - | 距离检测（上位机处理）|

### 硬件开源资源
🔗 **硬件设计开源地址**: [https://oshwhub.com/epi-laboratory/25-crane](https://oshwhub.com/epi-laboratory/25-crane)

## 🏗️ 软件架构

### 分层架构设计
```
Application/          # 高层机器人控制
├── Robot_task.c/h   # 主系统初始化
├── Chassis/         # 移动平台控制
├── Lift/            # 垂直运动系统
└── Crawl/           # 物体抓取操作

modules/             # 硬件抽象模块
├── dji_motor/       # 大疆电机控制 (M2006/M3508)
├── ZDT_EmmV5/       # 步进电机控制
├── Hwt101/          # IMU传感器接口
├── servo/           # 舵机控制
├── PID/             # PID控制器
└── ADRC/            # 自抗扰控制

bsp/                 # 板级支持包
├── bsp_can/         # CAN总线抽象
├── bsp_log/         # RTT日志系统
├── bsp_dwt/         # 高精度定时
└── bsp_printf/      # Printf实现
```

### 控制系统特性
- **多电机协调控制**：支持同步控制多个电机
- **先进控制算法**：ADRC（自抗扰控制）和PID控制器
- **状态机设计**：用于复杂的顺序操作（特别是抓取模块）
- **实时任务调度**：基于FreeRTOS的任务分配

## 🚀 快速开始

### 环境要求

#### 软件环境
- **开发工具**: VSCode + EIDE扩展
- **编译工具链**: ARM GCC
- **调试器**: J-Link (推荐) / OpenOCD / pyOCD
- **操作系统**: Windows/Linux/macOS

#### 硬件环境
- STM32F407VET6开发板
- J-Link调试器或CMSIS-DAP接口
- CAN收发器模块
- 相应的电机和传感器

### 编译构建

项目使用STM32CubeMX生成的Makefile进行构建：

```bash
# 克隆项目
git clone https://github.com/Xinhuaaa/2025EPIQZJ.git
cd 2025EPIQZJ

# 编译项目
make

# 清理构建产物
make clean

# 编译特定目标
make build/TKX.elf    # ELF可执行文件
make build/TKX.hex    # Intel HEX格式
make build/TKX.bin    # 二进制格式
```

### VSCode开发

通过VSCode任务系统 (Ctrl+Shift+P)：
- **build**: 编译项目
- **flash**: 烧录到设备
- **build and flash**: 一键编译并烧录
- **rebuild**: 清理并重新编译
- **clean**: 清理构建产物

## 🔍 调试与日志

### 实时日志系统

项目使用**SEGGER RTT**进行实时调试输出：

```c
// 日志级别 (在 bsp_log.h 中定义)
LOGINFO("信息消息");
LOGWARNING("警告消息");
LOGERROR("错误消息");

// 浮点数输出（需要先转换）
float value = 123.45;
char str_buff[64];
Float2Str(str_buff, value);
printf_log("浮点值: %s\n", str_buff);
```

**重要**: RTT日志需要J-Link调试会话。请先启动"Debug: JLINK"配置的调试，然后打开RTT查看器。

### 调试配置

项目支持多种调试接口：
- **J-Link**: 推荐用于RTT日志 (使用"Debug: JLINK"启动配置)
- **OpenOCD**: 配合CMSIS-DAP接口
- **pyOCD**: 替代调试器选择

## 📁 关键文件说明

| 文件路径 | 功能描述 |
|---------|----------|
| `Application/Robot_task.c` | 主系统初始化序列 |
| `bsp/log/bsp_log.h` | 日志宏和函数 |
| `modules/dji_motor/dji_motor.h` | 电机控制接口 |
| `Inc/main.h` | 全局包含和定义 |
| `Inc/FreeRTOSConfig.h` | RTOS配置 |

## 🛠️ 开发指南

### 添加新功能的步骤

1. **硬件抽象**: 如需要，在BSP层添加硬件抽象代码
2. **模块驱动**: 为传感器/执行器创建模块级驱动
3. **应用逻辑**: 实现应用级控制逻辑
4. **初始化集成**: 在`Robot_Init()`中添加初始化调用
5. **调试验证**: 使用适当的日志进行调试

### 电机控制规范
- **大疆电机**: 使用CAN总线通信，通过`dji_motor`模块控制
- **步进电机**: EMM V5系列，通过UART由`ZDT_EmmV5`模块控制  
- **舵机**: 通过`servo`模块的PWM控制
- **协调控制**: 通过同步控制循环进行多电机协调

### 定时与同步
- **DWT**: 使用`DWT_Delay()`进行微秒级精确延时
- **FreeRTOS**: 基于任务的调度，合理设置优先级
- **中断**: 精心管理中断优先级以确保实时性能

## 🤝 贡献指南

欢迎为项目贡献代码！请遵循以下规范：

1. **代码风格**: 遵循现有的代码风格和命名规范
2. **模块化设计**: 保持良好的模块化设计和硬件抽象
3. **文档更新**: 更新相关的代码注释和文档
4. **测试验证**: 确保新功能经过充分测试

### 开发流程
1. Fork本仓库
2. 创建特性分支 (`git checkout -b feature/新功能`)
3. 提交更改 (`git commit -am '添加新功能'`)
4. 推送到分支 (`git push origin feature/新功能`)
5. 创建Pull Request

## 📄 许可证

本项目采用开源许可证，详情请查看LICENSE文件。

## 📞 联系方式

如有技术问题或建议，欢迎通过以下方式联系：

- **项目仓库**: [GitHub Issues](https://github.com/Xinhuaaa/2025EPIQZJ/issues)
- **硬件讨论**: [立创开源平台](https://oshwhub.com/epi-laboratory/25-crane)

---

> **注意**: 本项目为竞赛专用代码，仅供学习和技术交流使用。
