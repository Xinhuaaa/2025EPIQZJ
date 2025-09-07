# 2025 国三起重机电控系统

[![Build](https://img.shields.io/badge/build-passing-brightgreen)]() 
[![License](https://img.shields.io/badge/license-MIT-blue)](./LICENSE)
[![Platform](https://img.shields.io/badge/platform-STM32F407VET6-orange)]()
[![Stars](https://img.shields.io/github/stars/Xinhuaaa/2025EPIQZJ?style=social)]()

基于 **STM32F407VET6** 的移动抓取机器人控制系统。  
这是东莞理工学院EPI实验室为 **2025 年起重机竞赛** 开发的一套电控与软件，目标是稳定可靠地实现底盘运动、升降和抓取功能。  

---

## 项目简介

- 底盘：三自由度全向运动 (X / Y / Yaw)，基于 ADRC 控制  
- 升降：4 个 M2006 电机 (CAN 总线)，联动控制  
- 抓取：状态机实现顺序动作（升降 → 伸展 → 抓取 → 收缩 → 放下）  

硬件设计开源地址：[OSHWHub 项目页](https://oshwhub.com/epi-laboratory/25-crane)

---

## 硬件配置

| 类型       | 型号            | 通信方式 | 说明         |
|------------|-----------------|----------|--------------|
| 主控       | STM32F407VET6   | -        | Cortex-M4F, 168MHz |
| 电机       | DJI M2006       | CAN      | 升降驱动     |
| 步进电机   | ZDT EMM57       | CAN      | 底盘运动     |
| 步进电机   | ZDT EMM42       | UART     | 保留的驱动控制     |
| 传感器     | HWT101 IMU      | UART     | 陀螺仪读取角度     |
| 编码器     | 轮式里程计      | -        | 位置反馈     |
| 测距       | STP23           | -        | 激光测距，上位机处理 |

---

## 软件结构

```text
Application/      # 应用层
├── Chassis       # 底盘控制
├── Lift          # 升降系统
└── Crawl         # 抓取逻辑

modules/          # 驱动/算法
├── dji_motor     # M2006/M3508 驱动
├── ZDT_EmmV5     # 步进电机驱动
├── Hwt101        # IMU 接口
├── PID           # PID 控制
└── ADRC          # 自抗扰控制

bsp/              # 板级支持
├── bsp_can       # CAN 抽象
├── bsp_log       # RTT 日志
└── bsp_dwt       # 精确延时
