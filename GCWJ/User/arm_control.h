/**
  ******************************************************************************
  * @file    arm_control.h
  * @brief   机械臂执行层：目标跟随、限速插值、软启动、总线同步下发、位置回读
  ******************************************************************************
  */

#ifndef __ARM_CONTROL_H
#define __ARM_CONTROL_H

#include "stm32f10x.h"
#include "robot_config.h"

/* 运行模式 */
#define ARM_MODE_IDLE           0U      /* 保持不动（不刷新总线） */
#define ARM_MODE_STREAM         1U      /* 跟随上位机目标（ROS 轨迹） */
#define ARM_MODE_HOME           2U      /* 回中位（定时插值） */

/* 状态标志（会上报给上位机，同步显示在 LED/串口） */
#define ARM_FLAG_STREAM         0x01U
#define ARM_FLAG_TIMEOUT        0x02U
#define ARM_FLAG_LIMIT          0x04U   /* 有目标被软限位钳过 */
#define ARM_FLAG_READBACK_ERR   0x08U   /* 有舵机读位置失败 */

/* 初始化（上电调用一次） */
void Arm_Init(void);

/* 主循环调用：按 ARM_SERVO_PERIOD_MS 节拍推进 */
void Arm_Tick(void);

/* 上位机命令 */
void    Arm_SetTargets(const uint16_t *ticks);      /* 6 个目标位置(0~4095) */
void    Arm_SetMode(uint8_t mode);
void    Arm_Stop(void);
void    Arm_Home(uint16_t timeMs);
void    Arm_SetLimits(const uint16_t *mn, const uint16_t *mx);
uint8_t Arm_SetBusMode(uint8_t m);

/* 状态查询 */
uint8_t  Arm_GetMode(void);
uint8_t  Arm_GetFlags(void);
uint8_t  Arm_GetBusMode(void);
uint16_t Arm_GetPeriodMs(void);
void     Arm_GetCmd(uint16_t *out);         /* 当前下发的 6 个位置 */
void     Arm_GetMeasured(uint16_t *out);    /* 回读的真实位置，0xFFFF=未知 */

#endif /* __ARM_CONTROL_H */
