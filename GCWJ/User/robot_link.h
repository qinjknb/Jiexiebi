/**
  ******************************************************************************
  * @file    robot_link.h
  * @brief   STM32 <-> 电脑（ROS 2 上位机）通信协议 + 原 ServoHost 文本命令兼容
  *
  * 串口：USART2，PA2=TX / PA3=RX，8N1，默认 115200
  *
  * ---- 二进制帧（推荐，ROS 2 用）----------------------------------------
  *   主机 → STM32:  AA 55 | CMD     | SEQ | LEN | PAYLOAD[LEN] | CRC_H CRC_L
  *   STM32 → 主机:  AA 55 | CMD|0x80 | SEQ | LEN | PAYLOAD[LEN] | CRC_H CRC_L
  *   CRC16-CCITT：初值 0xFFFF，多项式 0x1021，对 CMD/SEQ/LEN/PAYLOAD 计算，高字节先发
  *
  *   指令表（CMD）与负载：
  *     0x01 PING         -            → 0x81 PONG  {ver_major, ver_minor, njoint, busmode}
  *     0x02 SET_TARGETS  6×int16(BE)   → 0x82 ACK   目标位置 0~4095
  *     0x03 SET_MODE     u8 (0/1/2)    → ACK   0=保持 1=跟随 2=回中
  *     0x04 HOME         u16(BE) ms    → ACK   按时间插值回中位
  *     0x05 SET_LIMITS   12×u16(BE)    → ACK   前 6 个下限，后 6 个上限
  *     0x06 READ_POS     -            → 0x84 POS  6×u16(BE)，0xFFFF=未知
  *     0x07 SET_TELEM    u8 Hz        → ACK   状态帧上报频率 0~100
  *     0x08 STOP         -            → ACK   立即停止跟随（保持当前位置）
  *     0x09 GET_STATUS   -            → 0x83 STATE
  *     0x0A SET_BUSMODE  u8 (0/1/2)   → ACK   0=SYNC WRITE 1=逐帧写 2=REG+ACTION
  *     0x83 STATE 负载： {flags, mode, 6×u16 cmd(BE), 6×u16 meas(BE)} 共 26 字节
  *     0x82 ACK 负载：   {CMD, result}  result: 0=OK 1=未知指令 2=长度错 3=忙 4=超范围
  *
  * ---- 文本命令（兼容原 ServoHost 上位机，'\n' 结尾）----------------------
  *   PING id / MOVE id 角度 时间 / MOVEP id 位置 时间 / READ id / SETID id
  *   RANGE 角度 / RESET id / HELP / HOME [ms] / JOINTS / BUSMODE n / STREAM on|off
  ******************************************************************************
  */

#ifndef __ROBOT_LINK_H
#define __ROBOT_LINK_H

#include "stm32f10x.h"
#include "robot_config.h"

/* 帧头与响应位 */
#define LINK_SOH0               0xAAU
#define LINK_SOH1               0x55U
#define LINK_REPLY_FLAG         0x80U

/* 主机 → STM32 */
#define LINK_CMD_PING           0x01U
#define LINK_CMD_SET_TARGETS    0x02U
#define LINK_CMD_SET_MODE       0x03U
#define LINK_CMD_HOME           0x04U
#define LINK_CMD_SET_LIMITS     0x05U
#define LINK_CMD_READ_POS       0x06U
#define LINK_CMD_SET_TELEM      0x07U
#define LINK_CMD_STOP           0x08U
#define LINK_CMD_GET_STATUS     0x09U
#define LINK_CMD_SET_BUSMODE    0x0AU

/* STM32 → 主机 */
#define LINK_RSP_PONG           0x81U
#define LINK_RSP_ACK            0x82U
#define LINK_RSP_STATE          0x83U
#define LINK_RSP_POS            0x84U

/* ACK 结果码 */
#define LINK_RES_OK             0U
#define LINK_RES_UNKNOWN_CMD    1U
#define LINK_RES_BAD_LEN        2U
#define LINK_RES_BUSY           3U
#define LINK_RES_RANGE          4U

/* 固件版本 */
#define LINK_FW_VER_MAJOR       2U
#define LINK_FW_VER_MINOR       0U

/* 初始化串口与协议层 */
void Link_Init(uint32_t baud);

/* 主循环里反复调用：解析命令、收发文本、定时上报状态 */
void Link_Poll(void);

/* 主动发送 */
void Link_SendPong(uint8_t seq);
void Link_SendAck(uint8_t seq, uint8_t cmd, uint8_t result);
void Link_SendState(void);
void Link_SendPos(void);
void Link_SendText(const char *s);

/* 工具 */
uint16_t Link_Crc16(const uint8_t *p, uint16_t n);

#endif /* __ROBOT_LINK_H */
