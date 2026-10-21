/**
  ******************************************************************************
  * @file    Servo.h
  * @brief   德晟串口总线智能舵机协议 v4.01 驱动接口（自诊断版）
  *
  * 协议要点（详见《德晟串口智能舵机协议文件 v4.01.pdf》）：
  *  - 半双工主从问答，8N1
  *  - 主机帧：FF FF | ID | LEN | CMD | 参数... | SUM
  *      LEN = ID+CMD+参数 的字节数 = N+2
  *      SUM = ~(ID + LEN + CMD + 参数1 + ... + 参数N) & 0xFF
  *  - 舵机应答：FF F5 | ID | LEN | 状态 | 参数... | SUM
  *  - ID 有效范围 1~250，广播 ID = 254
  *  - 多字节数据一律大端；目标位置 0x2A/0x2B（0~4095），运行时间 0x2C/0x2D（ms）
  ******************************************************************************
  */

#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f10x.h"

/* ============================ 协议常量 ==================================== */
#define SERVO_BROADCAST_ID      254U

#define SERVO_CMD_PING          0x01U
#define SERVO_CMD_READ          0x02U
#define SERVO_CMD_WRITE         0x03U
#define SERVO_CMD_REG_WRITE     0x04U
#define SERVO_CMD_ACTION        0x05U
#define SERVO_CMD_RESET         0x06U
#define SERVO_CMD_SYNC_WRITE    0x83U

#define SERVO_ADDR_TARGET_POS   0x2AU
#define SERVO_ADDR_RUN_TIME     0x2CU
#define SERVO_ADDR_CUR_POS      0x38U

/* ===================== 总线配置（自检轮询） ================================ */
#define SERVO_PIN_PA9_TX        0U      /* 双线模式：PA9 发送，PA10 接收 */
#define SERVO_PIN_PA10_HD       1U      /* 单线半双工：只用 PA10 */
#define SERVO_PIN_PA9_HD        2U      /* 单线半双工：只用 PA9 */
#define SERVO_PHASE_MAX         7U

/* ============================ 运行诊断结构 ================================ */
typedef struct
{
    uint32_t magic;             /* 0xD1A60001，确认结构体位置/版本 */
    uint32_t tickMs;            /* SysTick 毫秒计数（观察它是否增长） */
    uint8_t  phase;             /* 当前总线配置阶段 */
    uint8_t  servoId;           /* 当前操作的舵机 ID */
    uint16_t reserved;
    uint32_t txBytes;           /* 累计发送字节数 */
    uint32_t rxBytes;           /* 累计接收字节数 */
    uint32_t readOk[SERVO_PHASE_MAX];    /* 各阶段读位置成功次数 */
    uint32_t moveSent[SERVO_PHASE_MAX];  /* 各阶段发送转动指令次数 */
    uint32_t lastRxByte;        /* 最后收到的字节 */
    uint8_t  foundCount;        /* PING 扫描找到的舵机个数 */
    uint8_t  foundIds[6];       /* 找到的舵机 ID 列表 */
    uint8_t  foundPad;          /* 对齐 */
    uint32_t syncWrites;        /* SYNC WRITE 帧累计发送次数 */
    uint32_t ackFail;           /* 等应答失败累计次数 */
    uint32_t moveCmds;          /* 机械臂下发的控制指令总数 */
} Diag_t;

extern volatile Diag_t g_diag;

/* ============================ 接口函数 ==================================== */
/* ---- 初始化/配置 ---- */
void Servo_Config(uint8_t pinSel, uint32_t baud);       /* 切换总线接法/波特率 */

/* ---- 协议底层 ---- */
void Servo_SendFrame(uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t n);

/* ---- 舵机控制封装 ---- */
void Servo_Move(uint8_t id, uint16_t position, uint16_t timeMs);   /* 转到目标位置(0~4095) */
void Servo_SetID(uint8_t id);                       /* 广播把总线上所有舵机 ID 设为 id */
int  Servo_Ping(uint8_t id, uint32_t timeoutMs);    /* 查询舵机是否存在 */
int  Servo_ReadPosition(uint8_t id, uint16_t *pos, uint32_t timeoutMs); /* 读当前位置 */
void Servo_MoveSync(const uint8_t *ids, const uint16_t *positions,
                    const uint16_t *times, uint8_t n);    /* 多舵机同时起步 */
void Servo_MoveV5(uint8_t id, int16_t angleDeciDeg, uint16_t timeMs); /* v5 协议兼容 */

/* ---- 机械臂用：多舵机同步控制（帧格式见《德晟…v4.01》2.3.4~2.3.6） ---- */
void Servo_RegWritePosTime(uint8_t id, uint16_t position, uint16_t timeMs); /* 异步预写 */
void Servo_Action(void);                            /* 广播执行所有异步预写 */
void Servo_SyncWritePosTime(uint8_t n, const uint8_t *ids,
                            const uint16_t *positions, const uint16_t *times); /* 一帧同步写 */
int  Servo_WaitAck(uint8_t id, uint32_t timeoutMs); /* 等一帧应答(任意指令) */
int  Servo_MoveWaitAck(uint8_t id, uint16_t position, uint16_t timeMs); /* 转动并等应答 */

/* ---- 角度控制（0~4095 满行程对应 s_angleRange 角度，默认 300°） ---- */
void Servo_SetAngleRange(uint16_t rangeDeg);        /* 修改满行程角度（180/270/300/360 等） */
void Servo_MoveAngle(uint8_t id, float angleDeg, uint16_t timeMs);  /* 转到指定角度(°) */
int  Servo_ReadAngle(uint8_t id, float *angleDeg, uint32_t timeoutMs); /* 读当前角度 */

/* ---- 通用寄存器读写 ---- */
void Servo_WriteData(uint8_t id, uint8_t addr, const uint8_t *data, uint8_t len);
int  Servo_ReadData(uint8_t id, uint8_t addr, uint8_t len,
                    uint8_t *data, uint32_t timeoutMs);

/* ---- 调试接口 ---- */
void Servo_RxFlush(void);
uint32_t Servo_RxAvailable(void);

#endif /* __SERVO_H */
