/**
  ******************************************************************************
  * @file    Servo.c
  * @brief   德晟串口总线智能舵机协议 v4.01 驱动（自诊断版）
  *
  * 支持三种总线接法（可运行中切换）：
  *   PA9_TX  ：双线模式，PA9 推挽发送 / PA10 输入接收（总线接在 PA9 或 PA9+PA10 短接）
  *   PA10_HD ：单线半双工，只用 PA10（USART 半双工模式）
  *   PA9_HD  ：单线半双工，只用 PA9（USART 半双工模式）
  * 波特率可在 115200 / 1000000 / 9600 间切换。
  * 所有运行状态写入 g_diag，供调试器在运行时读取。
  ******************************************************************************
  */

#include "Servo.h"

extern volatile uint32_t uwTick;   /* stm32f10x_it.c 中 SysTick 每 1ms 自增 */

/* 运行诊断结构（调试器可直接读 RAM 查看） */
volatile Diag_t g_diag;

/* ------------------------------ 接收环形缓冲 ------------------------------ */
#define SERVO_RX_BUF_SIZE   128
static volatile uint8_t  s_rxBuf[SERVO_RX_BUF_SIZE];
static volatile uint8_t  s_rxHead = 0;
static volatile uint8_t  s_rxTail = 0;

static uint8_t s_configured = 0;

/* ------------------------------ 底层发送 ---------------------------------- */
static void Servo_SendByte(uint8_t b)
{
    while ((USART1->SR & USART_SR_TXE) == 0)
    {
    }
    USART1->DR = b;
}

static void Servo_SendBytes(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        Servo_SendByte(buf[i]);
    }
    while ((USART1->SR & USART_SR_TC) == 0)   /* 发完释放总线（半双工时） */
    {
    }
    g_diag.txBytes += len;
}

/* ------------------------------ 接收接口 ---------------------------------- */
void Servo_RxFlush(void)
{
    s_rxTail = s_rxHead;
}

uint32_t Servo_RxAvailable(void)
{
    return (uint32_t)((uint8_t)(s_rxHead - s_rxTail)) & 0x7FU;
}

static uint8_t Servo_RxGet(void)
{
    uint8_t b = s_rxBuf[s_rxTail];
    uint8_t next = (uint8_t)(s_rxTail + 1);
    if (next == SERVO_RX_BUF_SIZE)
    {
        next = 0;
    }
    s_rxTail = next;
    return b;
}

/* ------------------------------ 总线配置 ---------------------------------- */
/**
  * @brief  配置总线接法和波特率（可运行中反复切换）
  * @param  pinSel  SERVO_PIN_PA9_TX / SERVO_PIN_PA10_HD / SERVO_PIN_PA9_HD
  * @param  baud    波特率
  */
void Servo_Config(uint8_t pinSel, uint32_t baud)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef  nvic;

    if (!s_configured)
    {
        /* 首次：1ms 滴答（供超时/延时） */
        if (SysTick_Config(SystemCoreClock / 1000U))
        {
            while (1)
            {
            }
        }
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
        s_configured = 1;
    }

    USART_Cmd(USART1, DISABLE);

    /* 引脚配置 */
    if (pinSel == SERVO_PIN_PA9_TX)
    {
        /* 双线：PA9 推挽 TX，PA10 浮空输入 RX */
        gpio.GPIO_Pin   = GPIO_Pin_9;
        gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
        gpio.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &gpio);

        gpio.GPIO_Pin   = GPIO_Pin_10;
        gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
        gpio.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &gpio);
    }
    else if (pinSel == SERVO_PIN_PA10_HD)
    {
        gpio.GPIO_Pin   = GPIO_Pin_10;
        gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
        gpio.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &gpio);

        gpio.GPIO_Pin   = GPIO_Pin_9;
        gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
        gpio.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &gpio);
    }
    else
    {
        gpio.GPIO_Pin   = GPIO_Pin_9;
        gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
        gpio.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &gpio);

        gpio.GPIO_Pin   = GPIO_Pin_10;
        gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
        gpio.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &gpio);
    }

    usart.USART_BaudRate            = baud;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &usart);

    /* 半双工模式开关 */
    if (pinSel == SERVO_PIN_PA10_HD || pinSel == SERVO_PIN_PA9_HD)
    {
        USART_HalfDuplexCmd(USART1, ENABLE);
    }
    else
    {
        USART_HalfDuplexCmd(USART1, DISABLE);
    }

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    nvic.NVIC_IRQChannel                   = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 1;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(USART1, ENABLE);

    (void)USART1->SR;
    (void)USART1->DR;
    Servo_RxFlush();
}

/* ------------------------------ 中断处理 ---------------------------------- */
void USART1_IRQHandler(void)
{
    if (USART1->SR & USART_SR_RXNE)
    {
        uint8_t b = (uint8_t)USART1->DR;
        uint8_t next = (uint8_t)(s_rxHead + 1);
        if (next == SERVO_RX_BUF_SIZE)
        {
            next = 0;
        }
        if (next != s_rxTail)
        {
            s_rxBuf[s_rxHead] = b;
            s_rxHead = next;
        }
        g_diag.rxBytes++;
        g_diag.lastRxByte = b;
    }
}

/* ------------------------------ 组帧发送 ---------------------------------- */
void Servo_SendFrame(uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t n)
{
    uint8_t frame[40];
    uint8_t len = (uint8_t)(n + 2);
    uint8_t sum;
    uint8_t i;

    frame[0] = 0xFF;
    frame[1] = 0xFF;
    frame[2] = id;
    frame[3] = len;
    frame[4] = cmd;

    sum = (uint8_t)(id + len + cmd);
    for (i = 0; i < n; i++)
    {
        frame[5 + i] = params[i];
        sum = (uint8_t)(sum + params[i]);
    }
    frame[5 + n] = (uint8_t)(~sum);

    Servo_SendBytes(frame, (uint16_t)(6 + n));
}

/* ------------------------------ 高级操作 ---------------------------------- */
void Servo_Move(uint8_t id, uint16_t position, uint16_t timeMs)
{
    uint8_t p[5];

    if (position > 4095U)
    {
        position = 4095U;
    }
    p[0] = SERVO_ADDR_TARGET_POS;
    p[1] = (uint8_t)(position >> 8);
    p[2] = (uint8_t)(position & 0xFFU);
    p[3] = (uint8_t)(timeMs >> 8);
    p[4] = (uint8_t)(timeMs & 0xFFU);

    Servo_SendFrame(id, SERVO_CMD_WRITE, p, 5);
}

/* ------------------------------ 角度控制 ---------------------------------- */
static uint16_t s_angleRange = 300;     /* 0~4095 满行程对应的角度（默认 300°） */

/**
  * @brief  设定满行程角度（位置 0~4095 对应 0~rangeDeg）
  *         常见串口舵机：300°（SCS 系列默认）、270°、180° 等，
  *         按舵机实际型号设置，设错只会导致角度换算比例不准。
  */
void Servo_SetAngleRange(uint16_t rangeDeg)
{
    if (rangeDeg > 0U && rangeDeg <= 360U)
    {
        s_angleRange = rangeDeg;
    }
}

/**
  * @brief  按角度控制舵机（0° ~ s_angleRange，超出自动钳位）
  * @param  id        舵机 ID
  * @param  angleDeg  目标角度（度，可小数）
  * @param  timeMs    运行时间 ms
  */
void Servo_MoveAngle(uint8_t id, float angleDeg, uint16_t timeMs)
{
    uint16_t position;

    if (angleDeg < 0.0f)
    {
        angleDeg = 0.0f;
    }
    if (angleDeg > (float)s_angleRange)
    {
        angleDeg = (float)s_angleRange;
    }
    position = (uint16_t)(angleDeg * 4095.0f / (float)s_angleRange + 0.5f);
    Servo_Move(id, position, timeMs);
}

/**
  * @brief  读当前角度（依赖 Servo_ReadPosition，舵机需支持应答）
  * @retval 1 = 成功；0 = 超时/校验错
  */
int Servo_ReadAngle(uint8_t id, float *angleDeg, uint32_t timeoutMs)
{
    uint16_t pos;
    if (Servo_ReadPosition(id, &pos, timeoutMs))
    {
        *angleDeg = (float)pos * (float)s_angleRange / 4095.0f;
        return 1;
    }
    return 0;
}

/**
  * @brief  德晟 v5 协议（F9 FF 帧头，小端）转动控制
  *         帧：F9 FF | ID | LEN(3+N) | 03 | 0x65 | 角度(0.1°, 有符号, 小端) | 时间(ms, 小端) | SUM
  *         SUM = ~(ID + LEN + CMD + ADDR + 参数...) & 0xFF
  */
void Servo_MoveV5(uint8_t id, int16_t angleDeciDeg, uint16_t timeMs)
{
    uint8_t frame[11];
    uint8_t sum;
    uint16_t angle = (uint16_t)angleDeciDeg;

    frame[0] = 0xF9;
    frame[1] = 0xFF;
    frame[2] = id;
    frame[3] = 7;                       /* LEN = ID+CMD+ADDR+4参数 */
    frame[4] = 0x03;                    /* 写指令 */
    frame[5] = 0x65;                    /* 角度定时控制地址 */
    frame[6] = (uint8_t)(angle & 0xFFU);        /* 角度低字节 */
    frame[7] = (uint8_t)(angle >> 8);           /* 角度高字节 */
    frame[8] = (uint8_t)(timeMs & 0xFFU);       /* 时间低字节 */
    frame[9] = (uint8_t)(timeMs >> 8);          /* 时间高字节 */

    sum = (uint8_t)(id + 7 + 0x03 + 0x65 +
                    frame[6] + frame[7] + frame[8] + frame[9]);
    frame[10] = (uint8_t)(~sum);

    Servo_SendBytes(frame, 11);
}

/**
  * @brief  把总线上所有舵机的 ID 设为 id（广播写寄存器 0x05）
  *         例（设 ID=1）：FF FF FE 04 03 05 01 F4（与协议文档示例一致）
  *         注意：多舵机总线上用广播会同时改所有舵机的 ID，
  *         只适合单舵机接线时使用。改完掉电保存。
  */
void Servo_SetID(uint8_t id)
{
    uint8_t p[2];
    p[0] = 0x05;                         /* 寄存器 0x05 = 舵机 ID */
    p[1] = id;
    Servo_SendFrame(SERVO_BROADCAST_ID, SERVO_CMD_WRITE, p, 2);
}

/**
  * @brief  通用寄存器写（WRITE DATA）
  * @param  id    舵机 ID（1~250 或广播 254）
  * @param  addr  寄存器首地址
  * @param  data  数据（大端，按协议地址表）
  * @param  len   数据字节数
  */
void Servo_WriteData(uint8_t id, uint8_t addr, const uint8_t *data, uint8_t len)
{
    uint8_t p[16];
    uint8_t i;

    p[0] = addr;
    for (i = 0; i < len && i < 15; i++)
    {
        p[1 + i] = data[i];
    }
    Servo_SendFrame(id, SERVO_CMD_WRITE, p, (uint8_t)(1 + i));
}

/**
  * @brief  通用寄存器读（READ DATA）
  * @param  id     舵机 ID
  * @param  addr   寄存器首地址
  * @param  len    要读的字节数
  * @param  data   输出缓冲（至少 len 字节）
  * @param  timeoutMs 超时
  * @retval 1 = 成功；0 = 超时/校验错/ID 不符
  */
int Servo_ReadData(uint8_t id, uint8_t addr, uint8_t len,
                   uint8_t *data, uint32_t timeoutMs)
{
    uint8_t  p[2];
    uint8_t  state = 0;
    uint8_t  cnt = 0;
    uint8_t  buf[16];
    uint8_t  expect;                     /* 应答体长度 = ID+LEN+状态+数据 = 2+len */
    uint32_t deadline = uwTick + timeoutMs;

    p[0] = addr;
    p[1] = len;
    expect = (uint8_t)(2 + len);

    Servo_RxFlush();
    Servo_SendFrame(id, SERVO_CMD_READ, p, 2);

    while ((int32_t)(uwTick - deadline) < 0)
    {
        while (Servo_RxAvailable() > 0)
        {
            uint8_t b = Servo_RxGet();
            switch (state)
            {
                case 0:
                    if (b == 0xFF)
                    {
                        state = 1;
                    }
                    break;

                case 1:
                    if (b == 0xF5)
                    {
                        state = 2;
                        cnt = 0;
                    }
                    else if (b != 0xFF)
                    {
                        state = 0;
                    }
                    break;

                default:
                    buf[cnt++] = b;
                    if (cnt == expect)
                    {
                        uint8_t i;
                        uint8_t sum = 0;
                        for (i = 0; i < (uint8_t)(expect - 1); i++)
                        {
                            sum = (uint8_t)(sum + buf[i]);
                        }
                        if (buf[0] == id && buf[1] == expect &&
                            buf[expect - 1] == (uint8_t)(~sum))
                        {
                            for (i = 0; i < len; i++)
                            {
                                data[i] = buf[2 + i];   /* 跳过 ID/状态 */
                            }
                            return 1;
                        }
                        state = 0;
                        cnt = 0;
                    }
                    break;
            }
        }
    }
    return 0;
}

void Servo_MoveSync(const uint8_t *ids, const uint16_t *positions,
                    const uint16_t *times, uint8_t n)
{
    uint8_t i;
    uint8_t p[5];

    p[0] = SERVO_ADDR_TARGET_POS;
    for (i = 0; i < n; i++)
    {
        uint16_t pos = positions[i];
        if (pos > 4095U)
        {
            pos = 4095U;
        }
        p[1] = (uint8_t)(pos >> 8);
        p[2] = (uint8_t)(pos & 0xFFU);
        p[3] = (uint8_t)(times[i] >> 8);
        p[4] = (uint8_t)(times[i] & 0xFFU);
        Servo_SendFrame(ids[i], SERVO_CMD_REG_WRITE, p, 5);
    }
    Servo_SendFrame(SERVO_BROADCAST_ID, SERVO_CMD_ACTION, 0, 0);
}

/* ==================== 机械臂用：多舵机同步控制 ============================ */

/**
  * @brief  异步写（REG WRITE 0x04）：先预写目标位置+运行时间，等 ACTION 才动
  *         协议 2.3.4
  */
void Servo_RegWritePosTime(uint8_t id, uint16_t position, uint16_t timeMs)
{
    uint8_t p[5];

    if (position > 4095U)
    {
        position = 4095U;
    }
    p[0] = SERVO_ADDR_TARGET_POS;
    p[1] = (uint8_t)(position >> 8);
    p[2] = (uint8_t)(position & 0xFFU);
    p[3] = (uint8_t)(timeMs >> 8);
    p[4] = (uint8_t)(timeMs & 0xFFU);

    Servo_SendFrame(id, SERVO_CMD_REG_WRITE, p, 5);
}

/**
  * @brief  广播执行所有异步预写（ACTION 0x05），让多个舵机严格同时起步
  *         协议 2.3.5：FF FF FE 02 05 FA
  */
void Servo_Action(void)
{
    Servo_SendFrame(SERVO_BROADCAST_ID, SERVO_CMD_ACTION, 0, 0);
}

/**
  * @brief  同步写（SYNC WRITE 0x83）：一帧里给 n 个舵机同时写目标位置+运行时间
  *         协议 2.3.6，只支持地址 0x2A 起 4 字节（目标位置2 + 运行时间2）
  *         帧：FF FF FE | LEN | 83 | 2A | 04 | (ID 位置H 位置L 时间H 时间L)*n | SUM
  *         LEN = ID + CMD + 地址 + 数据长度 + n*5 = 4 + 5n
  *         SUM = ~(ID + LEN + CMD + 地址 + 数据长度 + 所有数据) & 0xFF
  * @param  n      舵机个数（1~8）
  * @param  ids    舵机 ID 数组
  * @param  positions 目标位置数组（0~4095）
  * @param  times  运行时间数组（ms）
  */
void Servo_SyncWritePosTime(uint8_t n, const uint8_t *ids,
                            const uint16_t *positions, const uint16_t *times)
{
    uint8_t frame[8 + (5 * 8) + 1];
    uint8_t len;
    uint8_t idx;
    uint8_t i;
    uint8_t sum;

    if ((n == 0U) || (n > 8U))
    {
        return;
    }

    len = (uint8_t)(4U + (5U * n));

    idx = 0;
    frame[idx++] = 0xFF;
    frame[idx++] = 0xFF;
    frame[idx++] = SERVO_BROADCAST_ID;
    frame[idx++] = len;
    frame[idx++] = SERVO_CMD_SYNC_WRITE;
    frame[idx++] = SERVO_ADDR_TARGET_POS;
    frame[idx++] = 0x04U;

    sum = (uint8_t)(SERVO_BROADCAST_ID + len + SERVO_CMD_SYNC_WRITE +
                    SERVO_ADDR_TARGET_POS + 0x04U);

    for (i = 0; i < n; i++)
    {
        uint16_t pos = positions[i];
        uint8_t  ph;
        uint8_t  pl;
        uint8_t  th;
        uint8_t  tl;

        if (pos > 4095U)
        {
            pos = 4095U;
        }
        ph = (uint8_t)(pos >> 8);
        pl = (uint8_t)(pos & 0xFFU);
        th = (uint8_t)(times[i] >> 8);
        tl = (uint8_t)(times[i] & 0xFFU);

        frame[idx++] = ids[i];
        frame[idx++] = ph;
        frame[idx++] = pl;
        frame[idx++] = th;
        frame[idx++] = tl;

        sum = (uint8_t)(sum + ids[i] + ph + pl + th + tl);
    }

    frame[idx++] = (uint8_t)(~sum);

    Servo_SendBytes(frame, (uint16_t)idx);
    g_diag.syncWrites++;
}

/**
  * @brief  等待一帧舵机应答（FF F5 | ID | LEN | 状态 | 参数... | SUM）
  *         应答帧 LEN = 状态(1) + 参数(N) + 校验(1)
  * @retval 1 = 收到合法应答（且 ID 匹配时才算）；0 = 超时/校验错
  */
int Servo_WaitAck(uint8_t id, uint32_t timeoutMs)
{
    uint8_t  state = 0;
    uint8_t  cnt = 0;
    uint8_t  need = 0;
    uint8_t  buf[32];
    uint32_t deadline = uwTick + timeoutMs;

    while ((int32_t)(uwTick - deadline) < 0)
    {
        while (Servo_RxAvailable() > 0)
        {
            uint8_t b = Servo_RxGet();

            switch (state)
            {
                case 0:
                    if (b == 0xFF)
                    {
                        state = 1;
                    }
                    break;

                case 1:
                    if (b == 0xF5)
                    {
                        state = 2;
                        cnt = 0;
                    }
                    else if (b != 0xFF)
                    {
                        state = 0;
                    }
                    break;

                case 2:                 /* ID */
                    buf[cnt++] = b;
                    state = 3;
                    break;

                case 3:                 /* LEN */
                    buf[cnt++] = b;
                    need = b;
                    if ((need < 2U) || (need > 30U))
                    {
                        state = 0;
                        cnt = 0;
                    }
                    else
                    {
                        state = 4;
                    }
                    break;

                default:                /* 状态 + 参数 + SUM */
                    buf[cnt++] = b;
                    if (cnt >= (uint8_t)(need + 2U))
                    {
                        uint8_t i;
                        uint8_t sum = 0;
                        for (i = 0; i < (uint8_t)(cnt - 1U); i++)
                        {
                            sum = (uint8_t)(sum + buf[i]);
                        }
                        if ((buf[0] == id) &&
                            (buf[cnt - 1U] == (uint8_t)(~sum)))
                        {
                            return 1;
                        }
                        state = 0;
                        cnt = 0;
                    }
                    break;
            }
        }
    }
    return 0;
}

/**
  * @brief  转动到指定位置并等待舵机应答（逐帧确认，最保守的用法）
  * @retval 1 = 舵机已确认；0 = 超时（舵机不在线/总线接法不对）
  */
int Servo_MoveWaitAck(uint8_t id, uint16_t position, uint16_t timeMs)
{
    int ok;

    Servo_RxFlush();
    Servo_Move(id, position, timeMs);
    ok = Servo_WaitAck(id, 10U);
    if (ok == 0)
    {
        g_diag.ackFail++;
    }
    return ok;
}


/**
  * @brief  PING 一个舵机（查询指令 0x01），应答 FF F5 id 02 状态 SUM
  * @retval 1 = 该 ID 存在；0 = 无应答/校验错
  */
int Servo_Ping(uint8_t id, uint32_t timeoutMs)
{
    uint8_t  state = 0;
    uint8_t  cnt = 0;
    uint8_t  buf[4];         /* ID LEN 状态 SUM */
    uint32_t deadline = uwTick + timeoutMs;

    Servo_RxFlush();
    Servo_SendFrame(id, SERVO_CMD_PING, 0, 0);

    while ((int32_t)(uwTick - deadline) < 0)
    {
        while (Servo_RxAvailable() > 0)
        {
            uint8_t b = Servo_RxGet();
            switch (state)
            {
                case 0:
                    if (b == 0xFF)
                    {
                        state = 1;
                    }
                    break;

                case 1:
                    if (b == 0xF5)
                    {
                        state = 2;
                        cnt = 0;
                    }
                    else if (b != 0xFF)
                    {
                        state = 0;
                    }
                    break;

                default:
                    buf[cnt++] = b;
                    if (cnt == 4)
                    {
                        uint8_t sum = (uint8_t)(buf[0] + buf[1] + buf[2]);
                        if (buf[0] == id && buf[1] == 0x02 &&
                            buf[3] == (uint8_t)(~sum))
                        {
                            return 1;
                        }
                        state = 0;
                        cnt = 0;
                    }
                    break;
            }
        }
    }
    return 0;
}

/**
  * @brief  读取舵机当前位置（READ DATA @0x38，2 字节）
  * @retval 1 = 成功；0 = 超时/校验错/ID 不符
  */
int Servo_ReadPosition(uint8_t id, uint16_t *pos, uint32_t timeoutMs)
{
    uint8_t  p[2];
    uint8_t  state = 0;
    uint8_t  cnt = 0;
    uint8_t  buf[6];
    uint32_t deadline = uwTick + timeoutMs;

    p[0] = SERVO_ADDR_CUR_POS;
    p[1] = 0x02;

    Servo_RxFlush();
    Servo_SendFrame(id, SERVO_CMD_READ, p, 2);

    while ((int32_t)(uwTick - deadline) < 0)
    {
        while (Servo_RxAvailable() > 0)
        {
            uint8_t b = Servo_RxGet();
            switch (state)
            {
                case 0:
                    if (b == 0xFF)
                    {
                        state = 1;
                    }
                    break;

                case 1:
                    if (b == 0xF5)
                    {
                        state = 2;
                        cnt = 0;
                    }
                    else if (b != 0xFF)
                    {
                        state = 0;
                    }
                    break;

                default:
                    buf[cnt++] = b;
                    if (cnt == 6)
                    {
                        uint8_t sum = (uint8_t)(buf[0] + buf[1] + buf[2] +
                                                buf[3] + buf[4]);
                        if (buf[0] == id && buf[1] == 0x04 &&
                            buf[5] == (uint8_t)(~sum))
                        {
                            *pos = (uint16_t)((buf[3] << 8) | buf[4]);
                            return 1;
                        }
                        state = 0;
                        cnt = 0;
                    }
                    break;
            }
        }
    }
    return 0;
}
