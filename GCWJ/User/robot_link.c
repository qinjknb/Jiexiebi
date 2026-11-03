/**
  ******************************************************************************
  * @file    robot_link.c
  * @brief   电脑通信层实现：USART2 中断收发 + 二进制协议 + 文本命令兼容
  *
  *  - 接收：环形缓冲，主循环里解析（不在中断里做重活）
  *  - 发送：环形缓冲 + TXE 中断，不阻塞控制节拍
  *  - 协议：见 robot_link.h
  ******************************************************************************
  */

#include "robot_link.h"
#include "arm_control.h"
#include "Servo.h"

extern volatile uint32_t uwTick;

/* =============================== 内部状态 ================================= */
static volatile uint8_t  s_rx[LINK_RX_BUF_SIZE];
static volatile uint16_t s_rxHead = 0;
static volatile uint16_t s_rxTail = 0;

static volatile uint8_t  s_tx[LINK_TX_BUF_SIZE];
static volatile uint16_t s_txHead = 0;
static volatile uint16_t s_txTail = 0;

static uint8_t  s_txSeq = 0;

/* 接收模式：0=未定 1=二进制  2=文本 */
static uint8_t  s_rxMode = 0;
static uint32_t s_lastByteTick = 0;

/* 二进制拆帧状态 */
static uint8_t  s_binState = 0;
static uint8_t  s_binCmd = 0;
static uint8_t  s_binSeq = 0;
static uint8_t  s_binLen = 0;
static uint8_t  s_binIdx = 0;
static uint8_t  s_binPayload[LINK_MAX_PAYLOAD];
static uint8_t  s_binCrcHi = 0;

/* 文本行缓冲 */
static char     s_text[LINK_TEXT_LINE_MAX];
static uint8_t  s_textLen = 0;

/* 状态上报 */
static uint8_t  s_telemHz = ARM_TELEM_RATE_HZ;
static uint32_t s_lastTelemTick = 0;

/* =============================== 环形缓冲 ================================= */
static uint8_t Link_RxGet(uint8_t *b)
{
    if (s_rxTail == s_rxHead)
    {
        return 0;
    }
    *b = s_rx[s_rxTail];
    s_rxTail++;
    if (s_rxTail >= LINK_RX_BUF_SIZE)
    {
        s_rxTail = 0;
    }
    return 1;
}

static void Link_TxStart(void)
{
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}

static void Link_PutTx(uint8_t b)
{
    uint16_t next = (uint16_t)(s_txHead + 1U);

    if (next >= LINK_TX_BUF_SIZE)
    {
        next = 0;
    }
    if (next == s_txTail)
    {
        return;                     /* 缓冲满：丢弃（不阻塞控制节拍） */
    }
    s_tx[s_txHead] = b;
    s_txHead = next;
    Link_TxStart();
}

/* =============================== 中断 ===================================== */
void USART2_IRQHandler(void)
{
    uint16_t sr = USART2->SR;

    if ((sr & USART_SR_RXNE) != 0)
    {
        uint8_t b = (uint8_t)USART2->DR;
        uint16_t next = (uint16_t)(s_rxHead + 1U);

        if (next >= LINK_RX_BUF_SIZE)
        {
            next = 0;
        }
        if (next != s_rxTail)       /* 满了就丢新字节 */
        {
            s_rx[s_rxHead] = b;
            s_rxHead = next;
        }
    }

    if ((sr & USART_SR_TXE) != 0)
    {
        if ((USART2->CR1 & USART_CR1_TXEIE) != 0)
        {
            if (s_txTail != s_txHead)
            {
                USART2->DR = s_tx[s_txTail];
                s_txTail++;
                if (s_txTail >= LINK_TX_BUF_SIZE)
                {
                    s_txTail = 0;
                }
            }
            else
            {
                USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
            }
        }
    }
}

/* =============================== CRC16 =================================== */
static uint16_t Link_Crc16Update(uint16_t crc, uint8_t b)
{
    uint8_t i;

    crc ^= (uint16_t)((uint16_t)b << 8);
    for (i = 0; i < 8U; i++)
    {
        if ((crc & 0x8000U) != 0U)
        {
            crc = (uint16_t)((uint16_t)(crc << 1) ^ 0x1021U);
        }
        else
        {
            crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint16_t Link_Crc16(const uint8_t *p, uint16_t n)
{
    uint16_t crc = 0xFFFFU;

    while (n > 0U)
    {
        crc = Link_Crc16Update(crc, *p);
        p++;
        n--;
    }
    return crc;
}

/* =============================== 发送帧 =================================== */
static void Link_SendFrame(uint8_t cmd, uint8_t seq, const uint8_t *payload, uint8_t len)
{
    uint16_t crc = 0xFFFFU;
    uint8_t  i;

    Link_PutTx(LINK_SOH0);
    Link_PutTx(LINK_SOH1);

    crc = Link_Crc16Update(crc, cmd);
    Link_PutTx(cmd);
    crc = Link_Crc16Update(crc, seq);
    Link_PutTx(seq);
    crc = Link_Crc16Update(crc, len);
    Link_PutTx(len);

    for (i = 0; i < len; i++)
    {
        crc = Link_Crc16Update(crc, payload[i]);
        Link_PutTx(payload[i]);
    }

    Link_PutTx((uint8_t)(crc >> 8));
    Link_PutTx((uint8_t)(crc & 0xFFU));
}

void Link_SendPong(uint8_t seq)
{
    uint8_t p[4];

    p[0] = LINK_FW_VER_MAJOR;
    p[1] = LINK_FW_VER_MINOR;
    p[2] = (uint8_t)ARM_JOINT_NUM;
    p[3] = Arm_GetBusMode();
    Link_SendFrame(LINK_RSP_PONG, seq, p, 4);
}

void Link_SendAck(uint8_t seq, uint8_t cmd, uint8_t result)
{
    uint8_t p[2];

    p[0] = cmd;
    p[1] = result;
    Link_SendFrame(LINK_RSP_ACK, seq, p, 2);
}

void Link_SendState(void)
{
    /* 负载 = flags(1) + mode(1) + cmd(6*2) + meas(6*2) = 2 + 4*6 = 26 字节。
       注意：这里曾经误写成 2 + 2*6 = 14 字节，越界写 12 字节踩坏栈，
       导致上电发完问候 40ms 后 HardFault。下面这行编译期检查防止再犯。 */
    uint8_t  p[2U + (4U * ARM_JOINT_NUM)];
    typedef char link_state_buf_size_check[
        (sizeof(p) >= (2U + (4U * ARM_JOINT_NUM))) ? 1 : -1];
    uint16_t cmd[ARM_JOINT_NUM];
    uint16_t meas[ARM_JOINT_NUM];
    uint8_t  i;
    uint8_t  k = 0;

    Arm_GetCmd(cmd);
    Arm_GetMeasured(meas);

    p[k++] = Arm_GetFlags();
    p[k++] = Arm_GetMode();
    for (i = 0; i < ARM_JOINT_NUM; i++)
    {
        p[k++] = (uint8_t)(cmd[i] >> 8);
        p[k++] = (uint8_t)(cmd[i] & 0xFFU);
    }
    for (i = 0; i < ARM_JOINT_NUM; i++)
    {
        p[k++] = (uint8_t)(meas[i] >> 8);
        p[k++] = (uint8_t)(meas[i] & 0xFFU);
    }

    s_txSeq++;
    Link_SendFrame(LINK_RSP_STATE, s_txSeq, p, k);
}

void Link_SendPos(void)
{
    uint8_t  p[2U * ARM_JOINT_NUM];
    uint16_t meas[ARM_JOINT_NUM];
    uint8_t  i;
    uint8_t  k = 0;

    Arm_GetMeasured(meas);
    for (i = 0; i < ARM_JOINT_NUM; i++)
    {
        p[k++] = (uint8_t)(meas[i] >> 8);
        p[k++] = (uint8_t)(meas[i] & 0xFFU);
    }
    s_txSeq++;
    Link_SendFrame(LINK_RSP_POS, s_txSeq, p, k);
}

void Link_SendText(const char *s)
{
    while (*s != 0)
    {
        Link_PutTx((uint8_t)(*s));
        s++;
    }
}

/* ============================ 二进制命令处理 ============================== */
static uint16_t Link_GetU16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static void Link_HandleBinary(uint8_t cmd, uint8_t seq, uint8_t len, const uint8_t *data)
{
    uint8_t  i;
    uint8_t  result = LINK_RES_OK;
    uint16_t tmp[ARM_JOINT_NUM];
    uint16_t mn[ARM_JOINT_NUM];
    uint16_t mx[ARM_JOINT_NUM];

    switch (cmd)
    {
        case LINK_CMD_PING:
            Link_SendPong(seq);
            return;

        case LINK_CMD_SET_TARGETS:
            if (len != (uint8_t)(2U * ARM_JOINT_NUM))
            {
                result = LINK_RES_BAD_LEN;
                break;
            }
            for (i = 0; i < ARM_JOINT_NUM; i++)
            {
                tmp[i] = Link_GetU16(&data[2U * i]);
            }
            Arm_SetTargets(tmp);
            break;

        case LINK_CMD_SET_MODE:
            if (len != 1U)
            {
                result = LINK_RES_BAD_LEN;
                break;
            }
            if (data[0] > ARM_MODE_HOME)
            {
                result = LINK_RES_RANGE;
                break;
            }
            Arm_SetMode(data[0]);
            break;

        case LINK_CMD_HOME:
            if (len != 2U)
            {
                result = LINK_RES_BAD_LEN;
                break;
            }
            Arm_Home(Link_GetU16(data));
            break;

        case LINK_CMD_SET_LIMITS:
            if (len != (uint8_t)(4U * ARM_JOINT_NUM))
            {
                result = LINK_RES_BAD_LEN;
                break;
            }
            for (i = 0; i < ARM_JOINT_NUM; i++)
            {
                mn[i] = Link_GetU16(&data[2U * i]);
                mx[i] = Link_GetU16(&data[(2U * ARM_JOINT_NUM) + (2U * i)]);
            }
            Arm_SetLimits(mn, mx);
            break;

        case LINK_CMD_READ_POS:
            Link_SendPos();
            return;

        case LINK_CMD_SET_TELEM:
            if (len != 1U)
            {
                result = LINK_RES_BAD_LEN;
                break;
            }
            s_telemHz = (data[0] > 100U) ? 100U : data[0];
            break;

        case LINK_CMD_STOP:
            Arm_Stop();
            break;

        case LINK_CMD_GET_STATUS:
            Link_SendState();
            return;

        case LINK_CMD_SET_BUSMODE:
            if (len != 1U)
            {
                result = LINK_RES_BAD_LEN;
                break;
            }
            if (Arm_SetBusMode(data[0]) == 0U)
            {
                result = LINK_RES_RANGE;
                break;
            }
            break;

        default:
            Link_SendAck(seq, cmd, LINK_RES_UNKNOWN_CMD);
            return;
    }

    Link_SendAck(seq, cmd, result);
}

/* 二进制拆帧：返回 1 表示消费了该字节并处理完了一帧 */
static void Link_BinByte(uint8_t b)
{
    switch (s_binState)
    {
        case 0:
            if (b == LINK_SOH0)
            {
                s_binState = 1;
            }
            break;

        case 1:
            if (b == LINK_SOH1)
            {
                s_binState = 2;
            }
            else if (b == LINK_SOH0)
            {
                s_binState = 1;
            }
            else
            {
                s_binState = 0;
            }
            break;

        case 2:
            s_binCmd = b;
            s_binState = 3;
            break;

        case 3:
            s_binSeq = b;
            s_binState = 4;
            break;

        case 4:
            s_binLen = b;
            s_binIdx = 0;
            if (b > LINK_MAX_PAYLOAD)
            {
                s_binState = 0;
            }
            else
            {
                s_binState = (b == 0U) ? 6U : 5U;
            }
            break;

        case 5:
            s_binPayload[s_binIdx] = b;
            s_binIdx++;
            if (s_binIdx >= s_binLen)
            {
                s_binState = 6;
            }
            break;

        case 6:
            s_binCrcHi = b;
            s_binState = 7;
            break;

        default:                    /* case 7: 收到 CRC 低字节，校验并处理 */
        {
            uint16_t crc = 0xFFFFU;
            uint8_t  i;

            crc = Link_Crc16Update(crc, s_binCmd);
            crc = Link_Crc16Update(crc, s_binSeq);
            crc = Link_Crc16Update(crc, s_binLen);
            for (i = 0; i < s_binLen; i++)
            {
                crc = Link_Crc16Update(crc, s_binPayload[i]);
            }

            if (((uint8_t)(crc >> 8) == s_binCrcHi) &&
                ((uint8_t)(crc & 0xFFU) == b))
            {
                Link_HandleBinary(s_binCmd, s_binSeq, s_binLen, s_binPayload);
            }
            s_binState = 0;
            break;
        }
    }
}

/* ============================== 文本命令 ================================= */
static uint8_t Link_StrEqCI(const char *a, const char *b)
{
    while ((*a != 0) && (*b != 0))
    {
        char ca = *a;
        char cb = *b;

        if ((ca >= 'a') && (ca <= 'z'))
        {
            ca = (char)(ca - 32);
        }
        if ((cb >= 'a') && (cb <= 'z'))
        {
            cb = (char)(cb - 32);
        }
        if (ca != cb)
        {
            return 0;
        }
        a++;
        b++;
    }
    return (uint8_t)((*a == 0) && (*b == 0));
}

static uint8_t Link_StrToU16(const char *s, uint16_t *out)
{
    uint32_t v = 0;
    uint8_t  digits = 0;

    while (*s != 0)
    {
        if ((*s >= '0') && (*s <= '9'))
        {
            v = (v * 10UL) + (uint32_t)(*s - '0');
            if (v > 65535UL)
            {
                return 0;
            }
            digits = 1;
        }
        else
        {
            return 0;
        }
        s++;
    }
    if (digits == 0U)
    {
        return 0;
    }
    *out = (uint16_t)v;
    return 1;
}

static uint8_t Link_StrToFloat(const char *s, float *out)
{
    float   v = 0.0f;
    float   frac = 0.1f;
    float   sign = 1.0f;
    uint8_t digits = 0;
    uint8_t sawDot = 0;

    if (*s == '-')
    {
        sign = -1.0f;
        s++;
    }
    else if (*s == '+')
    {
        s++;
    }
    while (*s != 0)
    {
        char c = *s;

        if (c == '.')
        {
            if (sawDot != 0U)
            {
                return 0;
            }
            sawDot = 1;
        }
        else if ((c >= '0') && (c <= '9'))
        {
            digits = 1;
            if (sawDot != 0U)
            {
                v = v + ((float)(c - '0') * frac);
                frac = frac * 0.1f;
            }
            else
            {
                v = (v * 10.0f) + (float)(c - '0');
            }
        }
        else
        {
            return 0;
        }
        s++;
    }
    if (digits == 0U)
    {
        return 0;
    }
    *out = sign * v;
    return 1;
}

static uint8_t Link_Tokenize(char *line, char *tok[], uint8_t maxTok)
{
    uint8_t n = 0;
    char   *p = line;

    while ((*p != 0) && (n < maxTok))
    {
        while ((*p == ' ') || (*p == '\t'))
        {
            p++;
        }
        if (*p == 0)
        {
            break;
        }
        tok[n++] = p;
        while ((*p != 0) && (*p != ' ') && (*p != '\t'))
        {
            p++;
        }
        if (*p != 0)
        {
            *p = 0;
            p++;
        }
    }
    return n;
}

static void Link_SendNumU16(uint16_t v)
{
    char    tmp[6];
    uint8_t i = 0;

    if (v == 0U)
    {
        Link_PutTx('0');
        return;
    }
    while (v > 0U)
    {
        tmp[i] = (char)('0' + (v % 10U));
        i++;
        v = (uint16_t)(v / 10U);
    }
    while (i > 0U)
    {
        i--;
        Link_PutTx((uint8_t)tmp[i]);
    }
}

static void Link_TextHelp(void)
{
    Link_SendText("HELP: PING id | MOVE id deg ms | MOVEP id pos ms | READ id\r\n");
    Link_SendText("      SETID id | RANGE deg | RESET id | HOME [ms] | JOINTS\r\n");
    Link_SendText("      BUSMODE n | WIRING 0|1|2 | TELEM hz | STREAM 0|1 | HELP\r\n");
}

/* 直接把某个舵机转到一个位置（调试/单一舵机用，与机械臂跟随互斥） */
static void Link_TextServoCommand(char *tok[], uint8_t n)
{
    if (n == 0U)
    {
        return;
    }

    if (Link_StrEqCI(tok[0], "HELP") != 0U)
    {
        Link_TextHelp();
        return;
    }

    if (Link_StrEqCI(tok[0], "JOINTS") != 0U)
    {
        uint16_t meas[ARM_JOINT_NUM];
        uint8_t  i;
        char     buf[8];

        Arm_GetMeasured(meas);
        Link_SendText("JOINTS");
        for (i = 0; i < ARM_JOINT_NUM; i++)
        {
            uint8_t k = 0;
            uint16_t v = meas[i];

            Link_PutTx(' ');
            if (v == 0xFFFFU)
            {
                Link_SendText("?");
                continue;
            }
            if (v == 0U)
            {
                Link_PutTx('0');
                continue;
            }
            while ((v > 0U) && (k < 7U))
            {
                buf[k] = (char)('0' + (v % 10U));
                k++;
                v = (uint16_t)(v / 10U);
            }
            while (k > 0U)
            {
                k--;
                Link_PutTx((uint8_t)buf[k]);
            }
        }
        Link_SendText("\r\n");
        return;
    }

    if (Link_StrEqCI(tok[0], "HOME") != 0U)
    {
        uint16_t ms = 2000U;

        if ((n >= 2U) && (Link_StrToU16(tok[1], &ms) == 0U))
        {
            Link_SendText("ERR BAD ARGS\r\n");
            return;
        }
        Arm_Home(ms);
        Link_SendText("OK HOME\r\n");
        return;
    }

    if (Link_StrEqCI(tok[0], "BUSMODE") != 0U)
    {
        uint16_t m;

        if ((n < 2U) || (Link_StrToU16(tok[1], &m) == 0U) || (m > 2U))
        {
            Link_SendText("ERR BAD ARGS\r\n");
            return;
        }
        (void)Arm_SetBusMode((uint8_t)m);
        Link_SendText("OK BUSMODE ");
        Link_SendNumU16(m);
        Link_SendText("\r\n");
        return;
    }

    if (Link_StrEqCI(tok[0], "WIRING") != 0U)
    {
        uint16_t w;

        if ((n < 2U) || (Link_StrToU16(tok[1], &w) == 0U) || (w > 2U))
        {
            Link_SendText("ERR BAD ARGS\r\n");
            return;
        }
        Servo_Config((uint8_t)w, ARM_SERVO_BAUD);   /* 0=双线 PA9/PA10 1=单线PA10 2=单线PA9 */
        Link_SendText("OK WIRING ");
        Link_SendNumU16(w);
        Link_SendText("\r\n");
        return;
    }

    if (Link_StrEqCI(tok[0], "TELEM") != 0U)
    {
        uint16_t hz;

        if ((n < 2U) || (Link_StrToU16(tok[1], &hz) == 0U) || (hz > 100U))
        {
            Link_SendText("ERR BAD ARGS\r\n");
            return;
        }
        s_telemHz = (uint8_t)hz;
        Link_SendText("OK TELEM ");
        Link_SendNumU16(hz);
        Link_SendText("\r\n");
        return;
    }

    if (Link_StrEqCI(tok[0], "STREAM") != 0U)
    {
        uint16_t m;

        if ((n < 2U) || (Link_StrToU16(tok[1], &m) == 0U) || (m > ARM_MODE_HOME))
        {
            Link_SendText("ERR BAD ARGS\r\n");
            return;
        }
        Arm_SetMode((uint8_t)m);
        Link_SendText("OK STREAM ");
        Link_SendNumU16(m);
        Link_SendText("\r\n");
        return;
    }

    if (Link_StrEqCI(tok[0], "DEMO") != 0U)
    {
        Link_SendText("OK DEMO\r\n");   /* 新固件上电不做演示摆动 */
        return;
    }

    /* 其余是直接操作单个舵机的调试命令：跟随模式下拒绝，避免打架 */
    if (Arm_GetMode() == ARM_MODE_STREAM)
    {
        Link_SendText("ERR BUSY STREAM\r\n");
        return;
    }

    if (Link_StrEqCI(tok[0], "PING") != 0U)
    {
        uint16_t id;

        if ((n < 2U) || (Link_StrToU16(tok[1], &id) == 0U) || (id == 0U) || (id > 250U))
        {
            Link_SendText("ERR BAD ARGS\r\n");
            return;
        }
        if (Servo_Ping((uint8_t)id, 100U) != 0)
        {
            Link_SendText("OK PING ");
            Link_SendNumU16(id);
            Link_SendText("\r\n");
        }
        else
        {
            Link_SendText("ERR PING ");
            Link_SendNumU16(id);
            Link_SendText("\r\n");
        }
        return;
    }

    if (Link_StrEqCI(tok[0], "MOVE") != 0U)
    {
        uint16_t id;
        uint16_t ms;
        float    deg;

        if ((n < 4U) || (Link_StrToU16(tok[1], &id) == 0U) || (id == 0U) || (id > 250U) ||
            (Link_StrToFloat(tok[2], &deg) == 0U) || (Link_StrToU16(tok[3], &ms) == 0U))
        {
            Link_SendText("ERR BAD ARGS\r\n");
            return;
        }
        Servo_MoveAngle((uint8_t)id, deg, ms);
        Link_SendText("OK MOVE ");
        Link_SendNumU16(id);
        Link_SendText("\r\n");
        return;
    }

    if (Link_StrEqCI(tok[0], "MOVEP") != 0U)
    {
        uint16_t id;
        uint16_t pos;
        uint16_t ms;

        if ((n < 4U) || (Link_StrToU16(tok[1], &id) == 0U) || (id == 0U) || (id > 250U) ||
            (Link_StrToU16(tok[2], &pos) == 0U) || (Link_StrToU16(tok[3], &ms) == 0U))
        {
            Link_SendText("ERR BAD ARGS\r\n");
            return;
        }
        Servo_Move((uint8_t)id, pos, ms);
        Link_SendText("OK MOVEP ");
        Link_SendNumU16(id);
        Link_SendText("\r\n");
        return;
    }

    if (Link_StrEqCI(tok[0], "READ") != 0U)
    {
        uint16_t id;
        uint16_t pos;

        if ((n < 2U) || (Link_StrToU16(tok[1], &id) == 0U) || (id == 0U) || (id > 250U))
        {
            Link_SendText("ERR BAD ARGS\r\n");
            return;
        }
        if (Servo_ReadPosition((uint8_t)id, &pos, 50U) != 0)
        {
            Link_SendText("POS ");
            Link_SendNumU16(id);
            Link_SendText(" ");
            Link_SendNumU16(pos);
            Link_SendText("\r\n");
        }
        else
        {
            Link_SendText("ERR READ ");
            Link_SendNumU16(id);
            Link_SendText("\r\n");
        }
        return;
    }

    if (Link_StrEqCI(tok[0], "SETID") != 0U)
    {
        uint16_t id;

        if ((n < 2U) || (Link_StrToU16(tok[1], &id) == 0U) || (id == 0U) || (id > 250U))
        {
            Link_SendText("ERR BAD ARGS\r\n");
            return;
        }
        Servo_SetID((uint8_t)id);
        Link_SendText("OK SETID ");
        Link_SendNumU16(id);
        Link_SendText("\r\n");
        return;
    }

    if (Link_StrEqCI(tok[0], "RANGE") != 0U)
    {
        uint16_t deg;

        if ((n < 2U) || (Link_StrToU16(tok[1], &deg) == 0U) || (deg == 0U) || (deg > 360U))
        {
            Link_SendText("ERR BAD ARGS\r\n");
            return;
        }
        Servo_SetAngleRange(deg);
        Link_SendText("OK RANGE ");
        Link_SendNumU16(deg);
        Link_SendText("\r\n");
        return;
    }

    if (Link_StrEqCI(tok[0], "RESET") != 0U)
    {
        uint16_t id;

        if ((n < 2U) || (Link_StrToU16(tok[1], &id) == 0U) || (id == 0U) || (id > 250U))
        {
            Link_SendText("ERR BAD ARGS\r\n");
            return;
        }
        Servo_SendFrame((uint8_t)id, SERVO_CMD_RESET, 0, 0);
        Link_SendText("OK RESET ");
        Link_SendNumU16(id);
        Link_SendText("\r\n");
        return;
    }

    Link_SendText("ERR UNKNOWN CMD\r\n");
}

static void Link_HandleText(char *line)
{
    char    *tok[6];
    uint8_t  n = Link_Tokenize(line, tok, 6);

    if (n == 0U)
    {
        return;
    }
    Link_TextServoCommand(tok, n);
}

/* ============================== 主循环接口 =============================== */
void Link_Init(uint32_t baud)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef  nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* PA2 = USART2_TX */
    gpio.GPIO_Pin   = GPIO_Pin_2;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* PA3 = USART2_RX */
    gpio.GPIO_Pin   = GPIO_Pin_3;
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate            = baud;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &usart);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);

    nvic.NVIC_IRQChannel                   = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 2;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(USART2, ENABLE);

    s_rxHead = 0;
    s_rxTail = 0;
    s_txHead = 0;
    s_txTail = 0;

    s_rxMode      = 0;
    s_binState    = 0;
    s_textLen     = 0;
    s_telemHz     = ARM_TELEM_RATE_HZ;
    s_lastTelemTick = uwTick;
}

void Link_Poll(void)
{
    uint8_t b;

    /* 1) 处理接收缓冲 */
    while (Link_RxGet(&b) != 0)
    {
        s_lastByteTick = uwTick;

        if (s_rxMode == 0U)
        {
            if (b == LINK_SOH0)
            {
                s_rxMode   = 1U;
                s_binState = 0;
            }
            else
            {
                s_rxMode = 2U;
                s_textLen = 0;
            }
        }

        if (s_rxMode == 1U)
        {
            Link_BinByte(b);
        }
        else
        {
            if ((b == '\n') || (b == '\r'))
            {
                if (s_textLen > 0U)
                {
                    s_text[s_textLen] = 0;
                    Link_HandleText(s_text);
                    s_textLen = 0;
                }
            }
            else if (s_textLen < (LINK_TEXT_LINE_MAX - 1U))
            {
                s_text[s_textLen] = (char)b;
                s_textLen++;
            }
            else
            {
                s_textLen = 0;      /* 超长行丢弃 */
            }
        }
    }

    /* 2) 线路静默 200ms 后允许重新判定二进制/文本模式 */
    if ((s_rxMode != 0U) &&
        ((uint32_t)(uwTick - s_lastByteTick) > 200U))
    {
        s_rxMode   = 0U;
        s_binState = 0;
        s_textLen  = 0;
    }

    /* 3) 定时上报状态帧 */
    if (s_telemHz > 0U)
    {
        uint32_t period = 1000UL / (uint32_t)s_telemHz;

        if (period == 0UL)
        {
            period = 1UL;
        }
        if ((uint32_t)(uwTick - s_lastTelemTick) >= period)
        {
            s_lastTelemTick = uwTick;
            Link_SendState();
        }
    }
}
