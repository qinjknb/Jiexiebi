/**
  ******************************************************************************
  * @file    main.c
  * @brief   舵机控制 + 上位机命令接收版
  *
  * 双串口同时工作：
  *   1) USART1  PA9 单线半双工 115200 —— 与舵机通信（德晟串口总线智能
  *      舵机协议 v4.01，沿用原 Servo.c 驱动，未做任何改动）；
  *   2) USART2  PA2(TX) / PA3(RX) 115200-8-N-1 —— 与电脑上位机通信，
  *      文本命令行协议，'\n' 结尾（配套 Python 上位机见 ServoHost 目录）。
  *
  * 上位机命令（大小写不敏感，参数用空格分开，可带小数角度）：
  *   PING  <id>                      查询舵机是否存在     应答: OK PING id / ERR PING id
  *   MOVE  <id> <角度°> <时间ms>     按角度转动           应答: OK MOVE id pos
  *   MOVEP <id> <位置0~4095> <ms>    按原始位置转动       应答: OK MOVEP id
  *   READ  <id>                      读当前位置           应答: POS id pos / ERR READ id
  *   SETID <id>                      广播改总线舵机ID(掉电保存) 应答: OK SETID id
  *   RANGE <角度°>                   改满行程角度(180/270/300/360) 应答: OK RANGE deg
  *   RESET <id>                      复位舵机             应答: OK RESET id
  *   DEMO                            恢复上电演示摆动     应答: OK DEMO
  *   HELP                            打印命令帮助
  *
  * 工作模式：
  *   上电后先执行原演示程序（ID=1 舵机 150°~161° 小摆 4 下停 5 秒循环）；
  *   收到任意上位机命令即进入“手动模式”并停止演示，发送 DEMO 可恢复演示。
  *   演示过程中收到命令会在当前小摆结束后立即切换（最迟约 2 秒）。
  *
  * 电脑接线（USB-TTL 串口模块）：
  *   模块 TX  -> PA3 (USART2_RX)
  *   模块 RX  -> PA2 (USART2_TX)
  *   模块 GND -> GND（必须共地）
  *   舵机数据线仍然接 PA9，舵机供电用外部电源（勿用 USB-TTL 的 3.3V 带舵机）
  ******************************************************************************
  */

#include "stm32f10x.h"                  // Device header
#include "Servo.h"
#include "LED.h"

extern volatile uint32_t uwTick;        /* stm32f10x_it.c 中 SysTick 1ms 计数 */

/* ============================ PC 串口（USART2） ============================ */
#define PC_RX_BUF_SIZE   128
#define PC_LINE_MAX      64

static volatile uint8_t  s_pcRxBuf[PC_RX_BUF_SIZE];
static volatile uint8_t  s_pcRxHead = 0;
static volatile uint8_t  s_pcRxTail = 0;

static char       s_line[PC_LINE_MAX];
static uint8_t    s_lineLen = 0;
static volatile uint8_t s_lineReady = 0;

static volatile uint8_t s_mode = 0;     /* 0=演示模式  1=手动模式 */
static uint16_t  s_pcRange = 300;       /* 满行程角度（与 Servo.c 内一致） */

/* ------------------------------ 延时/演示 ---------------------------------- */
static void Delay_ms(uint32_t ms)
{
    uint32_t start = uwTick;
    uint32_t guard = 0;
    while ((uint32_t)(uwTick - start) < ms)
    {
        guard++;
        if (guard > 100000000UL)        /* 约 4 秒兜底，防 SysTick 异常死等 */
        {
            break;
        }
    }
    g_diag.tickMs = uwTick;
}

/* ID=1 舵机小摆一次：161° → 150° */
static void SwingOnce(void)
{
    Servo_MoveAngle(1, 161.0f, 400);    /* 从中位往外摆约 11° */
    Delay_ms(1000);

    Servo_MoveAngle(1, 150.0f, 400);    /* 回到中位 */
    Delay_ms(1000);
}

/* ------------------------------ PC 串口底层 ------------------------------- */
static void PC_SendByte(uint8_t b)
{
    while ((USART2->SR & USART_SR_TXE) == 0)
    {
    }
    USART2->DR = b;
}

static void PC_Send(const char *s)
{
    while (*s != 0)
    {
        PC_SendByte((uint8_t)*s);
        s++;
    }
}

static void PC_SendNum(uint16_t v)
{
    char tmp[6];
    uint8_t i = 0;
    if (v == 0)
    {
        PC_SendByte('0');
        return;
    }
    while (v > 0)
    {
        tmp[i++] = (char)('0' + (v % 10));
        v /= 10;
    }
    while (i > 0)
    {
        PC_SendByte((uint8_t)tmp[--i]);
    }
}

/* 发送一行文本并以 \r\n 结尾（兼容串口助手显示） */
static void PC_SendLine(const char *s)
{
    PC_Send(s);
    PC_Send("\r\n");
}

static void PC_Init(void)
{
    GPIO_InitTypeDef  gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef  nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* PA2 = USART2_TX 推挽复用 */
    gpio.GPIO_Pin   = GPIO_Pin_2;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* PA3 = USART2_RX 浮空输入 */
    gpio.GPIO_Pin   = GPIO_Pin_3;
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    usart.USART_BaudRate            = 115200;
    usart.USART_WordLength          = USART_WordLength_8b;
    usart.USART_StopBits            = USART_StopBits_1;
    usart.USART_Parity              = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &usart);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    nvic.NVIC_IRQChannel                   = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority        = 2;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(USART2, ENABLE);
}

/* USART2 接收中断：字节入环形缓冲（与 Servo.c 的 USART1 同思路） */
void USART2_IRQHandler(void)
{
    if ((USART2->SR & USART_SR_RXNE) != 0)
    {
        uint8_t b = (uint8_t)USART2->DR;
        uint8_t next = (uint8_t)(s_pcRxHead + 1);
        if (next == PC_RX_BUF_SIZE)
        {
            next = 0;
        }
        if (next != s_pcRxTail)
        {
            s_pcRxBuf[s_pcRxHead] = b;
            s_pcRxHead = next;
        }
    }
}

/* 从环形缓冲取字节拼行，遇 '\n'/'\r' 结束一行 */
static void PC_PollLine(void)
{
    while (s_pcRxHead != s_pcRxTail)
    {
        uint8_t b = s_pcRxBuf[s_pcRxTail];
        uint8_t next = (uint8_t)(s_pcRxTail + 1);
        if (next == PC_RX_BUF_SIZE)
        {
            next = 0;
        }
        s_pcRxTail = next;

        if (b == '\n' || b == '\r')
        {
            if (s_lineLen > 0)
            {
                s_line[s_lineLen] = 0;
                s_lineReady = 1;
                s_lineLen = 0;
                return;
            }
            continue;
        }
        if (s_lineLen < (PC_LINE_MAX - 1))
        {
            s_line[s_lineLen++] = (char)b;
        }
    }
}

/* ------------------------------ 命令解析 ---------------------------------- */
static uint8_t Tokenize(char *line, char *tok[], uint8_t maxTok)
{
    uint8_t n = 0;
    char *p = line;
    while (*p != 0 && n < maxTok)
    {
        while (*p == ' ' || *p == '\t')
        {
            p++;
        }
        if (*p == 0)
        {
            break;
        }
        tok[n++] = p;
        while (*p != 0 && *p != ' ' && *p != '\t')
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

static uint8_t StrEqCI(const char *a, const char *b)
{
    while (*a != 0 && *b != 0)
    {
        char ca = *a;
        char cb = *b;
        if (ca >= 'a' && ca <= 'z')
        {
            ca = (char)(ca - 32);
        }
        if (cb >= 'a' && cb <= 'z')
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
    if (*a == 0 && *b == 0)
    {
        return 1;
    }
    return 0;
}

static uint8_t StrToU16(const char *s, uint16_t *out)
{
    uint32_t v = 0;
    uint8_t  digits = 0;
    while (*s != 0)
    {
        if (*s >= '0' && *s <= '9')
        {
            v = v * 10UL + (uint32_t)(*s - '0');
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
    if (digits == 0)
    {
        return 0;
    }
    *out = (uint16_t)v;
    return 1;
}

static uint8_t StrToFloat(const char *s, float *out)
{
    float v = 0.0f;
    float frac = 0.1f;
    float sign = 1.0f;
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
            if (sawDot)
            {
                return 0;
            }
            sawDot = 1;
        }
        else if (c >= '0' && c <= '9')
        {
            digits = 1;
            if (sawDot)
            {
                v = v + (float)(c - '0') * frac;
                frac = frac * 0.1f;
            }
            else
            {
                v = v * 10.0f + (float)(c - '0');
            }
        }
        else
        {
            return 0;
        }
        s++;
    }
    if (digits == 0)
    {
        return 0;
    }
    *out = sign * v;
    return 1;
}

static void PC_SendHelp(void)
{
    PC_SendLine("HELP: PING id | MOVE id angleDeg ms | MOVEP id pos ms |");
    PC_SendLine("      READ id | SETID id | RANGE deg | RESET id | DEMO");
}

static void PC_ProcessLine(char *line)
{
    char *tok[6];
    uint8_t n = Tokenize(line, tok, 6);

    if (n == 0)
    {
        return;
    }

    /* DEMO：恢复演示模式，不切手动 */
    if (StrEqCI(tok[0], "DEMO"))
    {
        s_mode = 0;
        LED_OFF();
        PC_SendLine("OK DEMO");
        return;
    }

    /* HELP：打印帮助，不切手动 */
    if (StrEqCI(tok[0], "HELP"))
    {
        PC_SendHelp();
        return;
    }

    /* 其余命令一律进入手动模式，停止演示 */
    s_mode = 1;
    LED_ON();

    if (StrEqCI(tok[0], "PING"))
    {
        uint16_t id;
        if (n < 2 || !StrToU16(tok[1], &id) || id == 0 || id > 250)
        {
            PC_SendLine("ERR BAD ARGS");
            return;
        }
        if (Servo_Ping((uint8_t)id, 300))
        {
            PC_Send("OK PING ");
            PC_SendNum(id);
            PC_SendLine("");
        }
        else
        {
            PC_Send("ERR PING ");
            PC_SendNum(id);
            PC_SendLine("");
        }
        return;
    }

    if (StrEqCI(tok[0], "MOVE"))
    {
        uint16_t id, time, pos;
        float angle;
        if (n < 4 || !StrToU16(tok[1], &id) || id == 0 || id > 250 ||
            !StrToFloat(tok[2], &angle) || !StrToU16(tok[3], &time))
        {
            PC_SendLine("ERR BAD ARGS");
            return;
        }
        if (angle < 0.0f)
        {
            angle = 0.0f;
        }
        if (angle > (float)s_pcRange)
        {
            angle = (float)s_pcRange;
        }
        pos = (uint16_t)(angle * 4095.0f / (float)s_pcRange + 0.5f);
        Servo_MoveAngle((uint8_t)id, angle, time);
        PC_Send("OK MOVE ");
        PC_SendNum(id);
        PC_Send(" ");
        PC_SendNum(pos);
        PC_SendLine("");
        return;
    }

    if (StrEqCI(tok[0], "MOVEP"))
    {
        uint16_t id, pos, time;
        if (n < 4 || !StrToU16(tok[1], &id) || id == 0 || id > 250 ||
            !StrToU16(tok[2], &pos) || !StrToU16(tok[3], &time))
        {
            PC_SendLine("ERR BAD ARGS");
            return;
        }
        Servo_Move((uint8_t)id, pos, time);
        PC_Send("OK MOVEP ");
        PC_SendNum(id);
        PC_SendLine("");
        return;
    }

    if (StrEqCI(tok[0], "READ"))
    {
        uint16_t id, pos;
        if (n < 2 || !StrToU16(tok[1], &id) || id == 0 || id > 250)
        {
            PC_SendLine("ERR BAD ARGS");
            return;
        }
        if (Servo_ReadPosition((uint8_t)id, &pos, 200))
        {
            PC_Send("POS ");
            PC_SendNum(id);
            PC_Send(" ");
            PC_SendNum(pos);
            PC_SendLine("");
        }
        else
        {
            PC_Send("ERR READ ");
            PC_SendNum(id);
            PC_SendLine("");
        }
        return;
    }

    if (StrEqCI(tok[0], "SETID"))
    {
        uint16_t id;
        if (n < 2 || !StrToU16(tok[1], &id) || id == 0 || id > 250)
        {
            PC_SendLine("ERR BAD ARGS");
            return;
        }
        Servo_SetID((uint8_t)id);
        PC_Send("OK SETID ");
        PC_SendNum(id);
        PC_SendLine("");
        return;
    }

    if (StrEqCI(tok[0], "RANGE"))
    {
        uint16_t deg;
        if (n < 2 || !StrToU16(tok[1], &deg) || deg == 0 || deg > 360)
        {
            PC_SendLine("ERR BAD ARGS");
            return;
        }
        Servo_SetAngleRange(deg);
        s_pcRange = deg;
        PC_Send("OK RANGE ");
        PC_SendNum(deg);
        PC_SendLine("");
        return;
    }

    if (StrEqCI(tok[0], "RESET"))
    {
        uint16_t id;
        if (n < 2 || !StrToU16(tok[1], &id) || id == 0 || id > 250)
        {
            PC_SendLine("ERR BAD ARGS");
            return;
        }
        Servo_SendFrame((uint8_t)id, SERVO_CMD_RESET, 0, 0);
        PC_Send("OK RESET ");
        PC_SendNum(id);
        PC_SendLine("");
        return;
    }

    PC_SendLine("ERR UNKNOWN CMD");
}

/* ------------------------------ 主函数 ------------------------------------ */
int main(void)
{
    uint8_t i;

    g_diag.magic   = 0xD1A60001UL;
    g_diag.phase   = 3;                 /* 工作配置：PA9 半双工 115200 v4.01 */
    g_diag.servoId = 1;

    Servo_SetAngleRange(300);           /* 满行程 300°（与 s_pcRange 一致） */
    s_pcRange = 300;

    Servo_Config(SERVO_PIN_PA9_HD, 115200U);
    Delay_ms(1000);                     /* 等舵机上电稳定 */

    /* 广播把舵机 ID 设为 1（帧：FF FF FE 04 03 05 01 F4，掉电保存）
       注意：多舵机总线会同时改所有舵机 ID，单舵机时保留此行为 */
    Delay_ms(500);

    LED_Init();
    LED_OFF();

    PC_Init();
    PC_SendLine("READY STM32F103 SERVO HOST V1.0");   /* 开机问候 */
    PC_SendLine("MODE DEMO");

    /* ---- 主循环 ---- */
    while (1)
    {
        PC_PollLine();
        if (s_lineReady)
        {
            s_lineReady = 0;
            LED_Toggle();               /* 每收到一条命令闪一下灯 */
            PC_ProcessLine(s_line);
        }

        if (s_mode == 0)
        {
            /* 演示模式：摆 4 下 → 停 5 秒（原程序行为） */
            for (i = 0; i < 4; i++)
            {
                g_diag.servoId = 1;
                SwingOnce();

                /* 演示过程中也接收上位机命令，收到即打断演示 */
                PC_PollLine();
                if (s_lineReady)
                {
                    s_lineReady = 0;
                    LED_Toggle();
                    PC_ProcessLine(s_line);
                    if (s_mode == 1)
                    {
                        break;
                    }
                }
            }

            g_diag.servoId = 0;
            if (s_mode == 0)
            {
                Delay_ms(5000);         /* 停顿 5 秒 */
            }
        }
        else
        {
            Delay_ms(1);                /* 手动模式：空转等待命令 */
        }
    }
}
