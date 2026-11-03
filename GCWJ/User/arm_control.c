/**
  ******************************************************************************
  * @file    arm_control.c
  * @brief   机械臂执行层实现
  *
  *  上位机(ROS) 每 20ms 送来 6 个目标位置(0~4095) →
  *  本层做「软限位 → 角速度限速 → 软启动」→
  *  每个控制周期把当前 6 个位置用 SYNC WRITE 同步写给总线舵机。
  *
  *  限速用整数定点运算，无浮点，保证 Cortex-M3 上节拍稳定。
  ******************************************************************************
  */

#include "arm_control.h"
#include "Servo.h"

extern volatile uint32_t uwTick;        /* stm32f10x_it.c 中 SysTick 1ms 计数 */

/* 舵机内部插值时长：比控制周期略长，避免两帧之间出现"停一下"的顿挫 */
#define ARM_SERVO_RUNTIME_MS    (ARM_SERVO_PERIOD_MS + (ARM_SERVO_PERIOD_MS / 4U))

static const uint8_t  s_id[ARM_JOINT_NUM]      = ARM_SERVO_IDS;
static const uint16_t s_home[ARM_JOINT_NUM]    = ARM_HOME_TICKS;
static const uint16_t s_minDef[ARM_JOINT_NUM]  = ARM_TICK_MIN;
static const uint16_t s_maxDef[ARM_JOINT_NUM]  = ARM_TICK_MAX;
static const uint16_t s_dps[ARM_JOINT_NUM]     = ARM_MAX_DPS;

static uint16_t s_target[ARM_JOINT_NUM];    /* 上位机目标 */
static uint16_t s_cmd[ARM_JOINT_NUM];       /* 已下发给舵机的位置 */
static uint16_t s_meas[ARM_JOINT_NUM];      /* 回读的真实位置 */
static uint16_t s_tmin[ARM_JOINT_NUM];      /* 软限位 */
static uint16_t s_tmax[ARM_JOINT_NUM];
static uint16_t s_rampFrom[ARM_JOINT_NUM];  /* 回中位起点 */
static uint16_t s_timeBuf[ARM_JOINT_NUM];   /* 每次下发的运行时间 */

static uint8_t  s_mode;
static uint8_t  s_flags;
static uint8_t  s_busMode;
static uint8_t  s_readIdx;
static uint8_t  s_readDiv;
static uint32_t s_lastCycle;
static uint32_t s_lastTargetRx;
static uint32_t s_modeEnter;
static uint32_t s_homeStart;
static uint16_t s_homeDur;
static uint8_t  s_homeRunning;

/* ------------------------------------------------------------------ */
/* 内部工具                                                            */
/* ------------------------------------------------------------------ */

/* 每个控制周期允许的最大步进（ticks），由「度/秒」换算而来：
 * ticks_per_cycle = dps * 4095 * period / (rangeDeg * 1000) */
static uint16_t Arm_MaxStep(uint8_t idx)
{
    uint32_t num = (uint32_t)s_dps[idx] * 4095UL * (uint32_t)ARM_SERVO_PERIOD_MS;
    uint32_t den = (uint32_t)ARM_SERVO_RANGE_DEG * 1000UL;
    uint32_t step = num / den;

    if (step < 1UL)
    {
        step = 1UL;
    }
    if (step > 4095UL)
    {
        step = 4095UL;
    }
    return (uint16_t)step;
}

/* 把 value 往 goal 方向挪最多 step，返回新值 */
static uint16_t Arm_Slew(uint16_t value, uint16_t goal, uint16_t step)
{
    if (value < goal)
    {
        uint16_t d = (uint16_t)(goal - value);
        value = (d > step) ? (uint16_t)(value + step) : goal;
    }
    else if (value > goal)
    {
        uint16_t d = (uint16_t)(value - goal);
        value = (d > step) ? (uint16_t)(value - step) : goal;
    }
    return value;
}

/* 软限位钳位 */
static uint16_t Arm_Clamp(uint8_t idx, uint16_t ticks)
{
    if (ticks < s_tmin[idx])
    {
        s_flags |= ARM_FLAG_LIMIT;
        return s_tmin[idx];
    }
    if (ticks > s_tmax[idx])
    {
        s_flags |= ARM_FLAG_LIMIT;
        return s_tmax[idx];
    }
    return ticks;
}

/* 把 s_cmd 同步写给 6 个舵机 */
static void Arm_SendCmd(void)
{
    uint8_t i;

    for (i = 0; i < ARM_JOINT_NUM; i++)
    {
        s_timeBuf[i] = (uint16_t)ARM_SERVO_RUNTIME_MS;
    }

    if (s_busMode == 0U)
    {
        /* 一帧广播同步写：FF FF FE LEN 83 2A 04 ... SUM */
        Servo_SyncWritePosTime(ARM_JOINT_NUM, s_id, s_cmd, s_timeBuf);
    }
    else if (s_busMode == 1U)
    {
        /* 逐个 WRITE(0x03)，每帧等应答，最保守 */
        for (i = 0; i < ARM_JOINT_NUM; i++)
        {
            Servo_MoveWaitAck(s_id[i], s_cmd[i], s_timeBuf[i]);
        }
    }
    else
    {
        /* REG WRITE(0x04) 逐个预写 + ACTION(0x05) 广播同时执行 */
        for (i = 0; i < ARM_JOINT_NUM; i++)
        {
            Servo_RegWritePosTime(s_id[i], s_cmd[i], s_timeBuf[i]);
        }
        Servo_Action();
    }
}

/* ------------------------------------------------------------------ */
/* 初始化                                                              */
/* ------------------------------------------------------------------ */
void Arm_Init(void)
{
    uint8_t i;

    for (i = 0; i < ARM_JOINT_NUM; i++)
    {
        s_tmin[i]     = s_minDef[i];
        s_tmax[i]     = s_maxDef[i];
        s_target[i]   = s_home[i];
        s_cmd[i]      = s_home[i];
        s_meas[i]     = 0xFFFFU;
        s_rampFrom[i] = s_home[i];
        s_timeBuf[i]  = (uint16_t)ARM_SERVO_PERIOD_MS;

        if (s_tmin[i] > s_tmax[i])          /* 配置写反时兜底 */
        {
            s_tmin[i] = 0U;
            s_tmax[i] = 4095U;
        }
    }

    s_mode        = ARM_MODE_IDLE;
    s_flags       = 0U;
    s_busMode     = ARM_BUS_MODE;
    s_readIdx     = 0U;
    s_readDiv     = 0U;
    s_homeRunning = 0U;
    s_homeDur     = 0U;
    s_lastCycle   = uwTick;
    s_lastTargetRx = uwTick;
    s_modeEnter   = uwTick;
    s_homeStart   = uwTick;
}

/* ------------------------------------------------------------------ */
/* 上位机命令                                                          */
/* ------------------------------------------------------------------ */
void Arm_SetTargets(const uint16_t *ticks)
{
    uint8_t i;

    for (i = 0; i < ARM_JOINT_NUM; i++)
    {
        s_target[i] = (ticks[i] > 4095U) ? 4095U : ticks[i];
    }
    s_lastTargetRx = uwTick;
}

void Arm_SetMode(uint8_t mode)
{
    if (mode > ARM_MODE_HOME)
    {
        return;
    }
    if (mode != s_mode)
    {
        s_modeEnter    = uwTick;
        s_lastTargetRx = uwTick;    /* 重新计时：避免刚进跟随就被判超时 */
        s_homeRunning  = 0U;
        if (mode == ARM_MODE_STREAM)
        {
            s_flags &= (uint8_t)~ARM_FLAG_TIMEOUT;
            /* 从"当前位置"平滑接上轨迹，避免第一条目标造成跳变 */
            Arm_GetCmd(s_target);
        }
    }
    s_mode = mode;
}

void Arm_Stop(void)
{
    uint8_t i;

    s_mode        = ARM_MODE_IDLE;
    s_homeRunning = 0U;
    for (i = 0; i < ARM_JOINT_NUM; i++)
    {
        s_target[i] = s_cmd[i];     /* 目标锁死当前值，舵机自己保持 */
    }
}

void Arm_Home(uint16_t timeMs)
{
    uint8_t i;

    for (i = 0; i < ARM_JOINT_NUM; i++)
    {
        s_rampFrom[i] = s_cmd[i];
        s_target[i]   = s_home[i];
    }
    if (timeMs < 100U)
    {
        timeMs = 100U;
    }
    s_homeDur     = timeMs;
    s_homeStart   = uwTick;
    s_homeRunning = 1U;
    s_modeEnter   = uwTick;
    s_mode        = ARM_MODE_HOME;
}

void Arm_SetLimits(const uint16_t *mn, const uint16_t *mx)
{
    uint8_t i;

    for (i = 0; i < ARM_JOINT_NUM; i++)
    {
        if (mn[i] <= mx[i])
        {
            s_tmin[i] = mn[i];
            s_tmax[i] = mx[i];
        }
    }
}

uint8_t Arm_SetBusMode(uint8_t m)
{
    if (m > 2U)
    {
        return 0U;
    }
    s_busMode = m;
    return 1U;
}

/* ------------------------------------------------------------------ */
/* 状态查询                                                            */
/* ------------------------------------------------------------------ */
uint8_t  Arm_GetMode(void)      { return s_mode; }
uint8_t  Arm_GetFlags(void)     { return s_flags; }
uint8_t  Arm_GetBusMode(void)   { return s_busMode; }
uint16_t Arm_GetPeriodMs(void)  { return (uint16_t)ARM_SERVO_PERIOD_MS; }

void Arm_GetCmd(uint16_t *out)
{
    uint8_t i;
    for (i = 0; i < ARM_JOINT_NUM; i++)
    {
        out[i] = s_cmd[i];
    }
}

void Arm_GetMeasured(uint16_t *out)
{
    uint8_t i;
    for (i = 0; i < ARM_JOINT_NUM; i++)
    {
        out[i] = s_meas[i];
    }
}

/* ------------------------------------------------------------------ */
/* 每个控制周期的推进                                                  */
/* ------------------------------------------------------------------ */

/* 跟随上位机目标 */
static void Arm_StepStream(void)
{
    uint8_t  i;
    uint16_t step;
    uint8_t  soft = 0U;

    if ((uint32_t)(uwTick - s_modeEnter) < (uint32_t)ARM_SOFTSTART_MS)
    {
        soft = 1U;
    }

    for (i = 0; i < ARM_JOINT_NUM; i++)
    {
        uint16_t goal = Arm_Clamp(i, s_target[i]);

        step = Arm_MaxStep(i);
        if (soft != 0U)
        {
            step = (uint16_t)((step > 1U) ? (step / 2U) : 1U);
        }
        s_cmd[i] = Arm_Slew(s_cmd[i], goal, step);
    }
}

/* 回中位（按时间线性插值） */
static void Arm_StepHome(void)
{
    uint8_t  i;
    uint32_t el = (uint32_t)(uwTick - s_homeStart);

    if (s_homeRunning == 0U)
    {
        return;
    }
    if (el >= (uint32_t)s_homeDur)
    {
        for (i = 0; i < ARM_JOINT_NUM; i++)
        {
            s_cmd[i]    = s_home[i];
            s_target[i] = s_home[i];
        }
        Arm_SendCmd();                      /* 把最终位置补发一次 */
        s_homeRunning = 0U;
        s_mode        = ARM_MODE_IDLE;      /* 到位后保持 */
        return;
    }

    for (i = 0; i < ARM_JOINT_NUM; i++)
    {
        int32_t from = (int32_t)s_rampFrom[i];
        int32_t to   = (int32_t)s_home[i];
        int32_t v    = from + ((to - from) * (int32_t)el) / (int32_t)s_homeDur;

        if (v < 0)
        {
            v = 0;
        }
        if (v > 4095)
        {
            v = 4095;
        }
        s_cmd[i]    = (uint16_t)v;
        s_target[i] = s_cmd[i];
    }
}

/* 轮询回读一个舵机的真实位置 */
static void Arm_PollReadback(void)
{
#if (ARM_READBACK_ENABLE != 0U)
    uint16_t p;

    s_readDiv++;
    if (s_readDiv < ARM_READBACK_DIV)
    {
        return;
    }
    s_readDiv = 0U;

    if (Servo_ReadPosition(s_id[s_readIdx], &p, ARM_READBACK_TIMEOUT_MS) != 0)
    {
        s_meas[s_readIdx] = (p > 4095U) ? 4095U : p;
    }
    else
    {
        s_meas[s_readIdx] = 0xFFFFU;
        s_flags |= ARM_FLAG_READBACK_ERR;
    }

    s_readIdx++;
    if (s_readIdx >= ARM_JOINT_NUM)
    {
        s_readIdx = 0U;
    }
#endif
}

void Arm_Tick(void)
{
    if ((uint32_t)(uwTick - s_lastCycle) < (uint32_t)ARM_SERVO_PERIOD_MS)
    {
        return;
    }
    s_lastCycle = uwTick;

    /* 1) 链路超时保护：电脑不发了就原地停住 */
    if ((s_mode == ARM_MODE_STREAM) &&
        ((uint32_t)(uwTick - s_lastTargetRx) > (uint32_t)ARM_LINK_TIMEOUT_MS))
    {
        s_mode = ARM_MODE_IDLE;
        s_flags |= ARM_FLAG_TIMEOUT;
    }

    /* 2) 更新目标 */
    if (s_mode == ARM_MODE_STREAM)
    {
        s_flags |= ARM_FLAG_STREAM;
        Arm_StepStream();
    }
    else
    {
        s_flags &= (uint8_t)~ARM_FLAG_STREAM;
        if (s_mode == ARM_MODE_HOME)
        {
            Arm_StepHome();
        }
    }

    /* 3) 下发到总线舵机 */
    if (s_mode != ARM_MODE_IDLE)
    {
        Arm_SendCmd();
    }

    /* 4) 位置回读（每 ARM_READBACK_DIV 个周期读一个舵机） */
    Arm_PollReadback();
}
