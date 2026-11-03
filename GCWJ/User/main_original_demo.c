/**
  ******************************************************************************
  * @file    main_original_demo.c
  * @brief   原始演示版 main.c 的备份（修改前的原样拷贝，供回退参考）
  *
  * 本文件不会被 Keil 工程编译（未加入工程），仅作为备份留存。
  * 当前实际使用的程序是 main.c（上位机命令接收版）。
  ******************************************************************************
  */

#include "stm32f10x.h"                  // Device header
#include "Servo.h"

extern volatile uint32_t uwTick;        /* stm32f10x_it.c 中 SysTick 1ms 计数 */

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

int main(void)
{
    uint8_t i;

    g_diag.magic   = 0xD1A60001UL;
    g_diag.phase   = 3;                 /* 工作配置：PA9 半双工 115200 v4.01 */
    g_diag.servoId = 1;

    Servo_SetAngleRange(300);           /* 满行程 300°（SCS 系列舵机常见值） */

    Servo_Config(SERVO_PIN_PA9_HD, 115200U);
    Delay_ms(1000);                     /* 等舵机上电稳定 */

    /* 广播把舵机 ID 设为 1（帧：FF FF FE 04 03 05 01 F4，掉电保存） */
    Servo_SetID(1);
    Delay_ms(500);

    /* ---- 主循环：摆 4 下 → 停 5 秒 ---- */
    while (1)
    {
        for (i = 0; i < 4; i++)
        {
            g_diag.servoId = 1;
            SwingOnce();
        }

        g_diag.servoId = 0;
        Delay_ms(5000);                 /* 停顿 5 秒 */
    }
}
