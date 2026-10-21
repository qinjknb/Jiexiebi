/**
  ******************************************************************************
  * @file    main.c
  * @brief   DOFBOT Pro 机械臂固件：ROS 2 / MoveIt 轨迹跟随 + 调试命令
  *
  * 硬件与接线（已验证的接法）：
  *   电脑 USB ──> USB-TTL(CH340)  TX ──> STM32 PA3 (USART2_RX)
  *                                RX ──> STM32 PA2 (USART2_TX)
  *                                GND ──> STM32 GND（必须共地）
  *   STM32 PA9 ──> 总线舵机驱动板 UART 的 TX/RX（单线半双工，开关拨 A/UART-SERVO）
  *   舵机电源 ──> DC 9~12.6V（不要用 STM32 的 3.3V/5V 带 6 个舵机）
  *   ST-Link ──> SWD（烧录用）
  *
  * 两条链路：
  *   USART1 (PA9)  ── 德晟总线舵机 v4.01，115200，6 个舵机 ID = 1~6
  *   USART2 (PA2/3)── 电脑，115200：二进制帧（ROS 2）+ 文本命令（ServoHost）双兼容
  *
  * 上电后不自动摆动，必须收到上位机指令才动（安全第一）。
  * 二进制协议见 robot_link.h；配置见 robot_config.h。
  ******************************************************************************
  */

#include "stm32f10x.h"
#include "robot_config.h"
#include "arm_control.h"
#include "robot_link.h"
#include "Servo.h"
#include "LED.h"

extern volatile uint32_t uwTick;

/* ------------------------------ 延时 -------------------------------------- */
static void DelayMs(uint32_t ms)
{
    uint32_t start = uwTick;
    uint32_t guard = 0;

    while ((uint32_t)(uwTick - start) < ms)
    {
        guard++;
        if (guard > 20000000UL)     /* 约 1~2 秒兜底，防 SysTick 异常死等 */
        {
            break;
        }
    }
    g_diag.tickMs = uwTick;
}

/* ------------------------------ 状态灯 ------------------------------------ */
/* 跟随轨迹：常亮；其它状态：500ms 慢闪（表示固件活着） */
static void StatusLed(void)
{
    static uint32_t last = 0;

    if (Arm_GetMode() == ARM_MODE_STREAM)
    {
        LED_ON();
        return;
    }
    if ((uint32_t)(uwTick - last) >= 500U)
    {
        last = uwTick;
        LED_Toggle();
    }
}

/* ------------------------------ 主函数 ------------------------------------ */
int main(void)
{
    /* 调试器可直接在 RAM 里看这些值（见 Servo.h 的 Diag_t） */
    g_diag.magic    = 0xD1A60002UL;
    g_diag.phase    = 3;            /* PA9 半双工 115200 v4.01 */
    g_diag.servoId  = 0;

    LED_Init();
    LED_OFF();

    /* 1) 舵机总线（USART1 / PA9 单线半双工） */
    Servo_SetAngleRange(ARM_SERVO_RANGE_DEG);
    Servo_Config((uint8_t)ARM_SERVO_WIRING, ARM_SERVO_BAUD);
    DelayMs(500);                   /* 等舵机上电、总线稳定 */

    /* 2) 执行层与电脑通信 */
    Arm_Init();
    Link_Init(ARM_LINK_BAUD);

    /* 3) 开机问候（ServoHost 上位机靠这一行确认固件版本） */
    Link_SendText("\r\n");
    Link_SendText("READY STM32F103 ARM BRIDGE V2.0 (DOFBOT Pro)\r\n");
    Link_SendText("TEXT OK | BINARY AA55 | JOINTS=6\r\n");
    Link_SendText("HOLD: no motion until host sends commands.\r\n");

    /* 4) 主循环：解析上位机命令 + 按 20ms 节拍驱动机械臂 */
    while (1)
    {
        Link_Poll();                /* 收命令、发状态帧（非阻塞） */
        Arm_Tick();                 /* 限速插值 + 同步写舵机 */
        StatusLed();
        g_diag.tickMs = uwTick;
    }
}
