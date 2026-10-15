#include "ti_msp_dl_config.h"
#include "headfile.h"

/* ══════════════════ main ══════════════════ */
int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();  delay_ms(50);
    MPU6050_Init();     

    /* OLED 先初始化, 显示7校准进度 */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"MPU Calibrating", 8);
    OLED_ShowString(0, 2, (uint8_t *)"Wait 8-10s...", 8);

    Interrupt_Init();   /* 必须在 MPU6050_Init 之后, 才会使能 GPIOB 中断 */
    
    Tracking_SystemInit();

    uint8_t oled_buf[16];
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Task 0", 16);
    sprintf((char *)oled_buf, "Yaw:%5.1f", (double)yaw);
    OLED_ShowString(0, 2, oled_buf, 16);

    if (!enable_group1_irq)
        OLED_ShowString(0, 4, (uint8_t *)"MPU FAIL", 8);

    static unsigned long yaw_refresh_ms = 0;
    static unsigned long key_lockout_ms = 0;
    static bool key_lockout_active = false;
    static bool fault_reported = false;

    while (1)
    {
        unsigned long now_ms = tick_ms;

        /* MPU6050中断只置位；耗时的I2C FIFO读取和姿态解算在主循环完成。 */
        if (g_mpu_data_ready) {
            g_mpu_data_ready = 0;
            (void)Read_Quad();
        }

        if (key_lockout_active && (now_ms - key_lockout_ms >= 300UL))
            key_lockout_active = false;

        /* 速度环检测到编码器失速后已在中断中刹车，这里完成任务状态和显示处理。 */
        if (MotorControl_HasFault() && !fault_reported) {
            fault_reported = true;
            g_running = false;
            g_task = 0;
            g_workstep = 0;
            Alarm_Beep_ms(500);
            OLED_Clear();
            OLED_ShowString(0, 0, (uint8_t *)"ENCODER FAIL", 16);
        }

        /* ── 四键扫描 ── */
        uint8_t key = Key_Scan();
        if (key != 0 && !key_lockout_active) {
            key_lockout_active = true;
            key_lockout_ms = now_ms;

            if (g_running) {
                /* 运行中任意任务键只负责停车，不在同一次按键中重新启动。 */
                Tracking_Stop();
            } else {
                fault_reported = false;
                if      (key == 1) Task1_Start();
                else if (key == 2) Task2_Start();
                else if (key == 3) Task3_Start();
                else if (key == 4) Task4_Start();

                Alarm_Beep_ms(200);
                OLED_Clear();
                if      (key == 1) OLED_ShowString(0, 0, (uint8_t *)"Task 1", 16);
                else if (key == 2) OLED_ShowString(0, 0, (uint8_t *)"Task 2", 16);
                else if (key == 3) OLED_ShowString(0, 0, (uint8_t *)"Task 3", 16);
                else               OLED_ShowString(0, 0, (uint8_t *)"Task 4", 16);
                OLED_ShowString(0, 2, (uint8_t *)"Yaw:", 16);
            }
        }

        /* 使用SysTick毫秒时基，不依赖主循环执行速度。 */
        if (now_ms - yaw_refresh_ms >= 300UL) {
            yaw_refresh_ms = now_ms;
            uint8_t oled_buf[16];
            sprintf((char *)oled_buf, "Yaw:%5.1f", (double)yaw);
            OLED_ShowString(0, 2, oled_buf, 16);
        }

        /* ── 任务调度 (互斥: else if) ── */
        if      (g_running && g_task == 1) Task1_Loop();
        else if (g_running && g_task == 2) Task2_Loop();
        else if (g_running && g_task == 3) Task3_Loop();
        else if (g_running && g_task == 4) Task4_Loop();

        delay_ms(2);
    }
}

void Timer_Speed_INST_IRQHandler(void)
{
    DL_TimerG_clearInterruptStatus(Timer_Speed_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
    Encoder_SpeedCalc();
    if (g_running) pid_control();
}
