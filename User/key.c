#include "headfile.h"

/* 四键扫描: 读取中断标志并确认按键仍按下；不等待松手，避免阻塞控制主循环。 */
uint8_t Key_Scan(void)
{
    if (g_key_t1) {
        g_key_t1 = 0;
        delay_ms(20);
        if (DL_GPIO_readPins(GPIOB, KEY_T1_PIN) == 0) return 1;
    }
    if (g_key_t2) {
        g_key_t2 = 0;
        delay_ms(20);
        if (DL_GPIO_readPins(GPIOB, KEY_T2_PIN) == 0) return 2;
    }
    if (g_key_t3) {
        g_key_t3 = 0;
        delay_ms(20);
        if (DL_GPIO_readPins(GPIOB, KEY_T3_PIN) == 0) return 3;
    }
    if (g_key_t4) {
        g_key_t4 = 0;
        delay_ms(20);
        if (DL_GPIO_readPins(GPIOB, KEY_T4_PIN) == 0) return 4;
    }
    return 0;
}
