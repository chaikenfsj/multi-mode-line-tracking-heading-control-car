#ifndef _HEADFILE_H_
#define _HEADFILE_H_

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* 原 main.h 内容直接内联 */
#include "Drivers/MSPM0/clock.h"
#include "Drivers/MSPM0/interrupt.h"
#include "Drivers/MPU6050/mpu6050.h"
#include "Drivers/OLED_Hardware_I2C/oled_hardware_i2c.h"

#include "User/delay.h"
#include "User/motor.h"
#include "User/encoder.h"
#include "User/gray.sensor.h"
#include "User/kinematics.h"
#include "User/pid.h"
#include "User/track_ctrl.h"
#include "User/key.h"
#include "User/control.h"

/* 按键全局标志 (interrupt.c 置1, main 读取后清零) */
extern volatile uint8_t g_key_pressed;

#endif /* _HEADFILE_H_ */
