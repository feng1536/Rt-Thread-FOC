/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-01-19     feng_       the first version
 */
#ifndef APPLICATIONS_FOC_MOTOR_H_
#define APPLICATIONS_FOC_MOTOR_H_

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <drv_common.h>
#include <math.h>
#include <OLED.H>

#define PI 3.14159265358979323846
#define FOC_Motor_PWM "pwm2"
#define Pole_Pairs  7
#define Power_Supply 5
#define FOC_PWM_A   1
#define FOC_PWM_B   2
#define FOC_PWM_C   3

void FOC_Motor_Init(void);
void FOC_PWM_Write(float Uin,float speed,float time_cycle);
void FOC_STOP(void);


#define ENCODER_BUS_NAME "i2c1"
#define ENCODER_ADDR 0x36
#define Angle_Hight_Register_Addr 0x0C //寄存器高位地址
#define Angle_Low_Register_Addr   0x0D //寄存器低位地址

void encoder_init(void);
rt_err_t ENCODER_Read(rt_uint16_t *data);
int Encoder_Speed_Get(rt_uint16_t Sampling_time,rt_uint16_t out_put_gain,rt_uint16_t debug_flag);


#endif /* APPLICATIONS_FOC_MOTOR_H_ */
