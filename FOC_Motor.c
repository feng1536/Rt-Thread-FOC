/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-01-19     feng_       the first version
 */

#include<FOC_Motor.h>

static struct rt_device_pwm* FOC_PWM_Device;


void FOC_Motor_Init(void)
{

    FOC_PWM_Device = (struct rt_device_pwm *)rt_device_find(FOC_Motor_PWM);
    //查找pwm设备

    rt_pwm_set(FOC_PWM_Device, FOC_PWM_A, 1000000000, 0);
    rt_pwm_enable(FOC_PWM_Device, FOC_PWM_A);
    rt_pwm_set(FOC_PWM_Device, FOC_PWM_B, 1000000000, 0);
    rt_pwm_enable(FOC_PWM_Device, FOC_PWM_B);
    rt_pwm_set(FOC_PWM_Device, FOC_PWM_C, 1000000000, 0);
    rt_pwm_enable(FOC_PWM_Device, FOC_PWM_C);
    //开启pwm
}

void FOC_PWM_Write(float Uin,float speed,float time_cycle)//时间戳单位为毫秒
{
    float Up1,Up2,Ua,Ub,Uc,step,step_angle;
    static float E_angle = 0;
    uint32_t pulse1,pulse2,pulse3;

    if (Uin > (float)Power_Supply/2)
    {
        Uin = (float)Power_Supply/2;
    }

    step = 1000/time_cycle;
    step_angle = (speed*Pole_Pairs*2*PI)/step;

    E_angle = E_angle+step_angle;
        if(E_angle >=(2*PI))
    {
        E_angle = E_angle-2*PI;
    }

    Up2 = Uin*cos(E_angle);
    Up1 = -Uin*sin(E_angle);
    Ua = Up1 + Power_Supply/2;
    Ub = (Up2*sqrt(3)-Up1)/2 + Power_Supply/2;
    Uc = (-Up1-sqrt(3)*Up2)/2 + Power_Supply/2;
    pulse1 = (uint32_t)((Ua/Power_Supply)*10000);
    pulse2 = (uint32_t)((Ub/Power_Supply)*10000);
    pulse3 = (uint32_t)((Uc/Power_Supply)*10000);

    //rt_kprintf("%d %d %d %d\n",pulse1,pulse2,pulse3,(int)(E_angle*100));

    rt_pwm_set(FOC_PWM_Device, FOC_PWM_A, 10000, pulse1);
    rt_pwm_set(FOC_PWM_Device, FOC_PWM_B, 10000, pulse2);
    rt_pwm_set(FOC_PWM_Device, FOC_PWM_C, 10000, pulse3);


}

void FOC_STOP(void)
{
    rt_pwm_set(FOC_PWM_Device, FOC_PWM_A, 10000, 0);
    rt_pwm_set(FOC_PWM_Device, FOC_PWM_B, 10000, 0);
    rt_pwm_set(FOC_PWM_Device, FOC_PWM_C, 10000, 0);

}




void encoder_init(void)
{
    if(i2c_bus != NULL)
    {
        rt_kprintf("The Encoder i2c device is on used\n");
        return ;
    }
    i2c_bus=(struct rt_i2c_bus_device *)rt_device_find("i2c1");
        if(i2c_bus==RT_NULL)
        {
            rt_kprintf("no device \n");
        }
        else
        {
            rt_kprintf("find device \n");
         }

}

//ENCODER写指令
rt_err_t ENCODER_Read(rt_uint16_t *data)
{
    rt_uint8_t buf[3]={0},i=1;
    struct rt_i2c_msg msgs;
while(i<3){
    buf[0] = 0x0C+i-1;

    msgs.addr = 0x36;
    msgs.flags = RT_I2C_WR;
    msgs.buf = buf;
    msgs.len = 1;

    /* 调用I2C设备接口传输数据 */
    if (rt_i2c_transfer(i2c_bus, &msgs, 1) == 1){;}
    else
    {
        return -RT_ERROR;
    }

    msgs.addr = 0x36;
    msgs.flags = RT_I2C_RD;
    msgs.buf = (buf+i);
    msgs.len = 1;

    /* 调用I2C设备接口传输数据 */
    if (rt_i2c_transfer(i2c_bus, &msgs, 1) == 1){;}
    else
    {
        return -RT_ERROR;
    }
    i++;
}
    *data = buf[1]<<8|buf[2];
    return RT_EOK;
}

int Encoder_Speed_Get(rt_uint16_t Sampling_time,rt_uint16_t out_put_gain,rt_uint16_t debug_flag)
{
    static float angles_error,angle_speed,angle_get,angle_last=0;
    rt_uint16_t data;
    ENCODER_Read(&data);
    angle_get = data*0.087890625;
    angles_error = angle_get -angle_last;
    if(angles_error > 200)
    {
        angles_error = 360-angles_error;
    }
    else if (angles_error<-200)
    {
        angles_error = 360+angles_error;
    }

    angle_speed = angles_error*1000/Sampling_time/360;
    angle_last = angle_get;
    if(debug_flag)
    {
        rt_kprintf("Encoder num:%d  angle:%d  angle_error:%d  speed:%d\n",
                data,(int)(angle_get*out_put_gain),(int)(angles_error*out_put_gain),(int)(angle_speed*out_put_gain));
    }
    return angle_speed;
}

