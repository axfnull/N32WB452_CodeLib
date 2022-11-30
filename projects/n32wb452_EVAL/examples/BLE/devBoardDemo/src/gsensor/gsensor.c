/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file gsensor.c
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include "main.h"
#include "qma_i2c_drv.h"
#include "gsensor.h"

void qma7981_int(void) // QMA7981初始化设置
{
    qma_i2c_write_data(0xc3, 0x11); //将设备设置为活动模式,睡眠时间为0

    /*以下震动中断设置，芯片感受到震动INT1脚输出高电平*/
    qma_i2c_write_data(0x07, 0x09); // any_motion中断由X,Y,轴触发
    qma_i2c_write_data(0x1F, 0x18); //启用X，Y,轴上的any_motion中断
    qma_i2c_write_data(0x63, 0x1a); // any_motion中断映射到INT1脚
    qma_i2c_write_data(0x32, 0x2e); // any_motion阀值定义

    delay_ms(70); //延时70ms左右
}

void qma7981_read_raw_xyz(G_SENSOR_DATA* g_sensor_data) //读取X,Y,Z轴数据
{
//    uint16_t databuf[3];

//    databuf[0] = qma_i2c_read_data(0x01); // X轴低位
//    databuf[1] = qma_i2c_read_data(0x02); // X轴高位
//    databuf[2] = qma_i2c_read_data(0x03); // Y轴低位
//    databuf[3] = qma_i2c_read_data(0x04); // Y轴高位
//    databuf[4] = qma_i2c_read_data(0x05); // Z轴低位
//    databuf[5] = qma_i2c_read_data(0x06); // Z轴高位
    qma_i2c_read_ndata(0x01, 6, (uint8_t *)(&(g_sensor_data->x_acc)));
//    printf("x=%04x,y=%04x,z=%04x\r\n",g_sensor_data->x_acc,g_sensor_data->y_acc,g_sensor_data->z_acc);
    
    g_sensor_data->x_acc = g_sensor_data->x_acc >> 2; // X轴原始数据
    g_sensor_data->y_acc = g_sensor_data->y_acc >> 2; // Y轴原始数据
    g_sensor_data->z_acc = g_sensor_data->z_acc >> 2; // Z轴原始数据
}

int16_t dabs(int16_t x) //将负数据转化为正数
{
    int16_t Acc_data1 = 0;
    int16_t Acc_data2 = 0;

    if (x > 0)
    {
        Acc_data1 = x - 0;
        return Acc_data1;
    }
    else
    {
        Acc_data2 = 65535 - x;
        return Acc_data2;
    }
}

#if 0
 /*以下代码在主函数中调用*/
 QMA7981_int();//上电初始化设置
 
 
 qma7981_read_raw_xyz();//读取X,Y轴数据
 
 if((dabs(data[0]) > 1500)|(dabs(data[1]) > 1500))  
 {
                  //倾斜大于一定角度
 }
 else
 {
                  //倾斜小于一定角度或处于水平状态
 }
#endif
