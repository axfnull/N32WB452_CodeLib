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
 * @file i2c_drv.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "n32wb452.h"
#include "n32wb452_gpio.h"
#include "hdc_i2c_drv.h"
#include "main.h"


#if 1

#define I2C_SET_TIMEOUT ((uint32_t)0x1000)
#define I2C_LONG_TIMEOUT ((uint32_t)(10 * I2C_SET_TIMEOUT))
#define I2C_MASTER_ADDR   0x30
#define I2C_SLAVE_ADDR    (HDC_I2C_DRV_ADDR << 1)

static __IO uint32_t I2CTimeout = I2C_LONG_TIMEOUT;


int hdc_i2c_gpio_init(void)
{
    I2C_InitType i2c2_master;
    GPIO_InitType i2c2_gpio;
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_I2C2, ENABLE);
    RCC_EnableAPB2PeriphClk(HDC_I2C_SDA_GPIO_CLK, ENABLE);

    /*PB10 -- SCL; PB11 -- SDA*/
    i2c2_gpio.Pin        = HDC_I2C_SDA_PIN | HDC_I2C_SCL_PIN;
    i2c2_gpio.GPIO_Speed = GPIO_Speed_50MHz;
    i2c2_gpio.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_InitPeripheral(HDC_I2C_SDA_PORT, &i2c2_gpio);

    I2C_DeInit(HDC_I2C);
    i2c2_master.BusMode     = I2C_BUSMODE_I2C;
    i2c2_master.FmDutyCycle = I2C_FMDUTYCYCLE_2;
    i2c2_master.OwnAddr1    = I2C_MASTER_ADDR;
    i2c2_master.AckEnable   = I2C_ACKEN;
    i2c2_master.AddrMode    = I2C_ADDR_MODE_7BIT;
    i2c2_master.ClkSpeed    = HDC_I2C_CLOCK; // 100000 100K

    I2C_Init(HDC_I2C, &i2c2_master);
    I2C_Enable(HDC_I2C, ENABLE);
    return 0;
}


void hdc_i2c_gpio_deinit(void)
{
    GPIO_InitType GPIO_InitStructure;
    
    RCC_EnableAPB2PeriphClk(HDC_I2C_SDA_GPIO_CLK, ENABLE);
    GPIO_InitStructure.Pin        = HDC_I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitPeripheral(HDC_I2C_SDA_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.Pin        = HDC_I2C_SCL_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitPeripheral(HDC_I2C_SCL_PORT, &GPIO_InitStructure);
    RCC_EnableAPB2PeriphReset(HDC_I2C_SDA_GPIO_CLK, DISABLE);
    
    I2C_DeInit(HDC_I2C);
}


static int i2c_master_send(uint8_t* data, int len)
{
    uint8_t* sendBufferPtr = data;
    I2CTimeout             = I2C_LONG_TIMEOUT;
    while (I2C_GetFlag(HDC_I2C, I2C_FLAG_BUSY))
    {
        if ((I2CTimeout--) == 0)
            return 4;
    };

    I2C_ConfigAck(HDC_I2C, ENABLE);

    I2C_GenerateStart(HDC_I2C, ENABLE);
    I2CTimeout = I2C_SET_TIMEOUT;
    while (!I2C_CheckEvent(HDC_I2C, I2C_EVT_MASTER_MODE_FLAG)) // EV5
    {
        if ((I2CTimeout--) == 0)
            return 5;
    };

    I2C_SendAddr7bit(HDC_I2C, I2C_SLAVE_ADDR, I2C_DIRECTION_SEND);
    I2CTimeout = I2C_SET_TIMEOUT;
    while (!I2C_CheckEvent(HDC_I2C, I2C_EVT_MASTER_TXMODE_FLAG)) // EV6
    {
        if ((I2CTimeout--) == 0)
            return 6;
    };

    // send data
    while (len-- > 0)
    {
        I2C_SendData(HDC_I2C, *sendBufferPtr++);
        I2CTimeout = I2C_SET_TIMEOUT;
        while (!I2C_CheckEvent(HDC_I2C, I2C_EVT_MASTER_DATA_SENDING)) // EV8
        {
            if ((I2CTimeout--) == 0)
                return 7;
        };
    };

    I2CTimeout = I2C_SET_TIMEOUT;
    while (!I2C_CheckEvent(HDC_I2C, I2C_EVT_MASTER_DATA_SENDED)) // EV8-2
    {
        if ((I2CTimeout--) == 0)
            return 8;
    };
    // Delay_us(8);
    I2C_GenerateStop(HDC_I2C, ENABLE);
    I2C_RecvData(HDC_I2C);
    return 0;
}

static int i2c_master_recv(uint8_t* data, int len)
{
    uint8_t* recvBufferPtr = data;
    I2CTimeout             = I2C_LONG_TIMEOUT;
    while (I2C_GetFlag(HDC_I2C, I2C_FLAG_BUSY))
    {
        if ((I2CTimeout--) == 0)
            return 9;
    };

    I2C_ConfigAck(HDC_I2C, ENABLE);

    // send start
    I2C_GenerateStart(HDC_I2C, ENABLE);
    I2CTimeout = I2C_SET_TIMEOUT;
    while (!I2C_CheckEvent(HDC_I2C, I2C_EVT_MASTER_MODE_FLAG)) // EV5
    {
        if ((I2CTimeout--) == 0)
            return 10;
    };

    // send addr
    I2C_SendAddr7bit(HDC_I2C, I2C_SLAVE_ADDR, I2C_DIRECTION_RECV);
    I2CTimeout = I2C_SET_TIMEOUT;
    while (!I2C_CheckEvent(HDC_I2C, I2C_EVT_MASTER_RXMODE_FLAG)) // EV6
    {
        if ((I2CTimeout--) == 0)
            return 6;
    };
    
    // recv data
    while (len-- > 0)
    {
        if (len == 0)
        {
            I2C_ConfigAck(HDC_I2C, DISABLE);
            I2C_GenerateStop(HDC_I2C, ENABLE);
        }
        I2CTimeout = I2C_LONG_TIMEOUT;
        while (!I2C_CheckEvent(HDC_I2C, I2C_EVT_MASTER_DATA_RECVD_FLAG)) // EV7
        {
            if ((I2CTimeout--) == 0)
                return 14;
        };
        *recvBufferPtr++ = I2C_RecvData(HDC_I2C);
    };
    return 0;
}

void hdc_i2c_write_data(uint8_t data, uint8_t addr)
{
    uint8_t cmd[2] = {addr, data};
    i2c_master_send(cmd, 2);
}

uint8_t hdc_i2c_read_data(uint8_t addr)
{
    uint8_t rdata;
    i2c_master_send(&addr, 1);
    i2c_master_recv(&rdata, 1);
    return rdata;
}

void hdc_i2c_read_ndata(uint8_t addr, uint8_t len, uint8_t *data)
{
    i2c_master_send(&addr, 1);
    i2c_master_recv(data, len);
}

#else
#define I2C_DELAY 10

// static void i2c_init(void);

void hdc_i2c_gpio_init(void)
{

    GPIO_InitType GPIO_InitStructure;

    RCC_EnableAPB2PeriphClk(HDC_I2C_SDA_GPIO_CLK | HDC_I2C_SCL_GPIO_CLK, ENABLE);

    GPIO_InitStructure.Pin        = HDC_I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitPeripheral(HDC_I2C_SDA_PORT, &GPIO_InitStructure);
    GPIO_SetBits(HDC_I2C_SDA_PORT, HDC_I2C_SDA_PIN);

    GPIO_InitStructure.Pin        = HDC_I2C_SCL_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitPeripheral(HDC_I2C_SCL_PORT, &GPIO_InitStructure);
    GPIO_SetBits(HDC_I2C_SCL_PORT, HDC_I2C_SCL_PIN);
}


void hdc_i2c_gpio_deinit(void)
{
    GPIO_InitType GPIO_InitStructure;
    
    RCC_EnableAPB2PeriphClk(HDC_I2C_SDA_GPIO_CLK, ENABLE);
    GPIO_InitStructure.Pin        = HDC_I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitPeripheral(HDC_I2C_SDA_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.Pin        = HDC_I2C_SCL_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitPeripheral(HDC_I2C_SCL_PORT, &GPIO_InitStructure);
    RCC_EnableAPB2PeriphReset(HDC_I2C_SDA_GPIO_CLK, DISABLE);
}

void HDC_SDA_in(void)
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStructure.Pin        = HDC_I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(HDC_I2C_SDA_PORT, &GPIO_InitStructure);
}

void HDC_SDA_out(void)
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStructure.Pin        = HDC_I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(HDC_I2C_SDA_PORT, &GPIO_InitStructure);
}

// static void i2c_init(void)
//{
//    HDC_I2C_SCL_1();
//    HDC_I2C_SDA_1();
//}

static void i2c_start(void)
{
    HDC_I2C_SDA_OUT(); // sda线输出
    HDC_I2C_SDA_1();
    HDC_I2C_SCL_1();
    delay_us(I2C_DELAY);
    HDC_I2C_SDA_0(); // START:when CLK is high,DATA change form high to low
    delay_us(I2C_DELAY);
    ;
    HDC_I2C_SCL_0(); //钳住I2C总线，准备发送或接收数据
}

static void i2c_stop(void)
{
    HDC_I2C_SDA_OUT(); // sda线输出
    HDC_I2C_SCL_0();
    HDC_I2C_SDA_0(); // STOP:when CLK is high DATA change form low to high
    delay_us(I2C_DELAY);
    HDC_I2C_SCL_1();
    HDC_I2C_SDA_1(); //发送I2C总线结束信号
    delay_us(I2C_DELAY);
}

static uint8_t i2c_wait_ack(void)
{
    uint8_t ucErrTime = 0;
    HDC_I2C_SDA_1();
    HDC_I2C_SDA_IN(); // SDA设置为输入
    delay_us(I2C_DELAY);
    HDC_I2C_SCL_1();
    delay_us(I2C_DELAY);
    while (HDC_I2C_SDA_READ)
    {
        ucErrTime++;
        if (ucErrTime > 250)
        {
            i2c_stop();
            return 1;
        }
    }
    HDC_I2C_SCL_0(); //时钟输出0
    return 0;
}

static void i2c_send_ack(void)
{
    HDC_I2C_SCL_0();
    HDC_I2C_SDA_OUT();
    HDC_I2C_SDA_0();
    delay_us(I2C_DELAY);
    HDC_I2C_SCL_1();
    delay_us(I2C_DELAY);
    HDC_I2C_SCL_0();
}

////不产生ACK应答
// static void i2c_send_nack(void)
//{
//  HDC_I2C_SCL_0();
//  HDC_I2C_SDA_OUT();
//  HDC_I2C_SDA_1();
//  delay_us(I2C_DELAY);
//  HDC_I2C_SCL_1();
//  delay_us(I2C_DELAY);
//  HDC_I2C_SCL_0();
//}

static uint8_t i2c_read_byte(void)
{
    unsigned char i, receive = 0;
    HDC_I2C_SDA_IN(); // SDA设置为输入

    for (i = 0; i < 8; i++)
    {
        HDC_I2C_SCL_0();
        delay_us(I2C_DELAY);
        HDC_I2C_SCL_1();
        receive <<= 1;
        if (HDC_I2C_SDA_READ)
            receive++;
        delay_us(I2C_DELAY);
    }

    return receive;
}

static void i2c_send_byte(uint8_t tdata)
{
    uint8_t t;
    HDC_I2C_SDA_OUT();
    HDC_I2C_SCL_0(); //拉低时钟开始数据传输
    for (t = 0; t < 8; t++)
    {
        if ((tdata & 0x80) >> 7)
            HDC_I2C_SDA_1();
        else
            HDC_I2C_SDA_0();
        tdata <<= 1;
        delay_us(I2C_DELAY); //对TEA5767这三个延时都是必须的
        HDC_I2C_SCL_1();
        delay_us(I2C_DELAY);
        HDC_I2C_SCL_0();
        delay_us(I2C_DELAY);
    }
}

void hdc_i2c_write_data(uint8_t data, uint8_t addr)
{
    i2c_start();                       //启动IIC总线
    i2c_send_byte(HDC_I2C_DRV_ADDR_W); //发送从器件地址，即：0010010 0 为写操作，向QMA7981里面写操作
    i2c_wait_ack();                    //检查应答位，每发一字节都要检查应答位
    i2c_send_byte(addr);               //发送数据地址
    i2c_wait_ack();
    i2c_send_byte(data); //发送待发数据
    i2c_wait_ack();
    i2c_stop(); //全部发完后停止
    delay_ms(1);
}

uint8_t hdc_i2c_read_data(uint8_t addr)
{
    uint8_t rdata;
    i2c_start();
    i2c_send_byte(HDC_I2C_DRV_ADDR_W);
    i2c_wait_ack();
    i2c_send_byte(addr);
    i2c_wait_ack();
    i2c_start();
    i2c_send_byte(HDC_I2C_DRV_ADDR_R);
    i2c_wait_ack();
    rdata = i2c_read_byte();
    i2c_send_ack();
    i2c_stop();
    delay_ms(1);
    return rdata;
}

void hdc_i2c_read_ndata(uint8_t addr, uint8_t len, uint8_t *data)
{
    uint8_t n;
    
    i2c_start();
    i2c_send_byte(HDC_I2C_DRV_ADDR_W);
    i2c_wait_ack();
    i2c_send_byte(addr);
    i2c_wait_ack();
    i2c_start();
    i2c_send_byte(HDC_I2C_DRV_ADDR_R);
    
    i2c_wait_ack();
    for (n = 0; n < len - 1; n++)
    {
        data[n] = i2c_read_byte();
        i2c_send_ack();
    }
    data[n] = i2c_read_byte();
    i2c_stop();
    delay_ms(1);
}
#endif

