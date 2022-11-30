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
 * @file i2c_drv.h
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __I2C_DRV_H__
#define __I2C_DRV_H__

#include <stdint.h>

#include <stdlib.h>
#include <stdio.h>

#define QMA_I2C I2C1


#define QMA_I2C_CLOCK   (400000)    //400K

#define QMA_I2C_DRV_ADDR (0x12)

#define QMA_I2C_DRV_ADDR_W (QMA_I2C_DRV_ADDR << 1)
#define QMA_I2C_DRV_ADDR_R ((QMA_I2C_DRV_ADDR << 1) | 0x01)

#define QMA_I2C_SDA_GPIO_CLK RCC_APB2_PERIPH_GPIOB
#define QMA_I2C_SDA_PORT     GPIOB
#define QMA_I2C_SDA_PIN      GPIO_PIN_9

#define QMA_I2C_SCL_GPIO_CLK RCC_APB2_PERIPH_GPIOB
#define QMA_I2C_SCL_PORT     GPIOB
#define QMA_I2C_SCL_PIN      GPIO_PIN_8

#define QMA_I2C_SDA_1() GPIO_SetBits(QMA_I2C_SDA_PORT, QMA_I2C_SDA_PIN)
#define QMA_I2C_SDA_0() GPIO_ResetBits(QMA_I2C_SDA_PORT, QMA_I2C_SDA_PIN)

#define QMA_I2C_SCL_1() GPIO_SetBits(QMA_I2C_SCL_PORT, QMA_I2C_SCL_PIN)
#define QMA_I2C_SCL_0() GPIO_ResetBits(QMA_I2C_SCL_PORT, QMA_I2C_SCL_PIN)

//#define QMA_I2C_SDA_IN()  (SDA_in())
//#define QMA_I2C_SDA_OUT() (SDA_out())

#define QMA_I2C_SDA_READ GPIO_ReadInputDataBit(QMA_I2C_SDA_PORT, QMA_I2C_SDA_PIN)

int qma_i2c_gpio_init(void);
void qma_i2c_gpio_deinit(void);
void qma_i2c_write_data(uint8_t data, uint8_t addr);
uint8_t qma_i2c_read_data(uint8_t addr);
void qma_i2c_read_ndata(uint8_t addr, uint8_t len, uint8_t *data);

#endif //__I2C_DRV_H__
