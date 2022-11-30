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
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __HDC_I2C_DRV_H__
#define __HDC_I2C_DRV_H__

#include <stdint.h>

#include <stdlib.h>
#include <stdio.h>
#include "n32wb452.h"

#define HDC_I2C I2C2

#define HDC_I2C_CLOCK   (400000)    //400K

#define HDC_I2C_DRV_ADDR (0x40)

#define HDC_I2C_DRV_ADDR_W (HDC_I2C_DRV_ADDR << 1)
#define HDC_I2C_DRV_ADDR_R ((HDC_I2C_DRV_ADDR << 1) | 0x01)

#define HDC_I2C_SDA_GPIO_CLK RCC_APB2_PERIPH_GPIOB
#define HDC_I2C_SDA_PORT     GPIOB
#define HDC_I2C_SDA_PIN      GPIO_PIN_11

#define HDC_I2C_SCL_GPIO_CLK RCC_APB2_PERIPH_GPIOB
#define HDC_I2C_SCL_PORT     GPIOB
#define HDC_I2C_SCL_PIN      GPIO_PIN_10

#define HDC_I2C_SDA_1() GPIO_SetBits(HDC_I2C_SDA_PORT, HDC_I2C_SDA_PIN)
#define HDC_I2C_SDA_0() GPIO_ResetBits(HDC_I2C_SDA_PORT, HDC_I2C_SDA_PIN)

#define HDC_I2C_SCL_1() GPIO_SetBits(HDC_I2C_SCL_PORT, HDC_I2C_SCL_PIN)
#define HDC_I2C_SCL_0() GPIO_ResetBits(HDC_I2C_SCL_PORT, HDC_I2C_SCL_PIN)

#define HDC_I2C_SDA_IN()  (HDC_SDA_in())
#define HDC_I2C_SDA_OUT() (HDC_SDA_out())

#define HDC_I2C_SDA_READ GPIO_ReadInputDataBit(HDC_I2C_SDA_PORT, HDC_I2C_SDA_PIN)

int hdc_i2c_gpio_init(void);
void hdc_i2c_gpio_deinit(void);
void hdc_i2c_write_data(uint8_t data, uint8_t addr);
uint8_t hdc_i2c_read_data(uint8_t addr);
void hdc_i2c_read_ndata(uint8_t addr, uint8_t len, uint8_t *data);

#endif //__HDC_I2C_DRV_H__
