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
 * @file lcd_drv.h
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __LCD_DRV_H__
#define __LCD_DRV_H__

#include <stdint.h>

#include <stdlib.h> //itoa()
#include <stdio.h>

#define LCD_RST_GPIO_CLK RCC_APB2_PERIPH_GPIOE
#define LCD_RST_PORT     GPIOE
#define LCD_RST_PIN      GPIO_PIN_7

#define LCD_BLK_GPIO_CLK RCC_APB2_PERIPH_GPIOE
#define LCD_BLK_PORT     GPIOE
#define LCD_BLK_PIN      GPIO_PIN_8

#define LCD_DC_GPIO_CLK RCC_APB2_PERIPH_GPIOE
#define LCD_DC_PORT     GPIOE
#define LCD_DC_PIN      GPIO_PIN_9

#define LCD_CS_GPIO_CLK RCC_APB2_PERIPH_GPIOE
#define LCD_CS_PORT     GPIOE
#define LCD_CS_PIN      GPIO_PIN_10

#define LCD_RST_1() GPIO_SetBits(LCD_RST_PORT, LCD_RST_PIN)
#define LCD_RST_0() GPIO_ResetBits(LCD_RST_PORT, LCD_RST_PIN)

#define LCD_BLK_1() GPIO_SetBits(LCD_BLK_PORT, LCD_BLK_PIN)
#define LCD_BLK_0() GPIO_ResetBits(LCD_BLK_PORT, LCD_BLK_PIN)

#define LCD_DC_1() GPIO_SetBits(LCD_DC_PORT, LCD_DC_PIN)
#define LCD_DC_0() GPIO_ResetBits(LCD_DC_PORT, LCD_DC_PIN)

#define LCD_SPI_CS_1() GPIO_SetBits(LCD_CS_PORT, LCD_CS_PIN)
#define LCD_SPI_CS_0() GPIO_ResetBits(LCD_CS_PORT, LCD_CS_PIN)

#define LCD_HEIGHT 240
#define LCD_WIDTH  240

#define LCD_WIDTH_Byte 240

typedef enum
{
    HORIZONTAL = 0,
    VERTICAL   = 1
} E_SCAN_DIR;

typedef __packed struct
{
    E_SCAN_DIR scan_dir;
    uint16_t width;
    uint16_t height;
} lcd_attr_typedef;

#define LCD_MODE 0x36

extern lcd_attr_typedef lcd_attr;

void lcd_gpio_init(void);
void lcd_gpio_deint(void);
void LCD_PowerOff(void);
void lcd_init(E_SCAN_DIR Scan_dir);
void LCD_Clear(uint16_t Color);
void LCD_Display(uint16_t* Image);
void LCD_DisplayWindows(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend, uint16_t* Image);
void LCD_DisplayPoint(uint16_t X, uint16_t Y, uint16_t Color);

#endif //__LCD_DRV_H__
