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
 * @file lcd_drv.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include <stdlib.h>
#include <stdio.h>
#include "n32wb452.h"
#include "lcd_drv.h"
#include "bsp_spi.h"
#include "n32wb452_spi.h"
#include "main.h"

lcd_attr_typedef lcd_attr;

uint8_t SPI_WriteByte(uint8_t data)
{
    uint16_t retry;

    SPI_I2S_TransmitData(SPI2, data); //通过外设SPIx发送一个数据
    retry = 0;
    while (SPI_I2S_GetStatus(SPI2, SPI_I2S_TE_FLAG) == RESET) //检查指定的SPI标志位设置与否:    发送缓存空标志位
    {
        retry++;
        if (retry > 1000)
        {
            return 0;
        }
    }
    return 1;
}

void SPI_Write_nByte(uint8_t* data, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        SPI_WriteByte(data[i]);
    }
}

uint8_t SPI_ReadByte(void)
{
    uint8_t retry;
    uint8_t data;
    while (SPI_I2S_GetStatus(SPI2, SPI_I2S_RNE_FLAG) == RESET) //检查指定的SPI标志位设置与否:    接受缓存非空标志位
    {
        retry++;
        if (retry > 200)
        {
            return 0;
        }
    }
    data = SPI_I2S_ReceiveData(SPI2); //返回通过SPIx最近接收的数据
    return data;
}

void lcd_gpio_init(void)
{
    GPIO_InitType GPIO_InitStructure;

    RCC_EnableAPB2PeriphClk(LCD_RST_GPIO_CLK | LCD_BLK_GPIO_CLK | LCD_DC_GPIO_CLK | LCD_CS_GPIO_CLK, ENABLE);

    GPIO_InitStructure.Pin        = LCD_RST_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitPeripheral(LCD_RST_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin        = LCD_BLK_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitPeripheral(LCD_BLK_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin        = LCD_DC_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitPeripheral(LCD_DC_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin        = LCD_CS_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitPeripheral(LCD_CS_PORT, &GPIO_InitStructure);

    LCD_SPI_CS_1();
    LCD_DC_1();

    SPI_Configuration();
}

void lcd_gpio_deint(void)
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStructure.Pin        = LCD_RST_PIN | LCD_DC_PIN | LCD_CS_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitPeripheral(LCD_RST_PORT, &GPIO_InitStructure);
}

void LCD_PowerOff(void)
{
    LCD_RST_0();
    LCD_BLK_0();
}

/******************************************************************************
function :  Hardware reset
parameter:
******************************************************************************/
static void LCD_Reset(void)
{
    LCD_RST_1();
    delay_ms(100);
    LCD_RST_0();
    delay_ms(100);
    LCD_RST_1();
    delay_ms(100);
}

/******************************************************************************
function :  send command
parameter:
     Reg : Command register
******************************************************************************/
static void LCD_SendCommand(uint8_t Reg)
{
    LCD_DC_0();
    LCD_SPI_CS_0();
    SPI_WriteByte(Reg);
    delay_us(10);
    LCD_SPI_CS_1();
}

/******************************************************************************
function :  send data
parameter:
    Data : Write data
******************************************************************************/
static void LCD_SendData_8Bit(uint8_t Data)
{
    LCD_DC_1();
    LCD_SPI_CS_0();
    SPI_WriteByte(Data);
    delay_us(10);
    LCD_SPI_CS_1();
}

/******************************************************************************
function :  send data
parameter:
    Data : Write data
******************************************************************************/
static void LCD_SendData_16Bit(uint16_t Data)
{
    LCD_DC_1();
    LCD_SPI_CS_0();
    SPI_WriteByte((Data >> 8) & 0xFF);
    SPI_WriteByte(Data & 0xFF);
    delay_us(10);
    LCD_SPI_CS_1();
}

/********************************************************************************
function:   Set the resolution and scanning method of the screen
parameter:
                Scan_dir:   Scan direction
********************************************************************************/
static void LCD_SetAttributes(E_SCAN_DIR Scan_dir)
{
    // Get the screen scan direction
    lcd_attr.scan_dir        = Scan_dir;
    uint16_t MemoryAccessReg = 0x00;

    // Get GRAM and LCD width and height
    if (Scan_dir == HORIZONTAL)
    {
        lcd_attr.height = LCD_HEIGHT;
        lcd_attr.width  = LCD_WIDTH;
        MemoryAccessReg = 0x70;
    }
    else
    {
        lcd_attr.height = LCD_WIDTH;
        lcd_attr.width  = LCD_HEIGHT;
        MemoryAccessReg = 0x00;
    }

    // Set the read / write scan direction of the frame memory
    LCD_SendCommand(LCD_MODE);          // MX, MY, RGB mode
    LCD_SendData_8Bit(MemoryAccessReg); // 0x08 set RGB
}

/******************************************************************************
function :  Initialize the lcd register
parameter:
******************************************************************************/
static void LCD_InitReg(void)
{
    LCD_SendCommand(0x11);
    delay_ms(120);
    // LCD_SendCommand(0x36);
    // LCD_SendData_8Bit(0x00);

    LCD_SendCommand(0x3A);
    LCD_SendData_8Bit(0x05);

    LCD_SendCommand(0xB2);
    LCD_SendData_8Bit(0x0C);
    LCD_SendData_8Bit(0x0C);
    LCD_SendData_8Bit(0x00);
    LCD_SendData_8Bit(0x33);
    LCD_SendData_8Bit(0x33);

    LCD_SendCommand(0xB7);
    LCD_SendData_8Bit(0x35);

    LCD_SendCommand(0xBB);
    LCD_SendData_8Bit(0x37);

    LCD_SendCommand(0xC0);
    LCD_SendData_8Bit(0x2C);

    LCD_SendCommand(0xC2);
    LCD_SendData_8Bit(0x01);

    LCD_SendCommand(0xC3);
    LCD_SendData_8Bit(0x12);

    LCD_SendCommand(0xC4);
    LCD_SendData_8Bit(0x20);

    LCD_SendCommand(0xC6);
    LCD_SendData_8Bit(0x0F);

    LCD_SendCommand(0xD0);
    LCD_SendData_8Bit(0xA4);
    LCD_SendData_8Bit(0xA1);

    LCD_SendCommand(0xE0);
    LCD_SendData_8Bit(0xD0);
    LCD_SendData_8Bit(0x04);
    LCD_SendData_8Bit(0x0D);
    LCD_SendData_8Bit(0x11);
    LCD_SendData_8Bit(0x13);
    LCD_SendData_8Bit(0x2B);
    LCD_SendData_8Bit(0x3F);
    LCD_SendData_8Bit(0x54);
    LCD_SendData_8Bit(0x4C);
    LCD_SendData_8Bit(0x18);
    LCD_SendData_8Bit(0x0D);
    LCD_SendData_8Bit(0x0B);
    LCD_SendData_8Bit(0x1F);
    LCD_SendData_8Bit(0x23);

    LCD_SendCommand(0xE1);
    LCD_SendData_8Bit(0xD0);
    LCD_SendData_8Bit(0x04);
    LCD_SendData_8Bit(0x0C);
    LCD_SendData_8Bit(0x11);
    LCD_SendData_8Bit(0x13);
    LCD_SendData_8Bit(0x2C);
    LCD_SendData_8Bit(0x3F);
    LCD_SendData_8Bit(0x44);
    LCD_SendData_8Bit(0x51);
    LCD_SendData_8Bit(0x2F);
    LCD_SendData_8Bit(0x1F);
    LCD_SendData_8Bit(0x1F);
    LCD_SendData_8Bit(0x20);
    LCD_SendData_8Bit(0x23);

    LCD_SendCommand(0x21);

    LCD_SendCommand(0x29);
}

/********************************************************************************
function :  Initialize the lcd
parameter:
********************************************************************************/
void lcd_init(E_SCAN_DIR Scan_dir)
{
    // Turn on the backlight
    LCD_BLK_1();

    // Hardware reset
    LCD_Reset();

    // Set the resolution and scanning method of the screen
    LCD_SetAttributes(Scan_dir);

    // Set the initialization register
    LCD_InitReg();
}

/********************************************************************************
function:   Sets the start position and size of the display area
parameter:
                Xstart  :   X direction Start coordinates
                Ystart  :   Y direction Start coordinates
                Xend    :   X direction end coordinates
                Yend    :   Y direction end coordinates
********************************************************************************/
void LCD_SetWindows(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend)
{
    // set the X coordinates
    LCD_SendCommand(0x2A);
    LCD_SendData_8Bit((Xstart >> 8) & 0xFF);
    LCD_SendData_8Bit(Xstart & 0xFF);
    LCD_SendData_8Bit(((Xend - 1) >> 8) & 0xFF);
    LCD_SendData_8Bit((Xend - 1) & 0xFF);

    // set the Y coordinates
    LCD_SendCommand(0x2B);
    LCD_SendData_8Bit((Ystart >> 8) & 0xFF);
    LCD_SendData_8Bit(Ystart & 0xFF);
    LCD_SendData_8Bit(((Yend - 1) >> 8) & 0xFF);
    LCD_SendData_8Bit((Yend - 1) & 0xFF);

    LCD_SendCommand(0X2C);
}

/******************************************************************************
function :  Clear screen
parameter:
******************************************************************************/
void LCD_Clear(uint16_t Color)
{
    uint16_t i, j;
    // uint16_t Image[LCD_WIDTH*LCD_HEIGHT];

    Color = ((Color << 8) & 0xff00) | (Color >> 8);

    // for (j = 0; j < LCD_HEIGHT*LCD_WIDTH; j++)
    {
        //    Image[j] = Color;
    }

    LCD_SetWindows(0, 0, LCD_WIDTH, LCD_HEIGHT);
    LCD_DC_1();
    LCD_SPI_CS_0();
    for (j = 0; j < LCD_HEIGHT; j++)
    {
        // SPI_Write_nByte((uint8_t *)&Image[j*LCD_WIDTH], LCD_WIDTH*2);
        for (i = 0; i < LCD_WIDTH; i++)
            SPI_Write_nByte((uint8_t*)&Color, 2);
    }
    LCD_SPI_CS_1();
}

/******************************************************************************
function :  Sends the image buffer in RAM to displays
parameter:
******************************************************************************/
void LCD_Display(uint16_t* Image)
{
    uint16_t j;
    LCD_SetWindows(0, 0, LCD_WIDTH, LCD_HEIGHT);
    LCD_DC_1();
    LCD_SPI_CS_0();
    for (j = 0; j < LCD_HEIGHT; j++)
    {
        SPI_Write_nByte((uint8_t*)&Image[j * LCD_WIDTH], LCD_WIDTH * 2);
    }
    LCD_SPI_CS_1();
}

void LCD_DisplayWindows(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend, uint16_t* Image)
{
    // display
    uint32_t Addr = 0;

    uint16_t j;
    LCD_SetWindows(Xstart, Ystart, Xend - 1, Yend - 1);
    LCD_DC_1();
    LCD_SPI_CS_0();
    for (j = Ystart; j < Yend - 1; j++)
    {
        Addr = Xstart + j * LCD_WIDTH;
        SPI_Write_nByte((uint8_t*)&Image[Addr], (Xend - Xstart - 1) * 2);
    }
    LCD_SPI_CS_1();
}

void LCD_DisplayPoint(uint16_t X, uint16_t Y, uint16_t Color)
{
    LCD_SetWindows(X, Y, X, Y);
    LCD_SendData_16Bit(Color);
}
