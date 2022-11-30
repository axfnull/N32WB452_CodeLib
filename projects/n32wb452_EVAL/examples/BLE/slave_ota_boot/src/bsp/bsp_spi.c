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
 * @file bsp_spi.c
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include "n32wb452.h"
#include "bsp_spi.h"

void SPI_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;
    SPI_InitType SPI_InitStructure;

    RCC_EnableAPB2PeriphClk(SPI_FLASH_RCCCLK | RCC_APB2_PERIPH_AFIO, ENABLE);
    //GPIO_ConfigPinRemap(GPIO_RMP2_SPI2, ENABLE);
    
    /* Configure SCK and MOSI pins as Alternate Function Push Pull */
    GPIO_InitStructure.Pin        = SPI_FLASH_CLK_PIN | SPI_FLASH_MOSI_PIN | SPI_FLASH_MISO_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitPeripheral(SPI_FLASH_CLK_PORT, &GPIO_InitStructure);

    GPIO_SetBits(SPI_FLASH_CLK_PORT, SPI_FLASH_CLK_PIN);
    GPIO_SetBits(SPI_FLASH_MOSI_PORT, SPI_FLASH_MOSI_PIN);

    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_SPI3, ENABLE);

    /* SPI2 configuration ------------------------------------------------*/
    SPI_InitStructure.DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX;
    SPI_InitStructure.SpiMode       = SPI_MODE_MASTER;
    SPI_InitStructure.DataLen       = SPI_DATA_SIZE_8BITS;
    SPI_InitStructure.CLKPOL        = SPI_CLKPOL_LOW;
    SPI_InitStructure.CLKPHA        = SPI_CLKPHA_FIRST_EDGE;
    SPI_InitStructure.NSS           = SPI_NSS_SOFT;
    SPI_InitStructure.BaudRatePres  = SPI_BR_PRESCALER_4;
    SPI_InitStructure.FirstBit      = SPI_FB_MSB;
    SPI_InitStructure.CRCPoly       = 7;
    SPI_Init(SPI3, &SPI_InitStructure);

    /* Enable SPI2 */
    SPI_Enable(SPI3, ENABLE);
}

u8 SPI3_ReadWriteByte(u8 TxData)
{
    u16 retry = 0;
    u16 data;

    while (SPI_I2S_GetStatus(SPI3, SPI_I2S_TE_FLAG) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
    {
        retry++;
        if (retry > 400)
        {
            return 0;
        }
    }
    SPI_I2S_TransmitData(SPI3, TxData); //通过外设SPIx发送一个数据
    retry = 0;

    while (SPI_I2S_GetStatus(SPI3, SPI_I2S_RNE_FLAG) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
    {
        retry++;
        if (retry > 400)
        {
            return 0;
        }
    }
    data = SPI_I2S_ReceiveData(SPI3); //返回通过SPIx最近接收的数据

    return data;
}



