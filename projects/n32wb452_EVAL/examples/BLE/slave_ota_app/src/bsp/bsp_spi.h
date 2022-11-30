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
 * @file bsp_spi.h
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__

#define SPI_FLASH_RCCCLK (RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB)

#define SPI_FLASH_CS_PORT   GPIOA //
#define SPI_FLASH_CLK_PORT  GPIOB //
#define SPI_FLASH_MOSI_PORT GPIOB //
#define SPI_FLASH_MISO_PORT GPIOB //

#define SPI_FLASH_CS_PIN    GPIO_PIN_15 //
#define SPI_FLASH_CLK_PIN   GPIO_PIN_3 //
#define SPI_FLASH_MOSI_PIN  GPIO_PIN_5 //
#define SPI_FLASH_MISO_PIN  GPIO_PIN_4 //

#define __SPI_FLASH_CS_SET() GPIO_SetBits(SPI_FLASH_CS_PORT, SPI_FLASH_CS_PIN)
#define __SPI_FLASH_CS_CLR() GPIO_ResetBits(SPI_FLASH_CS_PORT, SPI_FLASH_CS_PIN)

void SPI_Configuration(void);
void SPI_DisConfiguration(void);
u8 SPI3_ReadWriteByte(u8 TxData);

#endif //__BSP_SPI_H__
