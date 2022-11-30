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
 * @file spi_flash.h
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __SPI_FLASH_H__
#define __SPI_FLASH_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32wb452.h"

/** @addtogroup Utilities
 * @{
 */

/** @addtogroup
 * @{
 */

/** @addtogroup Common
 * @{
 */

/** @addtogroup SPI_FLASH
 * @{
 */

/** @addtogroup SPI_FLASH_Exported_Types
 * @{
 */
/**
 * @}
 */

/** @addtogroup SPI_FLASH_Exported_Constants
 * @{
 */
/**
 * @brief  SPI Flash supported commands
 */
#define sFLASH_CMD_WRITE 0x02 /*!< Write to Memory instruction */
#define sFLASH_CMD_WRSR  0x01 /*!< Write Status Register instruction */
#define sFLASH_CMD_WREN  0x06 /*!< Write enable instruction */
#define sFLASH_CMD_READ  0x03 /*!< Read from Memory instruction */
#define sFLASH_CMD_RDSR  0x05 /*!< Read Status Register instruction  */
#define sFLASH_CMD_RDID  0x9F /*!< Read identification */
#define sFLASH_CMD_SE    0xD8 /*!< Sector Erase instruction */
#define sFLASH_CMD_BE    0xC7 /*!< Bulk Erase instruction */

#define sFLASH_WIP_FLAG 0x01 /*!< Write In Progress (WIP) flag */

#define sFLASH_DUMMY_BYTE   0xA5
#define sFLASH_SPI_PAGESIZE 0x100

#define sFLASH_W25Q128_ID       0x00EF4018
#define sFLASH_W25Q128_ID_DTR   0x00EF7018


#define sFLASH_SPI_DMA            DMA2
#define sFLASH_SPI_DMA_CLK        RCC_AHB_PERIPH_DMA2
#define sFLASH_SPI_Rx_DMA_Channel DMA2_CH1
#define sFLASH_SPI_Rx_DMA_FLAG    DMA2_FLAG_TC1
#define sFLASH_SPI_Tx_DMA_Channel DMA2_CH2
#define sFLASH_SPI_Tx_DMA_FLAG    DMA2_FLAG_TC2

#define SPI_MASTER_DR_Base 0x40003C0C

#define sFLASH_SPI                SPI3
#define sFLASH_SPI_CLK            RCC_APB1_PERIPH_SPI3
#define sFLASH_SPI_SCK_PIN        GPIO_PIN_3 /* PB.03 */
#define sFLASH_SPI_SCK_GPIO_PORT  GPIOB      /* GPIOB */
#define sFLASH_SPI_SCK_GPIO_CLK   RCC_APB2_PERIPH_GPIOB
#define sFLASH_SPI_MISO_PIN       GPIO_PIN_4 /* PB.04 */
#define sFLASH_SPI_MISO_GPIO_PORT GPIOB      /* GPIOB */
#define sFLASH_SPI_MISO_GPIO_CLK  RCC_APB2_PERIPH_GPIOB
#define sFLASH_SPI_MOSI_PIN       GPIO_PIN_5 /* PB.05 */
#define sFLASH_SPI_MOSI_GPIO_PORT GPIOB      /* GPIOB */
#define sFLASH_SPI_MOSI_GPIO_CLK  RCC_APB2_PERIPH_GPIOB
#define sFLASH_CS_PIN             GPIO_PIN_15 /* PA.15 */
#define sFLASH_CS_GPIO_PORT       GPIOA      /* GPIOA */
#define sFLASH_CS_GPIO_CLK        RCC_APB2_PERIPH_GPIOA
/**
 * @}
 */

/** @addtogroup SPI_FLASH_Exported_Macros
 * @{
 */
/**
 * @brief  Select sFLASH: Chip Select pin low
 */
#define sFLASH_CS_LOW() GPIO_ResetBits(sFLASH_CS_GPIO_PORT, sFLASH_CS_PIN)
/**
 * @brief  Deselect sFLASH: Chip Select pin high
 */
#define sFLASH_CS_HIGH() GPIO_SetBits(sFLASH_CS_GPIO_PORT, sFLASH_CS_PIN)
/**
 * @}
 */

/** @addtogroup SPI_FLASH_Exported_Functions
 * @{
 */
/**
 * @brief  High layer functions
 */
void sFLASH_DeInit(void);
void sFLASH_Init(void);
void sFLASH_EraseSector(uint32_t SectorAddr);
void sFLASH_EraseBulk(void);
void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint32_t sFLASH_ReadID(void);
void sFLASH_StartReadSequence(uint32_t ReadAddr);

/**
 * @brief  Low layer functions
 */
uint8_t sFLASH_ReadByte(void);
uint8_t sFLASH_SendByte(uint8_t byte);
uint16_t sFLASH_SendHalfWord(uint16_t HalfWord);
void sFLASH_WriteEnable(void);
void sFLASH_WaitForWriteEnd(void);

#ifdef __cplusplus
}
#endif

#endif
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
