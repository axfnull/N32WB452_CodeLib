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
 ******************************************************************************
 * @file n32g4FRx_smartlock_w25qxx.h
 * @author Nations
 * @version v1.0.0
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 * @date    2019-12-20
 * @brief 文件提供智能门锁中w25qxx系列读写擦操作功能头文件
 *
 ******************************************************************************
 */
#ifndef __N32G4FRX_SMARTLOCK_W25QXX_H__
#define __N32G4FRX_SMARTLOCK_W25QXX_H__
#include "n32wb452.h"

//W25X系列/Q系列芯片列表
#define W25Q80 0XEF13
#define W25Q16 0XEF14
#define W25Q32 0XEF15
#define W25Q64 0XEF16
#define W25Q128 0XEF17

extern uint16_t W25QXX_TYPE; //定义W25QXX芯片型号

////////////////////////////////////////////////////////////////////////////

//指令表
#define W25X_WriteEnable 0x06
#define W25X_WriteDisable 0x04
#define W25X_ReadStatusReg 0x05
#define W25X_WriteStatusReg 0x01
#define W25X_ReadData 0x03
#define W25X_FastReadData 0x0B
#define W25X_FastReadDual 0x3B
#define W25X_PageProgram 0x02
#define W25X_BlockErase 0xD8
#define W25X_SectorErase 0x20
#define W25X_ChipErase 0xC7
#define W25X_PowerDown 0xB9
#define W25X_ReleasePowerDown 0xAB
#define W25X_DeviceID 0xAB
#define W25X_ManufactDeviceID 0x90
#define W25X_JedecDeviceID 0x9F

void W25QXX_Init(void);
uint16_t W25QXX_ReadID(void);     //读取FLASH ID
uint8_t W25QXX_ReadSR(void);      //读取状态寄存器
void W25QXX_Write_SR(uint8_t sr); //写状态寄存器
void W25QXX_Write_Enable(void);   //写使能
void W25QXX_Write_Disable(void);  //写保护
void W25QXX_Write_NoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void W25QXX_Read(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);    //读取flash
void W25QXX_Write(uint8_t* pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite); //写入flash
void W25QXX_Clear(u32 ReadAddr, uint32_t NumByteToWrite);
void W25QXX_Erase_Chip(void);                                                     //整片擦除
void W25QXX_Erase_Sector(uint32_t Dst_Addr);                                      //扇区擦除
void W25QXX_Wait_Busy(void);                                                      //等待空闲
void W25QXX_PowerDown(void);                                                      //进入掉电模式
void W25QXX_WAKEUP(void);                                                         //唤醒

#endif //__N32G4FRX_SMARTLOCK_W25QXX_H__


