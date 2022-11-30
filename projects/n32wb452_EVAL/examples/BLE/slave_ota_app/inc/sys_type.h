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
 * @brief 智能锁的系统类型定义
 * @file sys_type.h
 * @author Nations
 * @version v1.0.1
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef __SYS_TYPE_H__
#define __SYS_TYPE_H__

#include <stdio.h>

/*!< N32G4FRX NATIONS Standard Peripheral Library old types (maintained for legacy purpose) */
#ifndef true
#define true 1
#endif // true

#ifndef false
#define false 0
#endif // false

#ifndef NULL
#define NULL ((void*)0)
#endif

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

#ifndef MIN
#define MIN(a, b) ((a) > (b) ? (b) : (a))
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

//typedef unsigned char bool;

// typedef int32_t  s32;
// typedef int16_t s16;
// typedef int8_t  s8;
#define REG8(addr)  (*(volatile u8*)(addr))
#define REG16(addr) (*(volatile u16*)(addr))
#define REG32(addr) (*(volatile u32*)(addr))

#if 0 
enum BITDEFINEenum
{
    BIT0  = (1 << 0),
    BIT1  = (1 << 1),
    BIT2  = (1 << 2),
    BIT3  = (1 << 3),
    BIT4  = (1 << 4),
    BIT5  = (1 << 5),
    BIT6  = (1 << 6),
    BIT7  = (1 << 7),
    BIT8  = (1 << 8),
    BIT9  = (1 << 9),
    BIT10 = (1 << 10),
    BIT11 = (1 << 11),
    BIT12 = (1 << 12),
    BIT13 = (1 << 13),
    BIT14 = (1 << 14),
    BIT15 = (1 << 15),
    BIT16 = (1 << 16),
    BIT17 = (1 << 17),
    BIT18 = (1 << 18),
    BIT19 = (1 << 19),
    BIT20 = (1 << 20),
    BIT21 = (1 << 21),
    BIT22 = (1 << 22),
    BIT23 = (1 << 23),
    BIT24 = (1 << 24),
    BIT25 = (1 << 25),
    BIT26 = (1 << 26),
    BIT27 = (1 << 27),
    BIT28 = (1 << 28),
    BIT29 = (1 << 29),
    BIT30 = (1 << 30),
    BIT31 = (int)(1U << 31)
};
#endif

//
//#define SET_BIT(reg,bit)  ((reg) |= (bit))
#define CLR_BIT(reg, bit) ((reg) &= ~(bit))
#define GET_BIT(reg, bit) ((reg) & (bit))

#define SET_REG_VAL(reg, val) ((*(volatile u32*)(reg)) = (val))
#define GET_REG_VAL(reg)      ((*(volatile u32*)(reg)))
#define SET_REG_BIT(reg, bit) ((*(volatile u32*)(reg)) |= (bit))
#define CLR_REG_BIT(reg, bit) ((*(volatile u32*)(reg)) &= ~(bit))
#define GET_REG_BIT(reg, bit) ((*(volatile u32*)(reg)) & (bit))

#define U32SWAP(value)                                                                                                                \
    ((((u32)value & 0xFF000000) >> 24) | (((u32)value & 0x00FF0000) >> 8) | (((u32)value & 0x0000FF00) << 8)                          \
     | (((u32)value & 0x000000FF) << 24))
#define U16SWAP(value) ((((u16)value & 0xFF00) >> 8) | (((u16)value & 0x00FF) << 8))

#ifdef _DEBUG_OUT_
#define printmsg printf
#else
#define printmsg(...)
#endif

#define __FPGA__
//#define __SDMA_EN__

#endif // __SYS_TYPE_H__
