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
 * @file n32wb452_ota_iap.c
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include "n32wb452.h"
#include "n32wb452_ota_iap.h"
#include "n32wb452_ota_conf.h"


#define APP_FLASH_ADDR_OFFSET (OF_BOOTLOADER_SIZE)

void set_vector_table(void)
{
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, APP_FLASH_ADDR_OFFSET);
}

/*********************************************************************
 *函数名：iap_jump_to_app
 *描述  ：执行跳入APP
 *输入  ：appAddr
 *输出  ：无
 *返回值：无
 *说明:
***********************************************************************/
uint8_t jump2app(uint32_t appAddr)
{
    uint32_t app_msp_addr;
    uint32_t app_jump_addr;
    void  (*pAppFun)(void);     //定义一个函数指针，用于指向app程序入口

    //FLASH_Lock();

    app_msp_addr = (*(__IO uint32_t*)appAddr);         //取栈顶地址保存的数据
    app_jump_addr = (*(__IO uint32_t*)(appAddr + 4));  //取出APP程序复位中断向量的地址

    /* 检查app栈顶地址合法性，判断是否已经下载程序 */
//    if ((app_msp_addr & 0x2FFDC000) != 0x20000000)  
//        return 1;

    if (app_msp_addr < 0x20000000)
        return 1;
    if (app_msp_addr > 0x20023FFF)
        return 1;
    if (app_msp_addr & 0x00000003)
        return 1;
        
    pAppFun = (void (*)(void))app_jump_addr;        //生成跳转函数

    //NVIC_SetVectorTable(NVIC_VectTab_FLASH, APP_FLASH_ADDR_OFFSET);

    __set_MSP(app_msp_addr);    //设置MSP指针指向app向量表的栈顶地址保存的地址
    //__set_PSP(app_msp_addr);
    (*pAppFun)();               //跳转，PC指针执行复位
    
    return 0;
}


/********************************************************************
函数: 跳入IAP
输入: 无
返回: 无.
说明:
************************************************************************/
void jump2iap(uint32_t iapAddr)
{
#if 0
    uint32_t  IapSpInitVal;
    uint32_t  IapJumpAddr;          //IAP程序的跳转地址.即,IAP程序的入口.
    void (*pIapFun)(void);          //定义一个函数指针.用于指向APP程序入口.

    //RCC_DeInit();
    //nvic_reset();               //恢复NVIC为复位状态.使中断不再发生.

    //__set_CONTROL(0);                       //将PSP指针切换为MSP指针

    IapSpInitVal = *(__IO uint32_t*)iapAddr;       //取APP的SP初值.
    IapJumpAddr = *(__IO uint32_t*)(iapAddr + 4);  //取程序入口.

    __set_MSP(IapSpInitVal);                     //设置SP.
    //__set_PSP(IapSpInitVal);

    pIapFun = (void (*)(void))IapJumpAddr;       //生成跳转函数.
    (*pIapFun)();                                //跳转.不再返回.

#else

    __set_FAULTMASK(1);       //屏蔽所有中断
    NVIC_SystemReset();       //软件复位

#endif
}

