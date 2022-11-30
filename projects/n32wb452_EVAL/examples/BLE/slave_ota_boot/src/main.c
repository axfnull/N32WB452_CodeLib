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
 * @file main.c
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "n32wb452.h"
#include "n32wb452_flash.h"
#include "n32wb452_ota_iap.h"
#include "n32wb452_ota_conf.h"
#include "n32wb452_ota_upgrade.h"
#include "n32wb452_w25qxx.h"
#include "log.h"
#include "n32wb452_ota_iap.h"
#include "n32wb452_flash_interface.h"
#include "crc16.h"

void RCC_Configuration(void);

void delay_us(int32_t t)
{
    while (t--)
        for (uint16_t m = 0; m < 13; m++)
            ;
}

void delay_ms(int32_t t)
{
    while (t--)
        delay_us(1000);
}

/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    /* PCLK1 = HCLK/4 */
    RCC_ConfigPclk1(RCC_HCLK_DIV4);

    /* TIM2 clock enable */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM2, ENABLE);

    /* GPIOC clock enable */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOC, ENABLE);
}

void SetSysClock_HSE_PLL(uint32_t pllmul)
{
    __IO uint32_t StartUpCounter = 0, HSEStartUpStatus = 0;

    // It is necessary to initialize the RCC peripheral to the reset state.
    RCC_DeInit();

    // Enable HSE, open external crystal oscillator.
    RCC_ConfigHse(RCC_HSE_ENABLE);

    // Wait for HSE to be stable.
    HSEStartUpStatus = RCC_WaitHseStable();

    // Go on until the HSE is stable.
    if (HSEStartUpStatus == SUCCESS)
    {
        //----------------------------------------------------------------------//
        // Enable flash Prefetch buffer
        FLASH_PrefetchBufSet(FLASH_PrefetchBuf_EN);

        // 0:HCLK <= 32M
        // 1:HCLK <= 64M
        // 2:HCLK <= 96M
        // 3:HCLK <= 128M
        // 4:HCLK <= 144M
        FLASH_SetLatency(FLASH_LATENCY_4);
        //----------------------------------------------------------------------//

        // AHB prescaler factor set to 1,HCLK = SYSCLK = 144M
        RCC_ConfigHclk(RCC_SYSCLK_DIV1);
        // AHB prescaler factor set to 2,PCLK2 = HCLK/2 = 72M
        RCC_ConfigPclk2(RCC_HCLK_DIV2);
        // AHB prescaler factor set to 4,PCLK1 = HCLK/4 = 36M
        RCC_ConfigPclk1(RCC_HCLK_DIV4);

        ////-----------------Set PLL clock source as HSE, set PLL frequency multiplication factor.-------------------//
        // PLLCLK = 8MHz * pllmul
        RCC_ConfigPll(RCC_PLL_SRC_HSE_DIV1, pllmul);
        ////------------------------------------------------------------------//

        // Enable PLL
        RCC_EnablePll(ENABLE);
        // Wait for PLL to be stable.
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRD) == RESET)
        {
        }
        RCC_ConfigSysclk(RCC_SYSCLK_SRC_PLLCLK);

        // Switch PLL clock to SYSCLK.
        RCC_ConfigSysclk(RCC_SYSCLK_SRC_PLLCLK);

        // Read the clock switch status bit and make sure pllclk is selected as the system clock.
        while (RCC_GetSysclkSrc() != 0x08)
        {
        }
    }
    else
    { // If HSE fails to open, the program will come here, where the user can add the error code to handle.
      // When HSE fails or breaks down,mcu will automatically set HSI as the system clock.HSI is an internal high speed
      // clock of 8MHz.
        while (1)
        {
        }
    }
}

int32_t check_app_image_data_crc(const image_upgrade_info_t * app_image_info, uint32_t img_offset)
{
    uint32_t data = img_offset;
    uint32_t len = app_image_info->file_size;
    uint16_t dcrc = CRC16(0, (uint8_t *)data, len);
//    log_debug("cal crc%x, app_image crc %x\r\n", dcrc, app_image_info->image_crc);
    return (dcrc == app_image_info->image_crc ? 1 : 0);
}

uint8_t check_backup_upgrade_info(image_upgrade_info_t *upgrade_info)
{
    uint16_t info_crc, info_crc_in;
    
    ota_read_backup_image_info(upgrade_info);
    if (upgrade_info->upgrage_tag == UPGRADE_IMAGE_FLAG)
    {
        info_crc_in = upgrade_info->header_crc;
        upgrade_info->header_crc = CRC16_INITIAL;
        info_crc = CRC16(0, (uint8_t *)&upgrade_info, sizeof(image_upgrade_info_t));
        if (info_crc_in == info_crc)
        {
            log_debug("upgrade tag ok\r\n");
            return 1;
        }
    }
    return 0;
}

void do_upgrade(image_upgrade_info_t *upgrade_info)
{
    uint32_t i;
    uint32_t file_size = upgrade_info->file_size;
    uint32_t offset = 0;
    uint32_t r_size = 0;
    uint8_t r_buf[FLASH_PAGE_SIZE];
    FLASH_STS sta = FLASH_COMPL;
    uint8_t err_time = 0;

    for (i = 0; i < file_size; i += r_size)
    {
        r_size = FLASH_PAGE_SIZE;
        if (file_size - offset < FLASH_PAGE_SIZE)
            r_size = file_size - offset;
        
        W25QXX_Read(r_buf, offset + SF_BACKUP_IMAGE_ADDR, r_size);
        
        sta = (FLASH_STS)N32WB452_FLASH_Write(offset + OF_FIRMWARE_ADDR, r_buf, r_size);
//        log_debug("file %x WritePage[%x] status %x\r\n",file_size, i, sta);
        if (sta == FLASH_COMPL)
        {
            //写入成功, 增加偏移量
            offset += r_size;
            err_time = 0;
        }
        else
        {
            err_time++;
            if (err_time >= 3)
            {
                break;
            }
        }
    }


    if (sta == FLASH_COMPL)
    {
        log_debug("do_upgrade OK %x\r\n",i);
    }
    else log_debug("do_upgrade Fail %x\r\n", i);
}


int main(void)
{
    // const char *response = "bt salve say hello";
    int32_t ret;
    uint8_t  retry_time = 0;
    image_upgrade_info_t upgrade_info;
    
    SystemInit();
    SetSysClock_HSE_PLL(RCC_PLL_MUL_18);
    /* System Clocks Configuration */
    RCC_Configuration();

    log_init();
    log_debug("\r\nsystem boot start...\r\n");

    W25QXX_Init();

    //检查升级标志
    ret = ota_check_upgrade_tag(&upgrade_info);
    if (ret == 1)
    {
        //需要升级, 对backup image crc16校验
        if (ota_image_file_check_crc(&upgrade_info, TYPE_IMAGE_BACKUP) == 1)
        {
            do
            {
                //校验成功, 升级程序
                do_upgrade(&upgrade_info);
                //检查升级后app的crc校验.
                if (check_app_image_data_crc(&upgrade_info, OF_FIRMWARE_ADDR) == 1)
                {
                    //升级成功清除升级标志.
                    ota_upgrade_tag_clean();
                    log_debug("check new fw crc OK...\r\n");
                    break;
                }
                else 
                {
                    retry_time++;
                    log_debug("******upgrade error, retry_time %d******\r\n", retry_time);
                    if (retry_time >= 3) 
                    {
                        log_debug("Pls reboot system!!!\r\n");
                        while(1);
                    }
                }
            }
            while (retry_time <= 3);
            
        }
        else 
            log_debug("check_crc error...\r\n");
        
    }
    
    //不需要升级, 跳转到APP
    log_debug("jump2app...\r\n");
    jump2app(OF_FIRMWARE_ADDR);

    while (1)
    {

    }
}
