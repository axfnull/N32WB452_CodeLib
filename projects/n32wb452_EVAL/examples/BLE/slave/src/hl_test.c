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
 * @file hl_test.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
 
#include "Eif_iom.h"
#include "log.h"
#include "main.h"
#include "interface.h"
#include "n32wb452_rtc.h"
#include "ble_monitor.h"

static uint8_t indicate_Flag = 0;

RTC_DateType RTC_DateStructure;
RTC_DateType RTC_DateDefault;
RTC_TimeType RTC_TimeStructure;
RTC_TimeType RTC_TimeDefault;
RTC_InitType RTC_InitStructure;
RTC_AlarmType RTC_AlarmStructure;
uint32_t SynchPrediv, AsynchPrediv;
    
    
void indication_init(void)
{
    GPIO_InitType GPIO_InitStruct;

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE); // PB0
    GPIO_InitStruct.Pin        = GPIO_PIN_0;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStruct);
    
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE); // PA7
    GPIO_InitStruct.Pin        = GPIO_PIN_7;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStruct);
    GPIO_SetBits(GPIOA, GPIO_PIN_7);
}

void indication_deinit(void)
{
    GPIO_InitType GPIO_InitStruct;

    GPIO_ResetBits(GPIOB,GPIO_PIN_0);
    GPIO_ResetBits(GPIOA,GPIO_PIN_7);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE); // PB0
    GPIO_InitStruct.Pin        = GPIO_PIN_0;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStruct);
    
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE); // PA7
    GPIO_InitStruct.Pin        = GPIO_PIN_7;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStruct);
    
}

void indicate_flag(void)
{
    if(indicate_Flag == 0)
    {
        GPIO_SetBits(GPIOB, GPIO_PIN_0);
        indicate_Flag = 1;
    }
    else
    {
        GPIO_ResetBits(GPIOB, GPIO_PIN_0);
        indicate_Flag = 0;
    }
}

void RTC_CLKSourceConfig(uint8_t ClkSrc, uint8_t FirstLastCfg, uint8_t RstBKP)
{
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR | RCC_APB1_PERIPH_BKP, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);
    PWR_BackupAccessEnable(ENABLE);
    /* Reset Backup */
    if (RstBKP == 1)
    {
        BKP_DeInit();
    }
    /* Disable RTC clock */
    RCC_EnableRtcClk(DISABLE);
    if (ClkSrc == 0x01)
    {
        //log_info("\r\n RTC_ClkSrc Is Set HSE128! \r\n");
        if (FirstLastCfg == 0)
        {
            /* Enable HSE */
            RCC_EnableLsi(DISABLE);
            RCC_ConfigHse(RCC_HSE_ENABLE);
            while (RCC_WaitHseStable() == ERROR)
            {
            }
            RCC_ConfigRtcClk(RCC_RTCCLK_SRC_HSE_DIV128);
        }
        else
        {
            RCC_EnableLsi(DISABLE);
            RCC_ConfigRtcClk(RCC_RTCCLK_SRC_HSE_DIV128);
            /* Enable HSE */
            RCC_ConfigHse(RCC_HSE_ENABLE);

            while (RCC_WaitHseStable() == ERROR)
            {
            }
        }
        SynchPrediv  = 0x1E8; // 8M/128 = 62.5KHz
        AsynchPrediv = 0x7F;  // value range: 0-7F  
    }
    else if (ClkSrc == 0x02)
    {
        if (FirstLastCfg == 0)
        {   
            RCC_EnableLsi(DISABLE); 
            RCC_ConfigLse(RCC_LSE_ENABLE);
            while (RCC_GetFlagStatus(RCC_FLAG_LSERD) == RESET)
            {
            }
            RCC_ConfigRtcClk(RCC_RTCCLK_SRC_LSE);
        }
        else
        {
            /* Enable the LSE OSC32_IN PC14 */
            RCC_EnableLsi(DISABLE);
            RCC_ConfigRtcClk(RCC_RTCCLK_SRC_LSE);
            RCC_ConfigLse(RCC_LSE_ENABLE);
            while (RCC_GetFlagStatus(RCC_FLAG_LSERD) == RESET)
            {
            }
        }
        SynchPrediv  = 0xFF; // 32.768KHz
        AsynchPrediv = 0x7F;    
    }
    else if (ClkSrc == 0x03)
    {       
        if (FirstLastCfg == 0)
        {
            /* Enable the LSI OSC */
            RCC_EnableLsi(ENABLE);      
            while (RCC_GetFlagStatus(RCC_FLAG_LSIRD) == RESET)
            {
            }
            RCC_ConfigRtcClk(RCC_RTCCLK_SRC_LSI);
        }
        else
        {
            RCC_ConfigRtcClk(RCC_RTCCLK_SRC_LSI);
            /* Enable the LSI OSC */
            RCC_EnableLsi(ENABLE);
            while (RCC_GetFlagStatus(RCC_FLAG_LSIRD) == RESET)
            {
            }
        }
        SynchPrediv  = 0x136; // 39.64928KHz
        AsynchPrediv = 0x7F;    
    }
    else
    {}  
    /* Enable the RTC Clock */
    RCC_EnableRtcClk(ENABLE);
    RTC_WaitForSynchro();
}
static void RTC_PrescalerConfig(void)
{
    RTC_InitStructure.RTC_AsynchPrediv = AsynchPrediv;
    RTC_InitStructure.RTC_SynchPrediv  = SynchPrediv;
    RTC_InitStructure.RTC_HourFormat   = RTC_24HOUR_FORMAT;

    /* Check on RTC init */
    if (RTC_Init(&RTC_InitStructure) == ERROR)
      {
        log_info("\r\n //******* RTC Prescaler Config failed **********// \r\n");
    }
    
}
void WakeUpClockSelect(uint8_t WKUPClkSrcSel)
{
    if (WKUPClkSrcSel == 0x01)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_RTCCLK_DIV16);
    else if (WKUPClkSrcSel == 0x02)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_RTCCLK_DIV8);
    else if (WKUPClkSrcSel == 0x03)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_RTCCLK_DIV4);
    else if (WKUPClkSrcSel == 0x04)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_RTCCLK_DIV2);
    else if (WKUPClkSrcSel == 0x05)
        RTC_ConfigWakeUpClock(RTC_WKUPCLK_CK_SPRE_16BITS);  
}
void EXTI20_RTCWKUP_Configuration(FunctionalState Cmd)
{
    EXTI_InitType EXTI_InitStructure;
    NVIC_InitType NVIC_InitStructure;
    EXTI_ClrITPendBit(EXTI_LINE20);
    EXTI_InitStructure.EXTI_Line = EXTI_LINE20;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel                   = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = Cmd;
    NVIC_Init(&NVIC_InitStructure);
}   


void hlt_rtc_init(void)
{
    RTC_CLKSourceConfig(3, 0, 1);   /*1:HSE/128  2:LSE  3:LSI*/
    RTC_PrescalerConfig();

    WakeUpClockSelect(4);
    EXTI20_RTCWKUP_Configuration(ENABLE);
    /* Enable the RTC Wakeup Interrupt */
    RTC_ConfigInt(RTC_INT_WUT, ENABLE);
                
    
    RTC_EnableWakeUp(DISABLE);
    RTC_SetWakeUpCounter(16328);//RTC_SetWakeUpCounter(16328);//65312  1s
    RTC_EnableWakeUp(ENABLE);
}
