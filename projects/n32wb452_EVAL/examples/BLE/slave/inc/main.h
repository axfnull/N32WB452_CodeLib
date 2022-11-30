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
 * @file main.h
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32wb452.h"

#define N32WB452XL_EVB

#ifdef  N32WB452XL_EVB
#define LED3_APBxClkCmd RCC_EnableAPB2PeriphClk
#define LED3_PORT_CLK   RCC_APB2_PERIPH_GPIOD
#define LED3_PORT       GPIOD
#define LED3_PIN        GPIO_PIN_9
#define LED3_ON()       GPIO_SetBits(LED3_PORT, LED3_PIN)
#define LED3_OFF()      GPIO_ResetBits(LED3_PORT, LED3_PIN)
#define LED3_BLINK()    (((LED3_PORT)->POD)^=LED3_PIN)

#define LED4_APBxClkCmd RCC_EnableAPB2PeriphClk
#define LED4_PORT_CLK   RCC_APB2_PERIPH_GPIOD
#define LED4_PORT       GPIOD
#define LED4_PIN        GPIO_PIN_8
#define LED4_ON()       GPIO_SetBits(LED4_PORT, LED4_PIN)
#define LED4_OFF()      GPIO_ResetBits(LED4_PORT, LED4_PIN)
#define LED4_BLINK()    (((LED4_PORT)->POD)^=LED4_PIN)

#else
#define LED3_APBxClkCmd RCC_EnableAPB2PeriphClk
#define LED3_PORT_CLK   RCC_APB2_PERIPH_GPIOA
#define LED3_PORT       GPIOA
#define LED3_PIN        GPIO_PIN_7
#define LED3_ON()       GPIO_SetBits(LED3_PORT, LED3_PIN)
#define LED3_OFF()      GPIO_ResetBits(LED3_PORT, LED3_PIN)
#define LED3_BLINK()    (((LED3_PORT)->POD)^=LED3_PIN)

#define LED4_APBxClkCmd RCC_EnableAPB2PeriphClk
#define LED4_PORT_CLK   RCC_APB2_PERIPH_GPIOB
#define LED4_PORT       GPIOB
#define LED4_PIN        GPIO_PIN_0
#define LED4_ON()       GPIO_SetBits(LED4_PORT, LED4_PIN)
#define LED4_OFF()      GPIO_ResetBits(LED4_PORT, LED4_PIN)
#define LED4_BLINK()    (((LED4_PORT)->POD)^=LED4_PIN)

#endif

#define SERVICE_UUID                (ATT_SVC_UKEY_SERVICE)
#define USER_WRITE_NOTIFY_CHAR      (ATT_CHAR_WRITE_NOTIFY)

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

typedef enum
{
    BT_IDLE = 0,
    BT_INITIALIZED,
    BT_ADVERTISING,
    BT_CONNECTED,
    BT_DISCONNECTED,
    BT_STS_TOTAL
}BT_SERVER_STS;
extern BT_SERVER_STS gBT_STS;

void delay_us(uint32_t t);
void delay_ms(uint32_t t);
uint8_t mainboard_enter_stop2(void);
void mainboard_exit_stop2(void);
void SetSysClock_HSE_PLL(uint32_t pllmul);
void SetSysClock_HSI(void);

extern void hlt_rtc_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */
