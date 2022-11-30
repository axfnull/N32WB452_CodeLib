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
#include "att.h"

#define LED1_APBxClkCmd RCC_EnableAPB2PeriphClk
#define LED1_PORT_CLK   RCC_APB2_PERIPH_GPIOD
#define LED1_PORT       GPIOD
#define LED1_PIN        GPIO_PIN_0
#define LED1_ON()       GPIO_SetBits(LED1_PORT, LED1_PIN); // LED1 ON
#define LED1_OFF()      GPIO_ResetBits(LED1_PORT, LED1_PIN); // LED1 OFF
#define LED1_BLINK()    (((LED1_PORT)->POD)^=LED1_PIN)

#define LED2_APBxClkCmd RCC_EnableAPB2PeriphClk
#define LED2_PORT_CLK   RCC_APB2_PERIPH_GPIOD
#define LED2_PORT       GPIOD
#define LED2_PIN        GPIO_PIN_1
#define LED2_ON()       GPIO_SetBits(LED2_PORT, LED2_PIN); // LED2 ON
#define LED2_OFF()      GPIO_ResetBits(LED2_PORT, LED2_PIN); // LED2 OFF
#define LED2_BLINK()    (((LED2_PORT)->POD)^=LED2_PIN)

#define LED3_APBxClkCmd RCC_EnableAPB2PeriphClk
#define LED3_PORT_CLK   RCC_APB2_PERIPH_GPIOE
#define LED3_PORT       GPIOE
#define LED3_PIN        GPIO_PIN_2
#define LED3_ON()       GPIO_SetBits(LED3_PORT, LED3_PIN); // LED1 ON
#define LED3_OFF()      GPIO_ResetBits(LED3_PORT, LED3_PIN); // LED1 OFF
#define LED3_BLINK()    (((LED3_PORT)->POD)^=LED3_PIN)

#define LED4_APBxClkCmd RCC_EnableAPB2PeriphClk
#define LED4_PORT_CLK   RCC_APB2_PERIPH_GPIOE
#define LED4_PORT       GPIOE
#define LED4_PIN        GPIO_PIN_3
#define LED4_ON()       GPIO_SetBits(LED4_PORT, LED4_PIN); // LED2 ON
#define LED4_OFF()      GPIO_ResetBits(LED4_PORT, LED4_PIN); // LED2 OFF
#define LED4_BLINK()    (((LED4_PORT)->POD)^=LED4_PIN)

#define KEY1_APBxClkCmd     RCC_EnableAPB2PeriphClk
#define KEY1_PORT_CLK       RCC_APB2_PERIPH_GPIOD
#define KEY1_PORT           GPIOD
#define KEY1_PIN            GPIO_PIN_8
#define KEY1_GET_VALUE      GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN)

#define KEY1_EXTI_IRQn      EXTI9_5_IRQn
#define KEY1_EXTI_PORTSRC   GPIOD_PORT_SOURCE
#define KEY1_EXTI_PINSRC    GPIO_PIN_SOURCE8
#define KEY1_EXTI_LINE      EXTI_LINE8

#define KEY2_APBxClkCmd     RCC_EnableAPB2PeriphClk
#define KEY2_PORT_CLK       RCC_APB2_PERIPH_GPIOD
#define KEY2_PORT           GPIOD
#define KEY2_PIN            GPIO_PIN_9
#define KEY2_GET_VALUE      GPIO_ReadInputDataBit(KEY2_PORT, KEY2_PIN)

#define KEY3_APBxClkCmd     RCC_EnableAPB2PeriphClk
#define KEY3_PORT_CLK       RCC_APB2_PERIPH_GPIOA
#define KEY3_PORT           GPIOA
#define KEY3_PIN            GPIO_PIN_0
#define KEY3_GET_VALUE      GPIO_ReadInputDataBit(KEY3_PORT, KEY3_PIN)

#define KEY3_EXTI_IRQn      EXTI0_IRQn
#define KEY3_EXTI_PORTSRC   GPIOA_PORT_SOURCE
#define KEY3_EXTI_PINSRC    GPIO_PIN_SOURCE0
#define KEY3_EXTI_LINE      EXTI_LINE0

#define BUZZER_APBxClkCmd   RCC_EnableAPB2PeriphClk
#define BUZZER_PORT_CLK     RCC_APB2_PERIPH_GPIOB
#define BUZZER_PORT         GPIOB
#define BUZZER_PIN          GPIO_PIN_15
#define BUZZER_ON()         GPIO_SetBits(BUZZER_PORT, BUZZER_PIN);
#define BUZZER_OFF()        GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);



#define SERVICE_UUID                (ATT_SVC_UKEY_SERVICE)
#define USER_WRITE_NOTIFY_CHAR      (ATT_CHAR_WRITE_NOTIFY)

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#define KEY_NUM     2
#define KEY_SEND    (0x02)
#define KEY_STOP    (0x01)

#define KEY_FILTER  2
typedef struct {
    uint8_t key;
    uint8_t temp;
    uint8_t cnt[KEY_NUM];
    uint8_t hold[KEY_NUM];
}KEY_STATUS;

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

typedef enum
{
    WKP_FROM_NONE = 0,
    WKP_FROM_BT = 0x01,
    WKP_FROM_KEY = 0x02,
    WKP_FROM_MAX
}WAKEUP_SRC;

void delay_us(uint32_t t);
void delay_ms(uint32_t t);

void mainboard_exit_stop2(void);
int mainboard_module_init(void);
void SetSysClock_HSI(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */
