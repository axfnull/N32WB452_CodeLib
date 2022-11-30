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
 * @file bsp_gpio.c
 * @author Nations
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "bsp_gpio.h"

GPIO_Module *LedPort[KEY_NUM] = {GPIOD,GPIOD,GPIOE,GPIOE};
uint32_t LedPin[KEY_NUM]  = {GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2,GPIO_PIN_3};

/***
 * @brief    Configure the ports of TSC channels for sample.
 * @param:  None
 * @retval: None
 */
void tsc_gpio_init(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed   = GPIO_INPUT;

    RCC_EnableAPB2PeriphClk( RCC_APB2_PERIPH_GPIOC
                            |RCC_APB2_PERIPH_GPIOD, ENABLE);

    /*  tsc GPIOC port*/
    GPIO_InitStructure.Pin  = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);

    /* tsc GPIOC port */
    GPIO_InitStructure.Pin  = GPIO_PIN_2;
    GPIO_InitPeripheral(GPIOD, &GPIO_InitStructure);
}

/***
 * @brief    Configure the ports of led.
 */
void led_gpio_init(void)
{
    uint32_t i;
    GPIO_InitType GPIO_InitStructure;

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOD, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOE, ENABLE);
    
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_2MHz;
    
    for(i=0;i<KEY_NUM;i++)
    {
        GPIO_ResetBits(LedPort[i],LedPin[i]);
    
        GPIO_InitStructure.Pin          = LedPin[i];
        GPIO_InitPeripheral(LedPort[i],&GPIO_InitStructure);
    }
}


/***
 * @brief    disable the port of LED.
  * @param:  None
  * @retval: None
  */
void led_gpio_DeInit(void)
{
    uint32_t i;
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed   = GPIO_INPUT;
    
    for(i=0;i<KEY_NUM;i++)
    {
        GPIO_ResetBits(LedPort[i],LedPin[i]);
    
        GPIO_InitStructure.Pin          = LedPin[i];
        GPIO_InitPeripheral(LedPort[i],&GPIO_InitStructure);
    } 
}



