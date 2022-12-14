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
 * @file application.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include <rtthread.h>
#include "drv_gpio.h"
#include "pin.h"
#include "drv_usart.h"
#include "serial.h"

#ifdef RT_USING_DFS
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#endif

#ifdef RT_USING_RTGUI
#include <rtgui/rtgui.h>
#include <rtgui/rtgui_server.h>
#include <rtgui/rtgui_system.h>
#include <rtgui/driver.h>
#include <rtgui/calibration.h>
#endif

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t led0_stack[ 512 ], led1_stack[ 512 ];
static rt_uint8_t key_stack[512];

static struct rt_thread led0_thread;
static struct rt_thread led1_thread;
static struct rt_thread key_thread;

#define LED0_PIN        GET_PIN(D,  0)
#define LED1_PIN        GET_PIN(D,  1)

#define KEY0_PIN        GET_PIN(D,  8)
#define KEY1_PIN        GET_PIN(D,  9)
#define KEY2_PIN        GET_PIN(A,  0)


#define KEY0  rt_pin_read(KEY0_PIN)
#define KEY1  rt_pin_read(KEY1_PIN)
 
#define KEY0_PRES   1
#define KEY1_PRES   2

/**
 * @brief  scan key
 */
uint8_t KEY_Scan(uint8_t mode)
{
    static uint8_t key_up=1;
    if(mode)key_up=1;  
    if(key_up&&(KEY0==0||KEY1==0))
    {
        rt_thread_delay(1);
        key_up=0;
        if(KEY0==0)return KEY0_PRES;
        else if(KEY1==0)return KEY1_PRES;
    }else if(KEY0==1&&KEY1==1)key_up=1;
    return 0;
}

/**
 * @brief  led0 thread entry
 */
static void led0_thread_entry(void* parameter)
{
    while(1)
    {
        rt_thread_delay(50);    //delay 500ms
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_delay(50);    //delay 500ms
        rt_pin_write(LED0_PIN, PIN_LOW);
    }
}

/**
 * @brief  led1 thread entry
 */
static void led1_thread_entry(void* parameter)
{
    while(1)
    {
        rt_thread_delay(25);    //delay 250ms
        rt_pin_write(LED1_PIN, PIN_HIGH);
        rt_thread_delay(25);    //delay 250ms
        rt_pin_write(LED1_PIN, PIN_LOW);
    }
}

/**
 * @brief  key2 inttrupt callback
 */
void key2_irq(void *args)
{
    rt_kprintf("enter key2 interrupt callback.\n");
}

/**
 * @brief  key thread entry
 */
void key_thread_entry(void* parameter)
{
    rt_uint8_t key;

    rt_pin_mode(KEY0_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(KEY1_PIN, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(KEY2_PIN, PIN_MODE_INPUT);
    
    rt_pin_attach_irq(KEY2_PIN, PIN_IRQ_MODE_RISING, key2_irq, RT_NULL);
    rt_pin_irq_enable(KEY2_PIN, ENABLE);
    
    while(1)
    {
        key = KEY_Scan(0);
        switch(key)
        {
            case KEY0_PRES:
                rt_kprintf("key0 pressed.\n");
                break;
            case KEY1_PRES:
                rt_kprintf("key1 pressed.\n");
                break;

            default:
                break;
            
        }
        rt_thread_delay(5);
    }
}

#ifdef RT_USING_RTGUI
rt_bool_t cali_setup(void)
{
    rt_kprintf("cali setup entered\n");
    return RT_FALSE;
}

void cali_store(struct calibration_data *data)
{
    rt_kprintf("cali finished (%d, %d), (%d, %d)\n",
               data->min_x,
               data->max_x,
               data->min_y,
               data->max_y);
}
#endif /* RT_USING_RTGUI */

/**
 * @brief  Main program
 */
int main(void)
{
    rt_err_t result;

    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);

    /* init led0 thread */
    result = rt_thread_init(&led0_thread, "led0", led0_thread_entry, RT_NULL, (rt_uint8_t*)&led0_stack[0], sizeof(led0_stack), 4, 5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&led0_thread);
    }
    /* init led1 thread */
    result = rt_thread_init(&led1_thread, "led1", led1_thread_entry, RT_NULL, (rt_uint8_t*)&led1_stack[0], sizeof(led1_stack), 5, 5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&led1_thread);
    }
    /* init key thread */
    result = rt_thread_init(&key_thread, "key", key_thread_entry, RT_NULL, (rt_uint8_t*)&key_stack[0], sizeof(key_stack), 3, 5);
    {
        rt_thread_startup(&key_thread);
    }
    while(1)
    {
        rt_thread_delay(50);
    }
}


/*@}*/
