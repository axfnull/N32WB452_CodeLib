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
#include "rwip_config.h"
#include "n32wb452_ble_api.h"
#include "interface.h"
#include "eif_iom.h"
#include "app.h"
#include "log.h"
#include "n32wb452_ble_protocol.h"
#include "n32wb452_ble_ota_api.h"
#include "n32wb452_w25qxx.h"
#include "n32wb452_ota_iap.h"

void led4_blink(void);
void RCC_Configuration(void);
int mainboard_module_init(void);

const char ble_name[] = "WB452_OTA";
const char ble_addr[] = "11:22:33:FF:30:AA";

const char product_sn[] = PRODUCE_SN;

bt_attr_param bt_init = {0};
uint8_t bt_rx_pkt_buf[BLE_PKT_MAX_LEN];     //BLE通信帧数据长度, 依据BLE MTU设置:20~250, 数据通信层使用
ble_packet_tag ble_rx_data;     //BLE接收数据区, 数据应用层使用

uint16_t ble_tx_len = 0;
uint8_t ble_tx_buff[BLE_TX_BUF_LEN]; //BLE发送数据区, 数据应用层使用


uint16_t bt_data_len     = 0;
uint16_t bt_connect_flag = 0;

uint8_t flag_led1             = 0; // The flag of led1 blink.
uint8_t flag_led2             = 0; // The flag of led2 blink.
uint8_t system_10s_flag        = 0;
uint32_t system_10s_cnt        = 0;
uint8_t flag_bt_enter_sleep  = 0;

BT_SERVER_STS gBT_STS = BT_IDLE;

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
 * @brief  Inserts a delay time.
 * @param count specifies the delay time length.
 */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_LINE14) != RESET)
    {
        //MCU在STOP2模式时, 被蓝牙唤醒,需要重新配置所用到的外设
        if (flag_bt_enter_sleep == 1)
        {
            flag_bt_enter_sleep = 0;
            mainboard_exit_stop2();
        }
        //log_debug("exti 14 irq %d.\r\n", flag_bt_enter_sleep);
        bt_handler();
        /* Clears the SEL Button EXTI line pending bits. */
        EXTI_ClrITPendBit(EXTI_LINE14);
    }
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

/**
 * @brief 设置1ms的计数定时器6中断函数
 * @(用于本方案软件系统的通用计数)
 * @param void
 * @return void
 */
void TIM6_IRQHandler(void)
{
    if (TIM_GetIntStatus(TIM6, TIM_FLAG_UPDATE) != RESET)
    {
        TIM6->STS &= ~(0x01 << 0); //清除中断标志位

        system_10s_cnt++;
        if (system_10s_cnt >= (2000 * 5)) // 5ms*2000 = 10s
        {
            system_10s_cnt  = 0;
            system_10s_flag = 1;
        }
    }
}

/**
 * @brief 设置1ms的计数定时器6初始化
 * @param void
 * @return void
 */
void TIM6_init(void)
{
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    NVIC_InitType NVIC_InitStructure;

    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM6, ENABLE); //时钟使能

    //定时器TIM3初始化
    TIM_TimeBaseStructure.Period    = 19;                //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.Prescaler = 3599;              //设置用来作为36MHZ的分频系数；TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.ClkDiv    = TIM_CLK_DIV1;      //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_DOWN; // TIM向上计数模式
    TIM_InitTimeBase(TIM6, &TIM_TimeBaseStructure);      //根据指定的参数初始化TIMx的时间基数单位

    /*定时器中断参数设置*/
    TIM6->STS &= 0xFFFE;                         //清除update中断标志位，否则会出现刚配置完中断就进入中断服务函数的问题
    TIM_ConfigInt(TIM6, TIM_INT_UPDATE, ENABLE); //使能指定的TIM6中断,允许更新中断

    //中断优先级NVIC设置
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM6_IRQn; // TIM6中断
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;    // IRQ通道被使能
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;         //抢占优先级2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;         //子优先级0
    NVIC_Init(&NVIC_InitStructure);                                   //初始化NVIC寄存器

    TIM_Enable(TIM6, ENABLE);
}

void TIM6_deinit(void)
{
    TIM_Enable(TIM6, DISABLE);
    TIM_DeInit(TIM6);
}

//注意!
//注意!
//注意!
//回调函数中，不可使用阻塞式应用，执行时间应尽可能短。否则会影响BLE传输效率,甚至传输出错;
void bt_event_callback_func(bt_event_enum event, uint8_t* data, uint32_t size, uint32_t character_uuid)
{
    uint16_t length;
//    static uint16_t rvc_total = 0;

    switch (event)
    {
        case BT_EVENT_VERSION:
            log_debug("ble version:\r\n");
            break;

        case BT_EVENT_CONNECTED:
            system_10s_flag = 0;
            system_10s_cnt = 0;
            TIM_Enable(TIM6, DISABLE);  //有蓝牙连接时,不允许进入低功耗计时
            log_debug("ble connect:\r\n");
            bt_connect_flag = 1;
            break;

        case BT_EVENT_DISCONNECTD:
            system_10s_flag = 0;
            system_10s_cnt = 0;
            ble_rx_data.RxCurrentLen = 0;
            ble_rx_data.RxTotalLen = 0;
            ble_rx_data.RxFinishFlag = 0;
            TIM_Enable(TIM6, ENABLE);   //有蓝牙断开连接时,重新开始进入低功耗计时
            log_debug("ble disconnect:\r\n");
            bt_connect_flag  = 0;
            LED4_OFF();
            break;

        case BT_EVENT_RCV_DATA:
            length = bt_rcv_data(bt_rx_pkt_buf, size, character_uuid);
            if (length == 0) 
            {
                log_debug("rcv data err,\r\n");
                break;
            }
            ble_packet_parse(bt_rx_pkt_buf, length, (uint8_t *)&ble_rx_data);
            led4_blink();
            break;

        default:
            break;
    }
}

void check_ble_tx_buff_is_empty(void)
{
    while(ble_tx_len)
    {
        ble_send_packet(&ble_tx_len, ble_tx_buff);
    }
    delay_ms(100);
    bt_disconnect();
}

/******************************************************************/
/***
 * @brief    Configure the ports of led.
 */
void led_gpio_init(void)
{
    GPIO_InitType GPIO_InitStruct;

    LED3_APBxClkCmd(LED3_PORT_CLK, ENABLE); // PB15 for LED1
    GPIO_InitStruct.Pin        = LED3_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(LED3_PORT, &GPIO_InitStruct);

    LED4_APBxClkCmd(LED4_PORT_CLK, ENABLE); // PD10 for LED2
    GPIO_InitStruct.Pin        = LED4_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(LED4_PORT, &GPIO_InitStruct);
}

void led_gpio_deinit(void)
{
    GPIO_InitType GPIO_InitStruct;

    LED3_APBxClkCmd(LED3_PORT_CLK, ENABLE); // PB15 for LED1
    GPIO_InitStruct.Pin        = LED3_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(LED3_PORT, &GPIO_InitStruct);

    LED4_APBxClkCmd(LED4_PORT_CLK, ENABLE); // PD10 for LED2
    GPIO_InitStruct.Pin        = LED4_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(LED4_PORT, &GPIO_InitStruct);
    
    LED3_APBxClkCmd(LED3_PORT_CLK, DISABLE);
    LED4_APBxClkCmd(LED4_PORT_CLK, DISABLE);
}


/******************************************************************/
/***
 * @brief    Toggle the led to blink.
 */
void led3_blink(void)
{
    if (flag_led1 == 0)
    {
        LED3_ON(); // LED1 ON
        flag_led1 = 1;
    }
    else
    {
        LED3_OFF(); // LED1 OFF
        flag_led1 = 0;
    }
}

/***
 * @brief    Config the led to ON or OFF.
 */
void led4_blink(void)
{
    if (flag_led2)
    {
        flag_led2 = 0;
        LED4_ON();
    }
    else
    {
        flag_led2 = 1;
        LED4_OFF();
    }
}


void mainboard_enter_stop2(void)
{
    led_gpio_deinit();
    eif_gpio_DeInit();      //蓝牙休眠时需要调用此接口,以降低功耗
    flag_bt_enter_sleep = 1;
    
    log_debug("system enter stop2 mode...\r\n");
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);
    PWR_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
}

//从STOP2模式退出来后,需要重新配置所有用到的MCU资源
void mainboard_exit_stop2(void)
{
    SystemInit();
    SetSysClock_HSE_PLL(RCC_PLL_MUL_18);
    RCC_Configuration();
    log_init();
    log_debug("system restart...\r\n");
    led_gpio_init();
    LED3_ON();
    ble_hardware_reinit();
    TIM6_init();
    flag_bt_enter_sleep = 0;
    system_10s_flag = 0;
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


int main(void)
{
    // const char *response = "bt salve say hello";
    int32_t ret;
    set_vector_table();
    SetSysClock_HSE_PLL(RCC_PLL_MUL_18);
    /* System Clocks Configuration */
    RCC_Configuration();

    log_init();
    log_debug("system start...\r\n");
    led_gpio_init();
    LED3_ON();
    LED4_OFF();

    memcpy(bt_init.device_name, ble_name, MIN(strlen(ble_name), sizeof(bt_init.device_name)));
    memcpy(bt_init.device_addr, ble_addr, MIN(strlen(ble_addr), sizeof(bt_init.device_addr)));
    bt_init.scan_rsp_data[0] = 0x13;
    bt_init.scan_rsp_data[1] = 0xff;
    bt_init.scan_rsp_data[2] = 0x56;
    bt_init.scan_rsp_data[3] = 0x00;
    memcpy(&bt_init.scan_rsp_data[4], product_sn, MIN(strlen(product_sn),  28));
    bt_init.scan_rsp_data_len = 4+strlen(product_sn);
    
    bt_init.service[0].svc_uuid                = SERVICE_UUID;
    bt_init.service[0].character[0].uuid       = USER_IDX_WRITE_NOTIFY_CHAR;
    bt_init.service[0].character[0].permission = BT_WRITE_PERM | BT_NTF_PERM;
    bt_init.service[0].character[1].uuid       = USER_IDX_READ_NOTIFY_CHAR;
    bt_init.service[0].character[1].permission = BT_NTF_PERM;

    ret = bt_ware_init(&bt_init, (bt_event_callback_handler_t)bt_event_callback_func);
    if (ret != BT_RET_SUCCESS)
    {
        log_debug("bt init failed.\r\n");
    }
    else
    {
        log_debug("bt init ok.\r\n");
    }
    uint8_t ver[10];
    BT_get_version(ver);
    printf("ble driver version %s\r\n", ver);

    TIM6_init();
    ble_ota_init();

    while (1)
    {
        bt_run_thread();

        //数据包接收完成, 处理数据包的数据
        if (ble_rx_data.RxFinishFlag)
        {
            ble_rx_data.RxFinishFlag = 0;
            ret = ble_communication_parse(ble_rx_data.RxBuf, ble_rx_data.RxTotalLen, ble_tx_buff, &ble_tx_len);
            ble_rx_data.RxTotalLen = 0;
            if (ret == 0)
            {
                //ble_ota_set_status_get_file();
            }
        }

        //接收到蓝牙数据, 并处理完成后, 如果有数据包需要上传, 则发送数据
        if (ble_tx_len > 0)
        {
            //在蓝牙连接状态发送ble数据
            if (bt_connect_flag == 1)
            {
                ble_send_packet(&ble_tx_len, ble_tx_buff);
            }
            else
            {
                ble_tx_len = 0;
                memset(ble_tx_buff, 0, BLE_TX_BUF_LEN);
            }
        }

        //执行OTA主程序
        ble_ota_process();

        
        //10s空闲, 进入STOP2模式, 
        if (system_10s_flag) 
        {
            system_10s_flag = 0;
            //mainboard_enter_stop2();
            //mainboard_exit_stop2();
        }
    }
}
