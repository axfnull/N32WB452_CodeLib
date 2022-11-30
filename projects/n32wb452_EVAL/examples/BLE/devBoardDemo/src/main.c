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
 * @version v1.0.2
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "n32wb452_ble_api.h"
#include "lcd_display.h"
#include "qma_i2c_drv.h"
#include "gsensor.h"
#include "bsp_adc.h"
#include "hdc_i2c_drv.h"
#include "hdc2010.h"
#include "n32xx_tsc_alg_api.h"
#include "interface.h"
#include "eif_iom.h"
#include "ke_event.h"
#include "app.h"
#include "app_task.h"
#include "user.h"
#include "log.h"
#include "ble_monitor.h"

#if     defined(__IAR_ARM)
const char ble_addr[] = "22:22:33:db:30:A0";
const char ble_name[] = "WB452IH22";
#elif   defined(WCHAR16)
const char ble_addr[] = "44:22:33:db:30:A0";
const char ble_name[] = "WB452DH44";
#else
const char ble_addr[] = "66:22:33:db:30:A0";
const char ble_name[] = "WB452DW66";
#endif

bt_attr_param bt_init           = {0};
uint8_t bt_packet_buf[256];
uint8_t bt_data_buf[1024];
uint16_t bt_data_rxlen          = 0;
uint8_t bt_total_rxlen          = 0;
uint16_t bt_data_finish_flag    = 0;
uint16_t bt_connect_flag        = 0;

uint8_t system_1s_flag          = 1;
uint32_t system_1s_cnt          = 0;
uint8_t flag_bt_enter_sleep     = 0;
uint32_t display_update_flag    = 0;

__IO uint8_t flag_bt_irq        = 0;
BT_SERVER_STS gBT_STS           = BT_IDLE;
G_SENSOR_DATA g_sensor_data     = {0, 0, 0};

KEY_STATUS gKey;

void delay_us(uint32_t t)//system clock=144MHz
{
    while (t--)
        for (uint32_t m = 0; m < 35; m++)
            ;
}

void delay_ms(uint32_t t)
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
        if (flag_bt_enter_sleep == 1)
        {
            flag_bt_irq = 1;
        }
        else
            bt_handler();
    }
    
    EXTI_ClrITPendBit(EXTI_LINE14);
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

void KeyCheck(void)
{
    uint8_t temp = 0;
    uint32_t i;
    
    if(KEY1_GET_VALUE == Bit_SET)
        temp |= KEY_STOP;
    
    if(KEY2_GET_VALUE == Bit_SET)
        temp |= KEY_SEND;
    
    temp ^= gKey.temp;
    for(i=0;i<KEY_NUM;i++)
    {
        if(0 == (temp & (0x01<<i)))
            continue;
        
        if(gKey.cnt[i] < KEY_FILTER)
        {
            gKey.cnt[i]++;
            continue;
        }
        
        gKey.cnt[i] = 0;
        
        if(gKey.temp & (0x1<<i))
            gKey.key |= (0x1<<i);
        
        gKey.temp ^= (0x01<<i);
    }
}

/**
 * @brief 设置10ms的计数定时器6中断函数
 * @(用于本方案软件系统的通用计数)
 * @param void
 * @return void
 */
void TIM6_IRQHandler(void)
{
    if (TIM_GetIntStatus(TIM6, TIM_FLAG_UPDATE) != RESET)
    {
        TIM_ClrIntPendingBit(TIM6,TIM_FLAG_UPDATE);  //清除中断标志位

        KeyCheck();
        
        system_1s_cnt++;
        if (system_1s_cnt >= 100) // 10ms*100 = 1s
        {
            system_1s_cnt  = 0;
            system_1s_flag = 1;
            LED4_BLINK();
        }
    }
}

/**
 * @brief 设置10ms的计数定时器6初始化
 * @param void
 * @return void
 */
void TIM6_init(void)
{
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    NVIC_InitType NVIC_InitStructure;

    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM6, ENABLE); //时钟使能

    //定时器TIM6初始化
    TIM_TimeBaseStructure.Period    = 199;                //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
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

    system_1s_cnt   = 0;
    
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
            log_debug("ble connect:\r\n");
            LED3_ON();
            bt_connect_flag = 1;
            display_update_flag |= display_update(DISP_BT_CONNECT, bt_data_buf, sizeof(bt_data_rxlen));
            break;

        case BT_EVENT_DISCONNECTD:
            log_debug("ble disconnect:\r\n");
            LED3_OFF();
            bt_connect_flag  = 0;
            bt_data_rxlen   = 0;
            bt_total_rxlen = 0;
            bt_data_finish_flag = 0;
            display_update_flag |= display_update(DISP_BT_DISCONNECT, bt_data_buf, sizeof(bt_data_rxlen));
            break;

        case BT_EVENT_RCV_DATA:
            LED2_BLINK();
            length = bt_rcv_data(bt_packet_buf, size, character_uuid);
            
            if (length)
            {
                if(bt_total_rxlen == 0)
                {
                    bt_data_rxlen = 0;    
                    bt_total_rxlen = bt_packet_buf[0] + (bt_packet_buf[1]<<8) + 2;
                }
                memcpy(bt_data_buf+bt_data_rxlen, bt_packet_buf, length);
                bt_data_rxlen += length;
                if(bt_data_rxlen >= bt_total_rxlen)
                {
                    bt_data_finish_flag = 1;
                }
                
            }
            else
            {
                log_debug("rcv data err,\r\n");
            }
            
            break;

        default:
            break;
    }
}

/******************************************************************/
/***
 * @brief    Configure the ports of led.
 */
void led_gpio_init(void)
{
    GPIO_InitType GPIO_InitStruct;

    LED1_APBxClkCmd(LED1_PORT_CLK, ENABLE); // PD0 for LED1
    GPIO_InitStruct.Pin        = LED1_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(LED1_PORT, &GPIO_InitStruct);

    LED2_APBxClkCmd(LED2_PORT_CLK, ENABLE); // PD1 for LED2
    GPIO_InitStruct.Pin        = LED2_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(LED2_PORT, &GPIO_InitStruct);

    LED3_APBxClkCmd(LED3_PORT_CLK, ENABLE); // PE2 for LED3
    GPIO_InitStruct.Pin        = LED3_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(LED3_PORT, &GPIO_InitStruct);

    LED4_APBxClkCmd(LED4_PORT_CLK, ENABLE); // PE3 for LED4
    GPIO_InitStruct.Pin        = LED4_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(LED4_PORT, &GPIO_InitStruct);

}

void led_gpio_deinit(void)
{
    GPIO_InitType GPIO_InitStruct;

    LED1_OFF();
    LED2_OFF();
    LED3_OFF();
    LED4_OFF();
    
    LED1_APBxClkCmd(LED1_PORT_CLK, ENABLE); // PD0 for LED1
    GPIO_InitStruct.Pin        = LED1_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(LED1_PORT, &GPIO_InitStruct);

    LED2_APBxClkCmd(LED2_PORT_CLK, ENABLE); // PD1 for LED2
    GPIO_InitStruct.Pin        = LED2_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(LED2_PORT, &GPIO_InitStruct);

    LED1_APBxClkCmd(LED3_PORT_CLK, ENABLE); // PE2 for LED3
    GPIO_InitStruct.Pin        = LED3_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(LED3_PORT, &GPIO_InitStruct);

    LED2_APBxClkCmd(LED4_PORT_CLK, ENABLE); // PE3 for LED4
    GPIO_InitStruct.Pin        = LED4_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(LED4_PORT, &GPIO_InitStruct);
    
    LED1_APBxClkCmd(LED1_PORT_CLK, DISABLE);
    LED2_APBxClkCmd(LED2_PORT_CLK, DISABLE);
    LED3_APBxClkCmd(LED3_PORT_CLK, DISABLE);
    LED4_APBxClkCmd(LED4_PORT_CLK, DISABLE);
}

void log_gpio_deinit(void)
{
    GPIO_InitType GPIO_InitStruct;

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE); // PA9
    GPIO_InitStruct.Pin        = GPIO_PIN_9;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStruct);
    
    USART_Enable(LOG_USARTx, DISABLE);
    USART_DeInit(LOG_USARTx);
}

void key_gpio_init(void)
{
    GPIO_InitType GPIO_InitStruct;

    KEY1_APBxClkCmd(KEY1_PORT_CLK, ENABLE);
    GPIO_InitStruct.Pin       = KEY1_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitPeripheral(KEY1_PORT, &GPIO_InitStruct);

    KEY1_APBxClkCmd(KEY2_PORT_CLK, ENABLE);
    GPIO_InitStruct.Pin       = KEY2_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitPeripheral(KEY2_PORT, &GPIO_InitStruct);
}

void key_gpio_deinit(void)
{
    GPIO_InitType GPIO_InitStruct;

    KEY1_APBxClkCmd(KEY1_PORT_CLK, ENABLE);
    GPIO_InitStruct.Pin       = KEY1_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitPeripheral(KEY1_PORT, &GPIO_InitStruct);
    KEY1_APBxClkCmd(KEY1_PORT_CLK, DISABLE);

    KEY2_APBxClkCmd(KEY2_PORT_CLK, ENABLE);
    GPIO_InitStruct.Pin       = KEY2_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitPeripheral(KEY2_PORT, &GPIO_InitStruct);
    KEY2_APBxClkCmd(KEY2_PORT_CLK, DISABLE);
}

void key_wakeup_setup(void)
{
    GPIO_InitType GPIO_InitStruct;

    KEY1_APBxClkCmd(KEY3_PORT_CLK, ENABLE);
    GPIO_InitStruct.Pin       = KEY3_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitPeripheral(KEY3_PORT, &GPIO_InitStruct);
    KEY1_APBxClkCmd(KEY3_PORT_CLK, DISABLE);

    NVIC_InitType NVIC_InitStruct;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitStruct.NVIC_IRQChannel = KEY3_EXTI_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    EXTI_InitType EXTI_InitStruct;
    GPIO_ConfigEXTILine(KEY3_EXTI_PORTSRC, KEY3_EXTI_PINSRC);
    EXTI_InitStruct.EXTI_Line = KEY3_EXTI_LINE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitPeripheral(&EXTI_InitStruct);
}

void EXTI0_IRQHandler(void)
{
    EXTI_ClrStatusFlag(KEY3_EXTI_LINE);
}

void EXTI9_5_IRQHandler(void)
{
    ble_monitor_callback();
}

uint16_t adc_value_get(ADC_Module *ADCx)
{
    uint16_t adc_value[ADC_FILTER_NUM] = {
        0,
    };
    

    for (uint8_t i = 0; i < ADC_FILTER_NUM; i++)
    {
        adc_value[i] = ADC_GetSampleValue(ADCx,ADC_SAMPLE_CH_ADC, 1);
    }
    
    return ADC_data_filter(adc_value);
}

void hdc2010_init(void)
{
    uint16_t devId;
    reset();
    if (checkHDC2010Chip(&devId) == 1)
    {
        log_debug("HDC2010Chip dev id:0x%04x\r\n", devId);
        hdc2010_init_reg();
    }
    else
    {
        log_debug("HDC2010Chip dev fail:0x%04x\r\n", devId);
        return;
    }

}

void hdc2010_data_read(void)
{
    float temp_humid[2] = {0.0, 0.0};

    //while((readInterruptStatus() & DRDY_STATUS_READAY) != DRDY_STATUS_READAY);

    temp_humid[0] = readTemp();
    temp_humid[1] = readHumidity();

    ////log_debug("temp %2.1f humid %2.1f\r\n", temp_humid[0], temp_humid[1]);
    display_update_flag |= display_update(DISP_TEMP_HUMID_UPDATE, (uint8_t *)&temp_humid, sizeof(temp_humid));
    clearMaxTemp();
    clearMaxHumidity();
    //triggerMeasurement();
}

void buzzer_init(void)
{
    GPIO_InitType GPIO_InitStruct;

    BUZZER_APBxClkCmd(BUZZER_PORT_CLK, ENABLE); // PD0 for LED1
    GPIO_InitStruct.Pin        = BUZZER_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitPeripheral(BUZZER_PORT, &GPIO_InitStruct);
}

uint8_t mainboard_enter_stop2(void)
{
    log_debug("Enter Stop2 Mode!\r\n");
    SetSysClock_HSI();
    display_poweroff();
    hdc_i2c_gpio_deinit();
    qma_i2c_gpio_deinit();
    log_gpio_deinit();

    key_gpio_deinit();
    key_wakeup_setup();
    eif_gpio_DeInit();      //蓝牙休眠时需要调用此接口,以降低功耗
    TIM6_deinit();
    ADC_DeConfig(ADC_MODULE);
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);
    if(ble_monitor_wait(SystemCoreClock/100))
        return 0;
    
    led_gpio_deinit();
    
    PWR_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
    
    led_gpio_init();

    return 1;
}

void mainboard_exit_stop2(void)
{
    SystemCoreClock = 144000000;    //SYSCLK_FREQ;
    SystemInit();
    mainboard_module_init();
    log_debug("Exit Stop2 Mode!\r\n");
    log_debug("System reinit!\r\n");
    
    ble_hardware_reinit();
    log_debug("BLE reinit...\r\n");

    system_1s_flag = 1;
    display_update_flag |= display_update(DISP_BT_DISCONNECT, 0, 0);
    
    log_debug("System restart...\r\n");
}

int mainboard_module_init(void)
{
    uint32_t i;
    
    log_init();

    led_gpio_init();
    key_gpio_init();
    ADC_Config(ADC_MODULE, ADC_SAMPLE_CH_ADC);
    qma_i2c_gpio_init();
    hdc_i2c_gpio_init();

    qma7981_int();
    display_init();
    hdc2010_init();
    buzzer_init();

    BUZZER_ON();
    delay_ms(200);
    BUZZER_OFF();
    
    gKey.key    = 0;
    gKey.temp   = 0;
    
    for(i=0;i<KEY_NUM;i++)
    {
        gKey.cnt[i]     = 0;
        gKey.hold[i]    = 0;
    }
    
    LED4_ON();
    TIM6_init();

    return 0;
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
        
        SystemCoreClock = 144000000;
    }
    else
    { // If HSE fails to open, the program will come here, where the user can add the error code to handle.
      // When HSE fails or breaks down,mcu will automatically set HSI as the system clock.HSI is an internal high speed
      // clock of 8MHz.
        log_init();
        while (1)
        {
            printf("HSE fail!\r\n");
        }
    }
}

void SetSysClock_HSI(void)
{
    RCC_EnableHsi(ENABLE);
    while(RCC_GetFlagStatus(RCC_FLAG_HSIRD) == RESET);
    
    RCC_ConfigSysclk(RCC_SYSCLK_SRC_HSI);
    while (RCC_GetSysclkSrc() != 0x00);
    
    RCC_EnablePll(DISABLE);
    RCC_ConfigHse(RCC_HSE_DISABLE);
    
    SystemCoreClock = HSI_VALUE;
}

int main(void)
 {
    // const char *response = "bt salve say hello";
    int32_t ret,i;
    uint16_t adc_value  = 0;
    uint8_t ver[10];

    delay_ms(100);
    mainboard_module_init();
    log_debug("system start...\r\n");

    memcpy(bt_init.device_name, ble_name, MIN(strlen(ble_name), sizeof(bt_init.device_name)));
    memcpy(bt_init.device_addr, ble_addr, MIN(strlen(ble_addr), sizeof(bt_init.device_addr)));

    bt_init.service[0].svc_uuid                = SERVICE_UUID;
    bt_init.service[0].character[0].uuid       = USER_IDX_WRITE_NOTIFY_CHAR;
    bt_init.service[0].character[0].permission = BT_WRITE_PERM | BT_NTF_PERM;

    ret = bt_ware_init(&bt_init, (bt_event_callback_handler_t)bt_event_callback_func);
    if (ret != BT_RET_SUCCESS)
    {
        log_debug("bt init failed.\r\n");
    }
    else
    {
        log_debug("bt init ok.\r\n");
    }
    
    BT_get_version(ver);
    printf("ble driver version %s\r\n", ver);

    while (1)
    {
        bt_run_thread();

        if (system_1s_flag)
        {
            qma7981_read_raw_xyz(&g_sensor_data);
            display_update_flag |= display_update(DISP_GSENSOR_UPDATE, (uint8_t*)&g_sensor_data, sizeof(g_sensor_data));
            adc_value = adc_value_get(ADC_MODULE);
            display_update_flag |= display_update(DISP_ADC_UPDATE, (uint8_t*)&adc_value, sizeof(adc_value));
            hdc2010_data_read();
            ble_status_monitor();
            system_1s_flag = 0;
        }

        if(gKey.key == KEY_STOP)
        {
            gKey.key = 0;

            flag_bt_enter_sleep = 1;
            if(bt_connect_flag != 1)
            {
                mainboard_enter_stop2();
                mainboard_exit_stop2();
            }
            else
                ble_monitor_wait(SystemCoreClock/100);
            
            flag_bt_enter_sleep = 0;
        }
        else
            ble_monitor_wait(SystemCoreClock/100);
        
        if(flag_bt_irq)
        {
            flag_bt_irq = 0;
            bt_handler();
        }
            
        if(gKey.key == KEY_SEND)
        { 
            if(bt_connect_flag == 1)
            {
                log_debug("Key send Press!\r\n");
                for (i = 0; i < 10; i++)
                {
                    bt_data_buf[i + 2] = i;
                }
                bt_data_buf[0] = i;
                bt_data_buf[1] = 0;
                
                LED1_ON();
                bt_snd_data(bt_data_buf, 12, USER_IDX_WRITE_NOTIFY_VAL);
                LED1_OFF();
            }
            
            gKey.key = 0;
        }
        
        display_handle(&display_update_flag);

        if(bt_data_finish_flag == 1)
        {
            display_update_flag |= display_update(DISP_BT_RECDATA, bt_data_buf, sizeof(g_sensor_data));
            LED1_ON();
            bt_snd_data(bt_data_buf, bt_total_rxlen, USER_IDX_WRITE_NOTIFY_VAL);
            bt_total_rxlen   = 0;
            bt_data_rxlen = 0;
            bt_data_finish_flag = 0;
            LED1_OFF();
        }
    }
}
