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
 * @brief 智能锁的ADC采样源文件
 * @file HAL_ADC.c
 * @author Nations
 * @version v1.0.1
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include "n32wb452.h"
#include "bsp_adc.h"

/**
 * @brief  ADC Configuration
 * @param  0=BAT   1=NFC
 * @retval None
 */

//获得ADC值
// ch:通道值 0~3
u16 Get_Adc(ADC_Module *ADCx, adc_sample_ch ch)
{
    //设置指定ADC的规则组通道，一个序列，采样时间
    ADC_ConfigRegularChannel(ADCx, ch, 1, ADC_SAMP_TIME_28CYCLES5); // ADC通道,采样时间为239.5周期

    ADC_EnableSoftwareStartConv(ADCx, ENABLE); //使能指定的ADC的软件转换启动功能

    while (!ADC_GetFlagStatus(ADCx, ADC_FLAG_ENDC))
        ; //等待转换结束
    
    ADC_ClearFlag(ADCx, ADC_FLAG_ENDC);
    ADC_ClearFlag(ADCx, ADC_FLAG_STR);
    return ADC_GetDat(ADCx); //返回最近一次ADC规则组的转换结果
}

// BAT:CH == 0
// NFC:CH == 1
void ADC_Config(ADC_Module *ADCx, adc_sample_ch ch)
{
    ADC_InitType ADC_InitStructure;
    GPIO_InitType GPIO_InitStructure;

    /* ADC1 Periph clock enable */
    ADC_ENABLE_CLK();

    ADC_ENABLE_PORTCLK();
    
    /* Configure ADC Channel11 as analog input */
    GPIO_InitStructure.Pin          = GPIO_PIN_2;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed   = GPIO_INPUT;
    GPIO_InitPeripheral(ADC_PORT, &GPIO_InitStructure);

    RCC_ConfigAdcPllClk(RCC_ADCPLLCLK_DIV6, ENABLE); //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M

    /* ADCs DeInit */
    ADC_DeInit(ADCx);

    /* Initialize ADC structure */
    ADC_InitStruct(&ADC_InitStructure);

    /* Configure the ADC1 in continuous mode with a resolution equal to 12 bits  */
    ADC_InitStructure.WorkMode       = ADC_WORKMODE_INDEPENDENT; // ADC工作在独立模式
    ADC_InitStructure.MultiChEn      = DISABLE;                  //模数转换工作在单通道模式
    ADC_InitStructure.ContinueConvEn = DISABLE;                  //模数转换工作在单次转换模式
    ADC_InitStructure.ExtTrigSelect  = ADC_EXT_TRIGCONV_NONE;    //转换由软件而不是外部触发启动
    ADC_InitStructure.DatAlign       = ADC_DAT_ALIGN_R;          // ADC数据右对齐
    ADC_InitStructure.ChsNumber      = 1;                        //顺序进行规则转换的ADC通道的数目
    ADC_Init(ADCx, &ADC_InitStructure);                          //根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器

    ADC_Enable(ADCx, ENABLE); //使能指定的ADC1
    
    /*Check ADC Ready*/
    while(ADC_GetFlagStatusNew(ADCx,ADC_FLAG_RDY) == RESET);


    ADC_StartCalibration(ADCx); //开启AD校准

    while (ADC_GetCalibrationStatus(ADCx))
        ; //等待校准结束
}

void ADC_DeConfig(ADC_Module *ADCx)
{
    ADC_DeInit(ADCx);
    ADC_Enable(ADCx, DISABLE);
}

u16 ADC_GetSampleValue(ADC_Module *ADCx, adc_sample_ch ch, u8 times)
{
    u32 temp_val  = 0;
    u32 temp_val1 = 0;
    u8 t;

    for (t = 0; t < times; t++)
    {
        temp_val1 = Get_Adc(ADCx, ch);
        temp_val += temp_val1;
        // delay_ms(5);
        // printf("[%d]temp_val=%3.3fV.\r\n", t, (temp_val1 / 4095.0) * 3.3f);
    }
    return temp_val / times;
}

uint16_t ADC_data_filter(uint16_t* adc_v)
{
    uint16_t max_v = 0, min_v = 0xfff, sum_v = 0;
    uint8_t i;

    for (i = 0; i < ADC_FILTER_NUM; i++)
    {
        if (adc_v[i] > max_v)
            max_v = adc_v[i];
        if (adc_v[i] < min_v)
            min_v = adc_v[i];
        sum_v += adc_v[i];
    }
    sum_v = sum_v - max_v - min_v;
    sum_v >>= 3;
    // printf("battery_value_get filter: %04x %3.3f\r\n", sum_v, (sum_v / 4096.0) * 3.3f);
    return sum_v;
}
