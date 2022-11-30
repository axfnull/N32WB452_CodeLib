#ifndef _HAL_ADC_H_
#define _HAL_ADC_H_

#include "n32wb452.h"

#define ADC_FILTER_NUM (10)

/**
 * @brief ADC采样通道定义
 */
typedef enum adc_sample_ch_e
{
    ADC_SAMPLE_CH_ADC = ADC_CH_13, ///< PB2-定义ADC采样
    ADC_SAMPLE_CH_MAX
} adc_sample_ch;

#define ADC_MODULE          ADC2
#define ADC_ENABLE_CLK()    (RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC2, ENABLE))

#define ADC_ENABLE_PORTCLK()    (RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE))
#define ADC_PORT            GPIOB
#define ADC_PIN             GPIO_PIN_2

void ADC_Config(ADC_Module *ADCx, adc_sample_ch ch);
void ADC_DeConfig(ADC_Module *ADCx);
u16 ADC_GetSampleValue(ADC_Module *ADCx, adc_sample_ch ch, u8 times);
uint16_t ADC_data_filter(uint16_t* adc_v);

#endif
