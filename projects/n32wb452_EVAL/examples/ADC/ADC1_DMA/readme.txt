1、功能说明
    1、ADC1采样、转换PC2引脚的模拟电压
    2、其中ADC1转换结果通过DMA_CH1通道读取到变量ADCConvertedValue
2、使用环境
    软件开发环境：  KEIL MDK-ARM V5.26.2.0
    硬件环境：      基于N32WB45xL_EVB开发
3、使用说明
    系统配置；
        1、时钟源：
            HSE=8M,PLL=144M,AHB=144M,APB1=36M,APB2=72M,ADC CLK=144M/16,ADC 1M CLK=HSE/8,DMA CLK=144M
        2、端口配置：
            PC2选择为模拟功能ADC1转换通道8
        3、DMA：
            DMA1_CH1通道回环模式搬运一个半字的ADC1转换结果到ADCConvertedValue变量
        4、ADC：
            ADC1独立工作模式、连续转换、扫描模式、软件触发、12位数据右对齐，转换通道8即PC2的模拟电压数据
    使用方法：
        1、编译后打开调试模式，将变量ADCConvertedValue添加到watch窗口观察
        2、通过改变PC2引脚的电压，可以看到转换结果变量同步改变
4、注意事项
    当系统采用HSE时钟时（一般HSI也是打开的），RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8)可以配置为HSE或者HSI
    当系统采用HSI时钟时（一般HSE是关闭的），RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8)只能配置为HSI