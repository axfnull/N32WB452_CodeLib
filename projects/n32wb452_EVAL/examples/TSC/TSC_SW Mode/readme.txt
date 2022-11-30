1、功能说明
    1、轮询个触摸按键T3、T4、T5、T6

2、使用环境

    软件开发环境：KEIL MDK-ARM V5.26
    硬件环境：基于N32WB45xL-EVB开发

3、使用说明
    
    系统配置；
        1、时钟源：HSE+PLL
        2、系统时钟：16MHz
        3、TSC端口： PD2:CH15  PC11:CH13  PC12:CH14  PC10:CH12
        4、LED控制端口：PD0:D1  PD1:D10  PE2:D11  PE3:D12
        5、串口：PA9（TX）,115200bps，8bit data，1bit stop

    使用方法：
        1、在KEIL下编译后烧录到全功能板，通电，
        2、有触摸按键时，对应LED点亮，松开按键，对应LED熄灭，同时串口输出相关信息
           T3-D1  T4-D10  T5-D11  T6-D12
        
4、注意事项
    无