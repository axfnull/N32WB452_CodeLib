1、功能说明
    1、配置四个触摸按键为硬件检测模式，MCU依次进入SLEEP、STOP0、STOP2模式，可通过任意触摸按键唤醒

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
        1、在KEIL下编译后烧录到全功能板，通电，D1闪烁三次后进行SLEEP模式
        2、按下任意触摸按键，唤醒MCU，D10闪烁一次后，进入STOP0模式
        3、按下任意触摸按键，唤醒MCU，D11闪烁一次后，进入STOP2模式
        4、按下任意触摸按键，唤醒MCU，D12闪烁一次后，再次进入SLEEP模式
        
4、注意事项
    无。