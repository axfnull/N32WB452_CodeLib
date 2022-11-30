1、功能说明
    通过DVP接口从摄像头采集数据

2、使用环境
    软件开发环境：KEIL MDK-ARM V5.26
    
    硬件环境：
        1、基于全功能板N32WB45xL_EVB V1.1 开发
        2、摄像头：GC0308

3、使用说明
    系统配置；
        1、时钟源：HSE+PLL
        2、系统时钟：144MHz
        3、DMA：DMA2通道8
        4、MCO：PA8
        5、DVP端口：PA1:HSYNC   PA2:VSYNC   PA3:PCLK
                    PA4:D0      PA5:D1      PA6:D2      PA7:D3
                    PC4:D4      PC5:D5      PB0:D6      PB1:D7
        6、摄像头控制端口：PA8:MCLK  PB13:RESET  PB12:PWDN  PC0:SCL  PC1:SDA
        7、串口：USART1，115200BPS，8bit data，1bit STOP
                TX:PA9

    使用方法：
        在KEIL下编译后烧录到全功能板，通电，通过串口工具查看演示结果
        间隔约1s采集一次摄像头数据并输出提示信息。


4、注意事项
    1、确保已正确接入摄像头
    2、用跳线帽短接J8（摄像头电源）


1. Function description
	Collect data from the camera through DVP interface.

2. Use environment
	Software development environment: KEIL MDK-ARM V5.26
    
	Hardware environment:
		1. based on the full function board N32WB45xL-EVB V1.1 development
		2. camera: GC0308 (640*480)

3. Instructions for use
	System configuration;
		1. Clock source: HSE+PLL
		2. System clock: 144MHz
		3. DMA: DMA2 channel 8
		4. MCO: PA8
		5. DVP port:
			PA1:HSYNC   PA2:VSYNC   PA3:PCLK
            PA4:D0      PA5:D1      PA6:D2      PA7:D3
            PC4:D4      PC5:D5      PB0:D6      PB1:D7
		6. Camera control port: PA8:MCLK  PB13:RESET  PB12:PWDN  PC0:SCL  PC1:SDA
		7. UART Port: USART1，115200BPS，8bit data，1bit STOP
                TX:PA9

	Usage:
		After compiling in KEIL, burn to the full function board, power on, you can view the demo results using the serial port tool.
        Collect camera data at an interval of about 1s and output a prompt message.

4. Precautions
	1. Ensure the camera is properly connected.
	2. Short connect J8(camera power)with jumper cap.