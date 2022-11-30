1、功能说明
    此例程用于演示开发板N32WB45xL_EVB的功能。

2、使用环境

    软件开发环境：
        IDE工具：KEIL MDK-ARM 5.26.2.0
    
    硬件环境：
        开发板 N32WB45xL_EVB


3、使用说明
    
    1、主时钟：144MHz (HSE+PLL)
    2、USART1配置：
            TX  -->  PA9
            波特率：115200
            数据位：8bit
            停止位：1bit
            无校验
    3、温湿度传感器：HDC2010
             接口：I2C2
             SCL：PB10
             SDA：PB11
    4、三轴加速器：QMA7981
            接口：I2C1
            SCL：PB8
            SDA：PB9
    5、LCD：1.3inch 240*240 LCD模组
            驱动芯片：ST7789
            接口：SPI2
            SPI_CLK:    PE11
            SPI_MOSI:   PE13
            RST:        PE7
            BLK:        PE8
            DC:         PE9
            CS:         PE10
    6、按键：
            S3:     PA0(WKUP)
            K04:    PD8
            K05:    PD9
    7、LED：
            D1:     PD0 (蓝牙数据发送状态指示灯)
            D10:    PD1 (蓝牙数据接收状态指示灯)
            D11:    PE2 (蓝牙连接状态指示灯)
            D12:    PE3 (系统运行状态指示灯)
    7、ADC：
            CH1:    PB2
            
    8、测试步骤与现象
        1、编译下载代码到开发板，复位，MCU进入正常工作状态，D12闪烁（0.5Hz），D1、D10、D11熄灭
        2、LCD点亮，所有数据定时刷新：
            第1、2行固定显示公司名称及DEMO信息
            第3行显示蓝牙设备名称
            第4行显示蓝牙工作状态（连接、断开、收到的数据长度）
            第5行在蓝牙收到数据后以HEX格式显示前5个字节
            第6、7、8行显示三轴加速器原始数据
            第9行显示可调电阻RP1的采样电压
            第10行显示温度和湿度
           
        3、打开手机APP（NS_BlueTooth）搜索蓝牙广播（WB452xxxx）
        4、连接此设备，D11常亮，LCD上更新连接状态
        5、连接后可进行回环测试，接收数据时D10闪烁，发送数据时D1闪烁
        6、同时LCD上显示接收到数据包长度及前5个字节数据
        7、连接后，轻按K05，主动发送10字节数据
        8、断开连接后，D11熄灭
        
        9、调节可调电阻，可在LCD上看到ADC采样电压的变化
        10、随着环境温度、温度的变化，也可在LCD上实时显示当前温湿度值
        
        11、在没有蓝牙连接时，轻按K04，MCU进入STOP2休眠模式，所有LED熄灭，LCD熄灭
        12、在休眠模式下，仍可通过手机APP（NS_BlueTooth）搜索蓝牙广播（WB452xxxx）
        13、此时可通过连接蓝牙设备、或轻按按键S3唤醒MCU，返回正常工作状态，循环演示
        14、可通过串口查看部分提示信息

4、注意事项
    1、必须连接跳线J17（ADC）、J25/26（三轴加速器）、J22/23/24（温湿度传感器）
    2、必须正确安装LCD（U6）

1. Function description
	This example is used to demonstrate the Bluetooth OTA upgrade function.

2. Use environment
	Software development environment	
		IDE tool：KEIL MDK-ARM 5.26.2.0
	Hardware environment: 
		Development board: N32WB45xL_EVB V1.1

3. Instructions for use

	1. Main clock: 144MHz (HSE+PLL)
    	2. USART1 configuration:
		TX --> PA9
            		Baud rate: 115200
            		Data bits: 8bit
            		Stop bit: 1bit
            		no verification
    	3. Temperature and humidity sensor: HDC2010
             		Interface: I2C2
             		SCL: PB10
             		SDA: PB11
    	4. Three-axis accelerator: QMA7981
            		Interface: I2C1
            		SCL: PB8
            		SDA: PB9
    	5. LCD: 1.3inch 240*240 LCD module
            		Driver chip: ST7789
            		Interface: SPI2
            		SPI_CLK: PE11
            		SPI_MOSI: PE13
            		RST: PE7
            		BLK: PE8
           		DC: PE9
            		CS: PE10
    	6. Button:
            		S3: PA0 (WKUP)
            		K04: PD8
            		K05: PD9
    	7. LED:
            		D1: PD0 (Bluetooth data sending status indicator)
            		D10: PD1 (Bluetooth data reception status indicator)
            		D11: PE2 (Bluetooth connection status indicator)
            		D12: PE3 (system running status indicator)
    	7. ADC:
            		CH1: PB2
            
	8. Test steps and phenomena
        		1. Compile and download the code to the development board, reset, the MCU enters the normal working state, D12 flashes (0.5Hz), D1, D10, D11 are off
        		2. The LCD is lit, and all data are refreshed regularly:
           		 Lines 1 and 2 fixedly display the company name and DEMO information
            		Line 3 shows the bluetooth device name
            		Line 4 shows the bluetooth working status (connected, disconnected, received data length)
            		Line 5 displays the first 5 bytes in HEX format after bluetooth received the data
            		Lines 6, 7, and 8 show the raw data of the three-axis accelerator
            		Line 9 shows the sampled voltage of the adjustable resistor RP1
            		Line 10 shows temperature and humidity
           
        		3. Open the mobile APP (NS_BlueTooth) to search for Bluetooth broadcast (WB452xxxx)
        		4. Connect this device, D11 is always on, and the connection status is updated on the LCD
        		5. After the connection, the loopback test can be performed, D10 flashes when receiving data, and D1 flashes when sending data
        		6. At the same time, the length of the received data packet and the first 5 bytes of data are displayed on the LCD
        		7. After connecting, tap K05 to send 10 bytes of data actively
        		8. After disconnection, D11 goes out
        
        		9. Adjust the adjustable resistance, you can see the change of ADC sampling voltage on the LCD
        		10. With the change of ambient temperature and temperature, the current temperature and humidity value can also be displayed on the LCD in real time
        
        		11. When there is no Bluetooth connection, tap K04, the MCU enters the STOP2 sleep mode, all LEDs are off, and the LCD is off
        		12. In sleep mode, you can still search for Bluetooth broadcast (WB452xxxx) through the mobile APP (NS_BlueTooth)
        		13. At this time, you can wake up the MCU by connecting a Bluetooth device, or pressing the button S3, return to the normal working state, and cycle the demonstration
       		14. Some prompt information can be viewed through the serial port

4. Matters needing attention

	1. Jumpers J17 (ADC), J25/26 (three-axis accelerator), J22/23/24 (temperature and humidity sensor) must be connected
     	2. The LCD (U6) must be installed correctly