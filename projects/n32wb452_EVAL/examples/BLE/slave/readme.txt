1、功能说明

    此例程用于演示BLE模块功能，通过手机APP软件，可以跟蓝牙模块通信，收发数据。
    

2、使用环境

    软件开发环境：
        IDE工具：KEIL MDK-ARM 5.26.2.0
    
    硬件环境：
        开发板 N32WB45xL_EVB


3、使用说明
    
    1、时钟源：HSE+PLL
    2、主时钟：144MHz
    3、USART1配置：
            TX  -->  PA9            50MHz，AF_PP
            波特率：115200
            数据位：8bit
            停止位：1bit
            无校验

    4、测试步骤与现象
        1、编译下载代码到开发板，复位运行，D2、D3点亮
        2、打开手机APP（NS_BlueTooth）搜索蓝牙广播,可看到设备“WB452xxxx”
        3、连接设备WB452xxxx，连接成功后D2闪烁，此时可进入回环测试，设置发送数据的长度和发送次数，开始测试
        4、断开设备连接或不连接，D2常亮，
        5、持续无连接约10秒后MCU进入STOP2休眠模式，D2、D3熄灭
        6、STOP2下通过RTC约1秒定时唤醒，可看到D2约1秒短暂点亮一次
        7、STOP2下仍可通过手机APP搜索设备“WB452xxxx”并连接，连接成功后MCU自动唤醒，D2闪烁，可进行回环测试
        8、断开设备连接，D2常亮
        9、回到第5步，重复演示

4、注意事项
    无


1. Function description
	This example is used to demonstrate the function of the BLE module. Through the mobile phone APP software, it can communicate with the Bluetooth module and send and receive data.

2. Use environment
	Software development environment	
		IDE tool：KEIL MDK-ARM 5.26.2.0
	Hardware environment: 
		Development board: N32WB45xL_EVB V1.1

3. Instructions for use

	1. Clock source: HSE+PLL
	2. Main clock: 144MHz
    	3. USART1 configuration:
		TX --> PA9	50MHz，AF_PP
            		Baud rate: 115200
            		Data bits: 8bit
            		Stop bit: 1bit
            		no verification

	4. Test steps and phenomena
         		1. Compile and download the code to the development board, reset and run, D2 and D3 light up
         		2. Open the mobile APP (NS_BlueTooth) to search for Bluetooth broadcast, you can see the device "WB452xxxx"
         		3. Connect the device WB452xxxx. After the connection is successful, D2 flashes. At this time, you can enter the loopback test, set the length of the sent data and the number of times, and start the test.
         		4. Disconnect or disconnect the device, D2 is always on,
         		5. After about 10 seconds without connection, the MCU enters the STOP2 sleep mode, and D2 and D3 go out.
         		6. Under STOP2, wake up regularly through RTC for about 1 second, you can see that D2 is briefly lit once about 1 second
         		7. In STOP2, you can still search for the device "WB452xxxx" through the mobile phone APP and connect it. After the connection is successful, the MCU wakes up automatically, D2 flashes, and the loopback test can be performed.
        		8. Disconnect the device, D2 is always on
         		9. Go back to step 5 and repeat the demonstration

4. Matters needing attention

	None