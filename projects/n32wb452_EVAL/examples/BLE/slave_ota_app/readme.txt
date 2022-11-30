1、功能说明

    此例程用于演示蓝牙OTA升级功能。
    

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
    4、FLASH:  W25Q128
        接口：SPI3
        SPI_CS    PA15
        SPI_CLK   PB3
        SPI_MOSI  PB5
        SPI_MISO  PB4

    5、测试步骤与现象
        1、在PC上打开串口工具
        2、先烧录BOOT程序（slave_ota_boot例程），再编译下载代码到开发板，复位运行
        3、在串口工具上可看到MCU先运行BOOT程序，然后跳转到当前APP,初始化蓝牙设备，D11点亮
        4、在手机上打开APP“门锁OTA_Demo”，点击“搜索设备”，在弹出窗口中可看到蓝牙设备“WB452_OTA”
        5、选择此设备，回到主界面，点击“选择升级文件”，浏览并选择需要升级的文件（*.zip）
        6、点击“开始升级”，D12闪烁，等待直至升级完成
        7、升级进程中可通过串口工具实时查看相关信息
        8、升级完成后，自动跳转到BOOT程序，回到第3步，循环演示
        
    6、升级包制作方法
        1、工程目录中__Get_OTA_Info.exe、GetInfo.bat用于生成升级文件信息，ota_file文件夹下有升级文件示例
        2、编译后运行GetInfo.bat，在工程目录下生成*.bin文件及__OTA_FW_INTO.txt文件
        3、解压升级文件示例中的ver_info.json文件
        4、将ver_info.json中的相关信息参照__OTA_FW_INTO.txt以及bin文件名修改
        5、将修改后的ver_info.json文件和ble_ota.bin一起压缩为成升级文件*.zip即可

4、注意事项
    必须与slave_ota_boot例程配套使用。
    必须连接跳线J1、J30、J21、J32


1. Function description
	This example is used to demonstrate the Bluetooth OTA upgrade function.

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

	4. FLASH: W25Q128
        		Interface: SPI3
        		SPI_CS PA15
        		SPI_CLK PB3
       		SPI_MOSI PB5
        		SPI_MISO PB4

    	5. Test steps and phenomena
        		1. Open the serial port tool on the PC
        		2. First burn the BOOT program (slave_ota_boot example), then compile and download the code to the development board, reset and run
        		3. On the serial port tool, you can see that the MCU first runs the BOOT program, then jumps to the current APP, initializes the Bluetooth device, and D11 lights up
        		4. Open the APP "Door Lock OTA_Demo" on the mobile phone, click "Search Device", and you can see the Bluetooth device "WB452_OTA" in the pop-up window
        		5. Select this device, return to the main interface, click "Select Upgrade File", browse and select the file (*.zip) to be upgraded
        		6. Click "Start Upgrade", D12 flashes, wait until the upgrade is complete
        		7. During the upgrade process, the relevant information can be viewed in real time through the serial port tool
        		8. After the upgrade is completed, it will automatically jump to the BOOT program, return to step 3, and cycle the demonstration
        
    	6. How to make an upgrade package
        		1. __Get_OTA_Info.exe and GetInfo.bat in the project directory are used to generate upgrade file information, and there are upgrade file examples in the ota_file folder
        		2. Run GetInfo.bat after compiling, and generate *.bin file and __OTA_FW_INTO.txt file in the project directory
        		3. Unzip the ver_info.json file in the upgrade file example
        		4. Refer to __OTA_FW_INTO.txt and the bin file name for the relevant information in ver_info.json
        		5. Compress the modified ver_info.json file and ble_ota.bin together into an upgrade file *.zip

4. Matters needing attention

	Must be used in conjunction with the slave_ota_boot example.
     	Jumpers J1, J30, J21, J32 must be connected