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