1. 功能说明
    USB  Joystick Mouse 设备

2. 使用环境
    硬件环境：工程对应的开发硬件平台 
    开发板：    N32WB45xL_EVB V1.1

3. 使用说明
    描述相关模块配置方法；例如:时钟，I/O等 
         1. SystemClock：144MHz
         2. USBClock: 48MHz
         3. GPIO：右KEY1（PA4）、左KEY2（PA5）、上KEY3（PA6）、下KEY4（PA7）。

    描述Demo的测试步骤和现象 
         1. 编译后下载程序复位运行；
         2. 通过 USB 线连接 J4 USB 口，挂在完成以后，用杜邦线连接 KEY1、KEY2、KEY3、KEY4 到 GND 鼠标会上下左右移动。

4. 注意事项
    无

1. Function description
    USB Joystick Mouse device

2. Use environment
    Hardware environment: development hardware platform corresponding to the project 
    Development board:      N32WB45xL_EVB V1.1

3. Instructions for use
    Describe the configuration method of related modules; for example: clock, I/O, etc. 
        1. SystemClock: 144MHz
        2. USBClock: 48MHz
        3. GPIO: right KEY1 (PA4), left KEY2 (PA5), upper KEY3 (PA6), lower KEY4 (PA7)
                  
    Describe the test steps and phenomena of Demo 
        1. After compiling, download the program to reset and run;
        2. Connect the J4 USB port through a USB cable, After the USB is mounted, connect KEY1、KEY2、KEY3、KEY4 to GND with DuPont line, and the mouse will move up, down, left and right.
 
4. Matters needing attention
    None.