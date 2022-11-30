﻿1. 功能说明
    此例程展示在 RT_Thread 邮件的创建、发送和接收

2. 使用环境
    硬件环境：工程对应的开发硬件平台 
    开发板：   N32WB452XL_EVB V1.1

3. 使用说明
    描述相关模块配置方法；例如:时钟，I/O等 
         1. SystemClock：144MHz
         2. GPIO：KEY：PD8(KEY04),PD9(KEY05), PA0(KEYUP)
                        日志：TX--PA9  RX--PA10，波特率：115200

    描述Demo的测试步骤和现象 
         1. 编译后下载程序复位运行
         2. 接好串口打印工具
         3. 本例程创建两个线程，key 线程和 test 线程，key 线程检测按键发送邮件，test 线程接收邮件，按下按键（KEY04、KEY05、KEYUP）发送邮件，查看打印日志

4. 注意事项
    无

1. Function description
    This routine shows the creation, sending, and receiving of a mail  in RT_Thread

2. Use environment
    Hardware environment: development hardware platform corresponding to the project 
    Development board:      N32WB452XL_EVB V1.1

3. Instructions for use
    Describe the configuration method of related modules; for example: clock, I/O, etc. 
        1. SystemClock: 144MHz
        2. GPIO: KEY: PD8(KEY04),PD9(KEY05), PA0(KEYUP)
                     Log: TX--PA9 RX--PA10 Baud rate: 115200

    Describe the test steps and phenomena of Demo 
        1. After compiling, download the program to reset and run;
        2. Connect the serial port printing tool;
        3. This routine creates two threads, key thread and test thread, key thread detects the key to send mail, test thread receives mail, press the key (KEY04, KEY05, KEYUP) to send mail, and view the print log

4. Matters needing attention
    None.