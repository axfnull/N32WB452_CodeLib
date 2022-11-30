1. 功能说明
    此例程展示在 RT_Thread 系统创建线程

2. 使用环境
    硬件环境：工程对应的开发硬件平台 
    开发板：    N32WB452XL_EVB V1.1

3. 使用说明
    描述相关模块配置方法；例如:时钟，I/O等 
         1. SystemClock：144MHz
         2. GPIO：PD0 控制 LED(D1) 闪烁；PD1 控制 LED(D10) 闪烁

    描述Demo的测试步骤和现象 
         1. 编译后下载程序复位运行；
         2. 本例程创建两个线程，LED0 线程和 LED1 线程，LED0 线程用于控制 D1 500ms闪烁，LED1 线程用于控制 D10 250ms闪烁，以此循环

4. 注意事项
    无

1. Function description
    This example shows how to create a thread in the RT_Thread system

2. Use environment
    Hardware environment: development hardware platform corresponding to the project 
    Development board:      N32WB452XL_EVB V1.1

3. Instructions for use
    Describe the configuration method of related modules; for example: clock, I/O, etc. 
        1. SystemClock: 144MHz
        2. GPIO: 
                    PD0 controls the LED (D1) to blink; PD1 controls the LED (D10) to blink;

    Describe the test steps and phenomena of Demo 
        1. After compiling, download the program to reset and run;
        2. This routine creates two threads, LED0 thread and LED1 thread. LED0 thread is used to control D1 500ms flashing, and LED1 thread is used to control D10 250ms flashing, so as to cycle

4. Matters needing attention
    None.