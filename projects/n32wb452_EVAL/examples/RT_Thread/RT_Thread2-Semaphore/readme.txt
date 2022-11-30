1. 功能说明
    此例程展示在 RT_Thread 信号量的创建、获取和释放

2. 使用环境
    硬件环境：工程对应的开发硬件平台 
    开发板：   N32WB452XL_EVB V1.1

3. 使用说明
    描述相关模块配置方法；例如:时钟，I/O等 
         1. SystemClock：144MHz
         2. GPIO：PD0 控制 LED(D1) 亮灭；PD1 控制 LED(D10) 闪烁
                        KEY： PD8（KEY04）

    描述Demo的测试步骤和现象 
         1. 编译后下载程序复位运行；
         2. 本例程创建三个线程，LED0 线程、 LED1 线程 和 KEY 线程，LED0 线程用于控制 D1 亮灭，LED1 线程用于控制 D10 250ms闪烁，
             KEY 线程扫描按键，当 KEY04 检测到按下时，释放信号量，LED0 线程获取信号量，获取成功后，翻转 D1

4. 注意事项
    无

1. Function description
    This example shows the creation, acquisition and release of the RT_Thread semaphore

2. Use environment
    Hardware environment: development hardware platform corresponding to the project 
    Development board:      N32WB452XL_EVB V1.1

3. Instructions for use
    Describe the configuration method of related modules; for example: clock, I/O, etc. 
        1. SystemClock: 144MHz
        2. GPIO: 
                    PD0 controls the LED (D1) to blink; PD1 controls the LED (D10) to blink;
                    KEY: PD8（KEY04）

    Describe the test steps and phenomena of Demo 
        1. After compiling, download the program to reset and run;
        2. This routine creates three threads, LED0 thread, LED1 thread and KEY thread, LED0 thread is used to control D1 on and off, LED1 thread is used to control D10 250ms blink, 
            KEY thread scans the KEY, when KEY04 detects that it is pressed, release the semaphore, LED0 thread obtains the semaphore, and after obtaining it successfully, reverses D1

4. Matters needing attention
    None.