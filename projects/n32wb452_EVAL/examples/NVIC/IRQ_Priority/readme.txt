1、功能说明

    /* 简单描述工程功能 */
        这个例程配置并演示NVIC优先级设置


2、使用环境

    /* 硬件环境：工程对应的开发硬件平台 */
        开发板：N32WB45xL_EVB V1.1
        

3、使用说明
    
    /* 描述相关模块配置方法；例如:时钟，I/O等 */
        SystemClock：144MHz
        USART：TX - PA9，波特率115200
        EXIT：PA0为浮空输入模式，外部中断线 - EXIT_LINE0，开启外部中断，优先级为0
        SysTick：中断优先级设置为0

    /* 描述Demo的测试步骤和现象 */
        1.编译后下载程序复位运行；
        2.程序正常运行会打印“Key_Status = 0”初始状态，此时SysTick中断优先级高于外部中断；
        3.按下按键WAKEUP(PA0)，触发外部中断并看到外部中断处理函数打印开始信息，在中断处理函数里触发SysTick中断，
          SysTick中断会打断外部中断处理函数，并可以看到SysTick中断信息打印，同时Key_Status值被修改为1，在完成
          SysTick中断处理函数后会继续执行外部中断处理函数，并看到外部中断处理函数打印结束信息；
        4.主循环中检测到Key_Status = 1，会将SysTick中断优先级修改到低于外部中断；
        5.按下按键WAKEUP(PA0)，触发外部中断并看到外部中断处理函数开始打印信息，在中断处理函数里触发SysTick中断，
          SysTick中断不会打断外部中断处理函数，等到外部中断处理函数结束并打印结束信息后开始执行SysTick中断处理函
          数，并可以看到SysTick中断信息打印，Key_Status值被修改为2；
        6.主循环中检测到Key_Status = 2，打印程序运行结束信息；
        7.芯片复位后可以重新开始；


4、注意事项


1. Function description
	/* A brief description of the engineering function */
	This routine configures and demonstrates NVIC priority Settings

2. Use environment
	/* Hardware environment: the corresponding development hardware platform */
	Development board: N32WB45xL_EVB V1.1
        
3. Instructions for use    
	/* Describe the related module configuration method; For example: clock, I/O, etc. */
	SystemClock: 144 MHZ
	USART: tX-PA9, baud rate 115200
	EXIT: PA0 is floating input mode, external interrupt line -exit_line0, external interrupt is enabled, and the priority is 0
	SysTick: Set the interrupt priority to 0

	/* Describes the test steps and symptoms of Demo */
	1. Download the program after compilation and reset it;

	2. The initial status of "key_status = 0" will be printed when the program runs normally. At this time, systick interrupt priority is higher than external interrupt;

	3. Press the key wakeup (PA0), trigger the external interrupt and see the printing start information of the external interrupt processing function, and trigger the systick interrupt in the interrupt processing function,

	   Systick interrupt will interrupt the external interrupt processing function, and you can see the systick interrupt information print, and the key_ The status value is modified to 1, and the

	   After systick interrupt processing function, it will continue to execute the external interrupt processing function and see the printing end information of the external interrupt processing function;

	4.  Detected Key_Status = 1 in main cycle, the systick interrupt priority will be modified to be lower than the external interrupt;

	5. Press the key wakeup (PA0), trigger the external interrupt and see that the external interrupt processing function starts printing information, and trigger the systick interrupt in the interrupt processing function,

	   Systick interrupt will not interrupt the external interrupt processing function. Wait until the external interrupt processing function ends and the end message is printed then executing the systick interrupt processing function, and  can see systick interrupt information printing, key_ The status value is modified to 2;

	6. Detected Key_Status = 2 in main cycle, print the program running end information;

	7. Restart after the chip is reset;

4. Matters needing attention