1、功能说明

    1、此例程展示控制 LED（D1、D10、D11、D12）闪烁

2、使用环境

    /* 硬件环境：工程对应的开发硬件平台 */
    开发板：N32WB45xL_EVB V1.1

3、使用说明
    
    /* 描述相关模块配置方法；例如:时钟，I/O等 */
    SystemClock：144MHz
    GPIO：PD0 控制 LED(D1) 常亮；PD1 控制 LED(D10) 闪烁；PE2 控制 LED(D11) 闪烁；PE3 控制 LED(D12) 闪烁


    /* 描述Demo的测试步骤和现象 */
    1.编译后下载程序复位运行
    2.D6常亮，D17 闪烁

4、注意事项
    无
	
	
	
1. Function description
    1. This example shows control LED (D1、D10、D11、D12) flashing.
	
2. Use environment
    /*Hardware environment: Project corresponding development hardware platform*/
	1.Developed based on the evaluation board N32WB45xL_EVB V1.1
	
3. Instructions for use
    /* Describe the related module configuration method; For example: clock, I/O, etc. */
	1. SystemClock: 144 MHZ
	2. GPIO: PD0 control LED(D1) steady on; PD1 control LED(D10) flashing; PE2 controls LED(D11) flashing; PE3 controls LED(D12) flashing.
	
	/* Describes the test steps and symptoms of Demo */
	1. The downloaded program is reset and running after compilation
	2. D0 is steady on, and D10，D11,D12 is blinking.
		
4. Matters needing attention
   None.