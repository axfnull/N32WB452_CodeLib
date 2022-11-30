1. ����˵��
    ������չʾ�� RT_Thread ϵͳ���� SPI �豸��������д���������� W25Q128

2. ʹ�û���
    Ӳ�����������̶�Ӧ�Ŀ���Ӳ��ƽ̨ 
    �����壺   N32WB45xL_EVB_V1.1

3. ʹ��˵��
    �������ģ�����÷���������:ʱ�ӣ�I/O�� 
         1. SystemClock��144MHz
         2. GPIO��LED:D1--PD0
	        SPI: NSS--PA4��SCK--PA5��MISO--PA6��MOSI--PA7
                        ��־��TX--PA9  RX--PA10�������ʣ�115200

    ����Demo�Ĳ��Բ�������� 
         1. ��������س���λ����
         2. ��������main()���洴�������̣߳�test0 �̺߳� test1 �̣߳�test0 �߳����ڿ��� D1 500ms��˸��test1�̶߳���д����������W25Q128�����Խ��ͨ�����ڴ�ӡ

4. ע������
    ע����W25Q128оƬ��Ӳ����������

1. Function description
    This routine shows how to create an SPI device on the RT_Thread system and read, write, and erase W25Q128

2. Use environment
    Hardware environment: development hardware platform corresponding to the project 
    Development board:      N32WB45xL_EVB_V1.1

3. Instructions for use
    Describe the configuration method of related modules; for example: clock, I/O, etc. 
        1. SystemClock: 144MHz
        2. GPIO: LED:D1--PD0
	      SPI: NSS--PA4��SCK--PA5��MISO--PA6��MOSI--PA7
                     Log: TX--PA9 RX--PA10 Baud rate: 115200

    Describe the test steps and phenomena of Demo 
        1. After compiling, download the program to reset and run;
        2. This routine creates two threads in main(), test0 thread and test1 thread, test0 thread is used to control D1 500ms flashing, test1 thread read, write, erase operation W25Q128, test results printed through the serial port

4. Matters needing attention
    Note the hardware connection problem with the W25Q128 chip
	