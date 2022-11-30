1. ����˵��
    ������չʾ�� RT_Thread ϵͳ���� PIN �豸

2. ʹ�û���
    Ӳ�����������̶�Ӧ�Ŀ���Ӳ��ƽ̨ 
    �����壺   N32WB45xL_EVB_V1.1

3. ʹ��˵��
    �������ģ�����÷���������:ʱ�ӣ�I/O�� 
         1. SystemClock��144MHz
         2. GPIO��PD0 ���� LED(D1) ��˸;PD1 ���� LED(D10) ��˸;
                        KEY0 -- PD8, KEY1 -- PD9, KEYUP -- PA0
                        ��־��TX--PA9  RX--PA10�������ʣ�115200

    ����Demo�Ĳ��Բ�������� 
         1. ��������س���λ����
         2. �����̴��������̣߳�led0 �̡߳�led1 �߳� key �̣߳�led0 �߳����ڿ��� D1 500ms��˸��led1 �߳����ڿ��� D10 250ms��˸��
            key �̼߳�� KEY0��KEY1��KEYUP������KEYUP�����жϹ��ܣ���KEYUP����ʱ�������жϻص��������а�������ʱ��ӡ��Ӧ��־

4. ע������
    ��

1. Function description
    This routine shows to create PIN devices on the RT_Thread system

2. Use environment
    Hardware environment: development hardware platform corresponding to the project 
    Development board:      N32WB45xL_EVB_V1.1

3. Instructions for use
    Describe the configuration method of related modules; for example: clock, I/O, etc. 
        1. SystemClock: 144MHz
        2. GPIO: PD0 controls LED(D1) flashing; The PD1 controls LED(D10) flashing
                     KEY0 -- PD8, KEY1 -- PD9, KEYUP -- PA0
                     Log: TX--PA9 RX--PA10 Baud rate: 115200

    Describe the test steps and phenomena of Demo 
        1. After compiling, download the program to reset and run;
        2. This routine creates three threads, led0 thread, led1 thread key thread, led0 thread is used to control D1 500ms flashing, led1 thread is used to control D10 250ms flashing, 
            key thread detects KEY0, KEY1 and KEYUP, and the KEYUP opens the interrupt function, When KEYUP is pressed, the interrupt callback function is called, and the corresponding log is printed when a key is pressed

4. Matters needing attention
    None.