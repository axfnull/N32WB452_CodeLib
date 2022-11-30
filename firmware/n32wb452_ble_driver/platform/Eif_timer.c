/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file Eif_timer.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include "Eif_timer.h"
#include "bsp_timer.h"
#include "ke_event.h"
//#include "Eif_iom.h"


/*************************************
*��������eif_timer_isr 
*���ܣ�����һ����ʱ���¼�
*������void
*
*����ֵ��void
*��ע��
**************************************/
void eif_timer_isr(void)
{
    ke_event_set(KE_EVENT_KE_TIMER);
}

/*************************************
*��������eif_timer_init 
*���ܣ���ʱ����ʼ���������Զ���װֵ�ͷ�Ƶϵ��
*������void
*
*����ֵ��void
*��ע��
**************************************/
void eif_timer_init(void)
{
    TIM3_config(9999,35);//36��Ƶ,1MHz����,10ms��ʱ�ж�
}

/*************************************
*��������eif_set_timeout 
*���ܣ����ó�ʱʱ��,10ms��ʱ����
*������to����ʱʱ��
*
*����ֵ��void
*��ע��
**************************************/
void eif_set_timeout(uint32_t to)
{
    TIM3_set_timeout(to);
}

/*************************************
*��������eif_get_time 
*���ܣ���ȡ��ʱʣ��ʱ��,��λms
*������void
*
*����ֵ��void
*��ע��
**************************************/
uint32_t eif_get_time(void)
{   
    return (TIM3_get_time());
}

/*************************************
*��������eif_enable_timer
*���ܣ�������رն�ʱ��
*������enable��1�������� 0���ر�
*
*����ֵ��void
*��ע��
**************************************/
void eif_enable_timer(bool enable)
{
    TIM3_IRQ_enable(enable);
}

