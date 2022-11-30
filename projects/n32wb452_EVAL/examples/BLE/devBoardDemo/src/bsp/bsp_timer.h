#ifndef _BSP_TIMER_H_
#define _BSP_TIMER_H_

#include "n32wb452.h"
#include "n32wb452_tim.h"

void TIM3_config(uint16_t arr,uint16_t psc);
void TIM3_IRQ_enable(uint8_t flag);
void TIM3_set_timeout(uint32_t count);
uint32_t TIM3_get_time(void);

#endif
