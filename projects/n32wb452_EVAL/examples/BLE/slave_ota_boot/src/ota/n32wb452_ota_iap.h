
#ifndef __IAP_H
#define __IAP_H

void set_vector_table(void);
uint8_t jump2app(uint32_t appAddr);
void jump2iap(uint32_t iapAddr);


#endif

