#ifndef _CRC16_H_
#define _CRC16_H_
#include "n32wb452.h"

#define CRC16(crc, ptr, len) cal_crc16(crc, ptr, len)
u16 cal_crc16(u32 crc, u8* ptr, u32 len);

#endif /* CRC16_H_ */
