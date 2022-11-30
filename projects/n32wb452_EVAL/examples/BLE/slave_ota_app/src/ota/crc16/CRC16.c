#include "CRC16.h"

// ------------------ POPULAR POLYNOMIALS -----------------------
//  CRC16:          x^16 + x^15 + x^2 + x^0         (0x8005)
//  CRC16_CCITT:    x^16 + x^12 + x^5 + x^0         (0x1021)
// --------------------------------------------------------------
#define CRC_16      0x8005
#define CRC16_CCITT 0x1021
#define CRC_Select  CRC_16

u16 cal_crc16(u32 crc, u8* ptr, u32 len)
{
    u32 k;

    while (len-- != 0)
    {
        for (k = 0x80; k != 0; k /= 2)
        {
            if ((crc & 0x8000) != 0) //  余式CRC*2 再求CRC
            {
                crc *= 2;
                crc ^= 0x8005;
            }
            else
            {
                crc *= 2;
            }

            if ((*ptr & k) != 0) //  加本位的CRC
            {
                crc ^= 0x8005;
            }
            if (crc > 0x10000)
            {
                crc -= 0x10000;
            }
        }
        ptr++;
    }
    return crc;
}
