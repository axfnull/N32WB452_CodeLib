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
 ******************************************************************************
 * @file n32wb452_data_fifo.h
 * @author Nations
 * @version v0.7.6
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 * @date    2020-05-8
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __N32WB452_DATA_FIFO_H__
#define __N32WB452_DATA_FIFO_H__
#include "n32wb452.h"

#define E_OK        (0)     /* No Error                */
#define E_MSGFULL   (-7)    /* msg of buffer is full   */
#define E_NODATA    (-8)    /* no data                 */
#define E_NOTENOUGH (-9)    /* not enough              */
#define E_OVERFLOW  (-10)   /* overflow                */
#define E_ERRDATA   (-11)   /* err data                */
#define E_WAIT      (-15)   /* wait status             */
#define E_PAR       (-33)   /* Parameter error         */

#ifndef NULL
#define NULL ((void*)0)
#endif

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif


#define DIV_FILL_ALIGN(length, size) (((length) + (size)-1) / (size))
#define DIV_CUT_ALIGN(length, size) ((length) / (size))

#define FIFO_NEW(pvFifo, item_size, item_total) uint32_t pvFifo[DIV_FILL_ALIGN(sizeof(FIFO_ENTRY) + item_size * item_total, sizeof(uint32_t))] = {item_size, item_total, 0U, 0U, 0U}
#define FIFO_IS_EMPTY(pvFifo) (((prFIFO_ENTRY)pvFifo)->head == ((prFIFO_ENTRY)pvFifo)->tail)
#define FIFO_IS_FULL(pvFifo) ((((prFIFO_ENTRY)pvFifo)->head == (((prFIFO_ENTRY)pvFifo)->tail + 1U)) || (((((prFIFO_ENTRY)pvFifo)->head + ((prFIFO_ENTRY)pvFifo)->length) - 1U) == (((prFIFO_ENTRY)pvFifo)->tail)))
#define FIFO_GET_TOTAL(pvFifo) ((((prFIFO_ENTRY)pvFifo)->length + ((prFIFO_ENTRY)pvFifo)->tail - ((prFIFO_ENTRY)pvFifo)->head) % ((prFIFO_ENTRY)pvFifo)->length)
#define FIFO_GET_FREE(pvFifo) (((prFIFO_ENTRY)pvFifo)->length - FIFO_GET_TOTAL(pvFifo) - 1U)

typedef struct tagFIFO_ENTRY
{
    const uint32_t size;
    const uint32_t length;
    uint32_t head;
    uint32_t tail;
    volatile uint8_t readBusy;
    volatile uint8_t writeBusy;
    uint8_t buf[2];
} FIFO_ENTRY, *prFIFO_ENTRY;

uint32_t fifo_read(uint8_t* buf, uint32_t* size);
int32_t fifo_write(const uint8_t* buf, uint32_t size);
int32_t fifo_is_full(void);
int32_t fifo_check_item(uint32_t inogesize,
                        uint8_t const* headerbuf,
                        uint32_t checkitemnum,
                        uint32_t itemsize,
                        uint32_t times);
void fifo_clear(void);
void fifo_init(void);
int32_t fifo_is_enough(void);
int32_t fifo_is_remain_data(void);
int32_t fifo_is_usb_full(void);
int32_t fifo_is_bt_full(void);

#endif /* __N32WB452_DATA_FIFO_H__ */
