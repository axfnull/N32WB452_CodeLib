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
 * @file lcd_display.h
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef __LCD_DISPLAY_H__
#define __LCD_DISPLAY_H__

#include "lcd_drv.h"
#include "gsensor.h"

#define FONT_SIZE  (24)
#define FONT_WIDTH (17)
#define DISP_WIDTH (LCD_WIDTH / FONT_WIDTH)
#define DISP_GSENSEOR_WIDTH   6     //must be less than DISP_WIDTH

#define LINE1  (FONT_SIZE * 0)
#define LINE2  (FONT_SIZE * 1)
#define LINE3  (FONT_SIZE * 2)
#define LINE4  (FONT_SIZE * 3)
#define LINE5  (FONT_SIZE * 4)
#define LINE6  (FONT_SIZE * 5)
#define LINE7  (FONT_SIZE * 6)
#define LINE8  (FONT_SIZE * 7)
#define LINE9  (FONT_SIZE * 8)
#define LINE10 (FONT_SIZE * 9)

typedef enum
{
    DISP_INIT               = 0x0000001,
    DISP_BT_CONNECT         = 0x0000002,
    DISP_BT_DISCONNECT      = 0x0000004,
    DISP_BT_RECDATA         = 0x0000008,
    DISP_GSENSOR_UPDATE     = 0x0000010,
    DISP_ADC_UPDATE         = 0x0000020,
    DISP_TEMP_HUMID_UPDATE  = 0x0000040,
    DISP_IDLE = 0
} display_state;

typedef struct
{
    char dispdata_init[2][DISP_WIDTH];
    char dispdata_gsensor[DISP_GSENSEOR_WIDTH*3];
    char dispdata_adc[DISP_WIDTH];
    char dispdata_bt[2][DISP_WIDTH];
    char dispdata_temp_humid[DISP_WIDTH];
} display_data_t;

typedef __packed struct
{
    uint8_t disp_init_pos;
    uint8_t disp_gsensor_pos;
    uint8_t disp_adc_pos;
    uint8_t disp_bt_pos;
    uint8_t disp_temp_humid_pos;
}display_refresh_typedef;

void display_init(void);
void display_deinit(void);
void display_clear(void);
void display_clear_line(uint16_t line_num);
void display_refresh(void);
void display_refresh_line(uint16_t line_num);
void display_poweroff(void);
void display_poweron(void);
void display_bt_disconnect(void);
void display_bt_connect(void);
void display_bt_receive_data(char* data);
uint32_t display_update(uint32_t state, uint8_t* data, uint16_t len);
void display_handle(uint32_t * update_line);

#endif //__LCD_DISPLAY_H__
