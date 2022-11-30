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
 * @file n32wb452_data_fifo.c
 * @author Nations
 * @version v1.1.0
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 * @date    2020-05-07
 * @brief   文件包含信息FIFO缓冲机制源文件.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "n32wb452.h"
#include "n32wb452_data_fifo.h"
#include "n32wb452_log_level.h"
#include "rwble_config.h"


#define APP_DATA_USB_MIN (64)    /* buf中保留的USB端点存储大小，用于USB下次数据存储.一般不修改 */
#define APP_DATA_BLE_MIN (64)    /* buf中保留的BLE单次存储大小，用于BLE下次数据存储.一般不修改 */

#define APP_DATA_MIN (APP_DATA_USB_MIN + APP_DATA_BLE_MIN)    /* buf中保留的最小存储大小=USB+BLE，用于下次数据存储.一般不修改 */
#define APP_DATA_EXD_SIZE (512 + 160)    /* 可自由扩展内存大小:用户则根据内存剩余空间自由修改 */

//#define APP_DATA_SIZE (APP_DATA_MIN + APP_DATA_EXD_SIZE + 1) /* 数据总缓冲区大小.注意必须要大于128字节，否则因FIFO机制原因导致数据会溢出丢失 */
#define APP_DATA_SIZE (512 + 4) /* 数据总缓冲区大小.注意必须要大于128字节，否则因FIFO机制原因导致数据会溢出丢失 */

#define APP_START_PRINTER_SIZE (480)    /* 开始打印的数据大小:用户则根据内存自由修改 */

FIFO_NEW(APP_BtRcvDataBuffer, 1, APP_DATA_SIZE);

static int32_t fifo_write_item(void* pvFifo, const void* pvdata);
static int32_t fifo_read_item(void* pvFifo, void* pvdata);
static int32_t fifo_clear_all(void* pvFifo);

/************************************************************************
* Function Name : fifo_init
* Description   :初始化fifo空间为sram
* Date      : 2019/08/01
* Parameter :   void
* Return Code   :void
* author Nations
*************************************************************************/
void fifo_init(void)
{
    ble_log(BLE_DEBUG,"total fifo size=%d,start size = %d.\r\n", APP_DATA_SIZE, APP_START_PRINTER_SIZE);
}


/************************************************************************
* Function Name : fifo_write
* Description   :供外部模块写入数据到内部FIFO中
* Date      : 2019/03/13
* Parameter :   buf:数据buf指针
* Parameter :   size 数据大小.
* Return Code   :如内部buf溢出，则丢弃多余数据.
* author Nations
*************************************************************************/
int32_t fifo_write(const uint8_t* buf, uint32_t size)
{
    uint32_t i, overflow = 0;

    if ((!buf) || (!size))
    {
        return 1;
    }

    for (i = 0; i < size; i++)
    {
        /* 将data 写入FIFO中 */
        if (!FIFO_IS_FULL(APP_BtRcvDataBuffer))
        {
            fifo_write_item(APP_BtRcvDataBuffer, buf + i);
        }
        else
        {
            ble_log(BLE_DEBUG,"wr-ferr.\r\n");
            overflow = 1;
            break;
        }
    }

    if (overflow)
    {
        ble_log(BLE_DEBUG,"\r\nCMD&DATA:FIFO OVERFLOW!\r\n");

        /* 如果溢出，数据可能是错乱的，则清除FIFO */
        fifo_clear_all(APP_BtRcvDataBuffer);
        return E_OVERFLOW;
    }
    else
    {
        return E_OK;
    }
}

/************************************************************************
* Function Name : fifo_read
* Description   :供外部模块读取数据到外部BUF中
* Date      : 2019/03/13
* Parameter :   [OUT]buf:读取的buf指针
* Parameter :   [IN/OUT]size 需要读取的数据大小,输入buf大小.
* Return Code   :如不足，则读取所有剩余数据.
* author Nations
*************************************************************************/
uint32_t fifo_read(uint8_t* buf, uint32_t* size)
{
    uint32_t i;
    //uint32_t empty = 0;

    if ((!buf) || (!size))
    {
        return 0;
    }

    for (i = 0; i < *size; i++)
    {
        /* 将data 写入FIFO中 */
        if (!FIFO_IS_EMPTY(APP_BtRcvDataBuffer))
        {
            fifo_read_item(APP_BtRcvDataBuffer, buf + i);
            //log_debuginfo("(%c,%02x) ", *(buf + i), *(buf + i));
            //log_debuginfo("(%02x) ", *(buf + i));
        }
        else
        {
            //empty = 1;
            break;
        }
    }

    return i;
//    if (empty)
//    {
//        //log_debuginfo("CMD&DATA:FIFO is empty!\r\n");
//        return E_NODATA;
//    }
//    else
//    {
//        return E_OK;
//    }
}

/************************************************************************
* Function Name : fifo_is_full
* Description   :判断当前fifo是否已满
* Date      : 2019/07/15
* Parameter :   void
* Return Code   :0:未满；        非0：已满
* author Nations
*************************************************************************/
int32_t fifo_is_full(void)
{
    /* 判断当前FIFO是否满了 */
    if (FIFO_GET_FREE(APP_BtRcvDataBuffer) >= APP_DATA_MIN)/* 大于等于USB端点+一次蓝牙数据大小64 */
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

/************************************************************************
* Function Name : fifo_is_usb_full
* Description   :判断当前fifo是否可以存储USB的一个端点数据量
* Date      : 2019/07/26
* Parameter :   void
* Return Code   :0:未满；        非0：已满
* author Nations
*************************************************************************/
int32_t fifo_is_usb_full(void)
{
    /* 判断当前FIFO是否满了 */
    if (FIFO_GET_FREE(APP_BtRcvDataBuffer) >= APP_DATA_USB_MIN)/* 大于等于USB端点 */
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

/************************************************************************
* Function Name : fifo_is_bt_full
* Description   :判断当前fifo是否可以存储ble的一个单次数据量
* Date      : 2019/07/26
* Parameter :   void
* Return Code   :0:未满；        非0：已满
* author Nations
*************************************************************************/
int32_t fifo_is_bt_full(void)
{
    /* 判断当前FIFO是否满了 */
    if (FIFO_GET_FREE(APP_BtRcvDataBuffer) >= APP_DATA_BLE_MIN)/* 一次蓝牙数据大小64 */
    {
        return 0;
    }
    else
    {
        return 1;
    }
}


/************************************************************************
* Function Name : fifo_is_enough
* Description   :判断当前fifo是否足够数据可以打印
* Date      : 2019/07/24
* Parameter :   void
* Return Code   :0:无足够数据        非0：有足够数据
* author Nations
*************************************************************************/
int32_t fifo_is_enough(void)
{
    /* 判断FIFO是否有足够数据打印 */
    if (FIFO_GET_TOTAL(APP_BtRcvDataBuffer) >= APP_START_PRINTER_SIZE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uint32_t get_fifo_total(void)
{
    return FIFO_GET_TOTAL(APP_BtRcvDataBuffer);
}


uint32_t get_fifo_free(void)
{
    return FIFO_GET_FREE(APP_BtRcvDataBuffer);
}


/************************************************************************
* Function Name : fifo_is_remain_data
* Description   :判断当前fifo是否有残留数据可以打印
* Date      : 2019/08/01
* Parameter :   void
* Return Code   :0:无残留数据        非0：有残留数据
* author Nations
*************************************************************************/
int32_t fifo_is_remain_data(void)
{
    /* 判断FIFO是否有残留数据打印 */
    if (FIFO_GET_TOTAL(APP_BtRcvDataBuffer) > 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


/************************************************************************
* Function Name : fifo_clear_all
* Description   :clear fifo buffer
* Date      : 2019/01/30
* Parameter :   pvFifo: fifo buffer
* Return Code   :error id
* author Nations
*************************************************************************/
static int32_t fifo_clear_all(void* pvFifo)
{
    prFIFO_ENTRY pstFifo = NULL;

    if (pvFifo == NULL)
    {
        return E_PAR;
    }
    pstFifo = (prFIFO_ENTRY)pvFifo;

    if (FIFO_IS_EMPTY(pstFifo))
    {
        return E_OK;
    }

    pstFifo->head      = 0U;
    pstFifo->tail      = 0U;
    pstFifo->readBusy  = 0U;
    pstFifo->writeBusy = 0U;

    return E_OK;
}

/************************************************************************
* Function Name : fifo_clear
* Description   :clear all fifo data
* Date      : 2019/04/28
* Parameter :   void
* Return Code   :void
* author Nations
*************************************************************************/
void fifo_clear(void)
{
    fifo_clear_all(APP_BtRcvDataBuffer);
}

/************************************************************************
* Function Name : fifo_clear_all
* Description   :clear fifo buffer
* Date      : 2019/01/30
* Parameter :   pvFifo: fifo buffer
* Return Code   :error id
* author Nations
*************************************************************************/
static int32_t fifo_read_item(void* pvFifo, void* pvdata)
{
    prFIFO_ENTRY pstFifo = NULL;
        
    if (pvFifo == NULL)
    {
        return E_PAR;
    }
    if (FIFO_IS_EMPTY(pvFifo))
    {
        return E_NODATA;
    }
    pstFifo = (prFIFO_ENTRY)pvFifo;

    if (pstFifo->readBusy == 1U)
    {
        return E_PAR;
    }
    pstFifo->readBusy = 1U;
    memcpy(pvdata, pstFifo->buf + (pstFifo->head * pstFifo->size), pstFifo->size);
    pstFifo->head++;
    if (pstFifo->head >= pstFifo->length)
    {
        pstFifo->head = 0U;
    }

    pstFifo->readBusy = 0U;

    return E_OK;
}

/************************************************************************
* Function Name : fifo_check_item
* Description   :check fifo item
* Date      : 2019/04/29
* Parameter :   pvFifo: fifo buffer
* Return Code   :error id
* author Nations
*************************************************************************/
int32_t fifo_check_item(uint32_t inogesize, uint8_t const* headerbuf, uint32_t checkitemnum, uint32_t itemsize, uint32_t times)
{
    uint32_t i, j;
    uint8_t item[4]   = {0};
    uint32_t err_flag = 1;
    FIFO_ENTRY backup = {.size = 0};

    /* restore info */
    memcpy(&backup, ((prFIFO_ENTRY)APP_BtRcvDataBuffer), sizeof(backup));
    if (inogesize)
    {
        for (j = 0; j < inogesize; j++)
        {
            if (FIFO_IS_EMPTY(APP_BtRcvDataBuffer))
            {
                memcpy(((prFIFO_ENTRY)APP_BtRcvDataBuffer), &backup, sizeof(backup));
                return E_WAIT;
            }
            if (E_OK != fifo_read_item(APP_BtRcvDataBuffer, item))
            {
                memcpy(((prFIFO_ENTRY)APP_BtRcvDataBuffer), &backup, sizeof(backup));
                return E_WAIT;
            }
        }
    }

    for (i = 0; i < times; i++)
    {
        for (j = 0; j < itemsize; j++)
        {
            if (FIFO_IS_EMPTY(APP_BtRcvDataBuffer))
            {
                break;
            }
            if (E_OK != fifo_read_item(APP_BtRcvDataBuffer, item))
            {
                memcpy(((prFIFO_ENTRY)APP_BtRcvDataBuffer), &backup, sizeof(backup));
                return E_WAIT;
            }

            if (j < checkitemnum)
            {
                if (headerbuf[j] != item[0])
                {
                    break;
                }
            }
        }

        if (j < itemsize)
        {
            break;
        }
    }
    if (i >= times)
    {
        err_flag = 0;
    }

    if (err_flag)
    {
        memcpy(((prFIFO_ENTRY)APP_BtRcvDataBuffer), &backup, sizeof(backup));
        return E_WAIT;
    }
    else
    {
        memcpy(((prFIFO_ENTRY)APP_BtRcvDataBuffer), &backup, sizeof(backup));

        return E_OK;
    }
}

/************************************************************************
* Function Name : fifo_clear_all
* Description   :clear fifo buffer
* Date      : 2019/01/30
* Parameter :   pvFifo: fifo buffer
* Return Code   :error id
* author Nations
*************************************************************************/
static int32_t fifo_write_item(void* pvFifo, const void* pvdata)
{
    prFIFO_ENTRY pstFifo = NULL;

    if (pvFifo == NULL)
    {
        return E_PAR;
    }
    if (FIFO_IS_FULL(pvFifo))
    {
        return E_MSGFULL;
    }
    pstFifo = (prFIFO_ENTRY)pvFifo;

    if (pstFifo->writeBusy == 1U)
    {
        return E_PAR;
    }

    pstFifo->writeBusy = 1U;

    memcpy(pstFifo->buf + (pstFifo->tail * pstFifo->size), pvdata, pstFifo->size);
    pstFifo->tail++;
    if (pstFifo->tail >= pstFifo->length)
    {
        pstFifo->tail = 0U;
    }

    pstFifo->writeBusy = 0U;

    return E_OK;
}

