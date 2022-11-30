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
 * @brief 调试及控制输出头文件
 * @file n32wb452_log_level.h
 * @author Nations
 * @version v1.0.0
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef __N32WB452_LOG_LEVEL_H__
#define __N32WB452_LOG_LEVEL_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "n32wb452.h"

#define BLE_NO_LOG      0 // 0 -- 没有日志
#define BLE_DBG         1 // 1 -- debug级别
#define BLE_INFO        2 // 2 -- info级别
#define BLE_WARNING     3 // 3 -- warning级别
#define BLE_ERROR       4 // 4 -- err级别
#define BLE_FAULT       5 // 5 -- fault级别
#define BLE_LOG_LIMIT   6 // 6 -- 边界

#if 0
#define ble_log(x, ...)   
#else
#define ble_log(x, ...)         \
    if (x >= BLE_DBG)           \
    {                           \
        printf(__VA_ARGS__);    \
    }
#endif

#ifdef __cplusplus
}
#endif // defined __cplusplus

#endif //__N32WB452_LOG_LEVEL_H__

