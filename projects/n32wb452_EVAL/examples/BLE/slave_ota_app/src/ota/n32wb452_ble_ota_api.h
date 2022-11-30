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
 * @file n32wb452_ble_ota_api.h
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __N32WB452_BLE_OTA_API_H__
#define __N32WB452_BLE_OTA_API_H__

#include "n32wb452_ota_upgrade.h"

#define PRODUCE_ID  "TZ3HBN1"  //产品ID
#define HW_VERSION  "100"       //硬件版本号
#define FW_VERSION  "101"       //软件版本号
#define CUSTOMER_CODE 14    //客户ID
#define PRODUCE_SN  "AM1001D001001219"

//
typedef enum _ble_ota_status_t
{
    OTA_STA_IDLE = 0,
    OTA_STA_GET_FILE_INFO,      //下载新固件
    OTA_STA_GET_FILE,           //下载新固件
    OTA_STA_ENTER_UPGRADE,   //把新固件更新至备份区
    
}ble_ota_status_t;

typedef enum _ble_ota_download_status_t
{
    FW_DOWNLOAD_NONE = 0,   //无下载固件
    FW_DOWNLOAD_ING,        //正在下载新固件
}ble_ota_download_status_t;

typedef __packed struct _ble_ota_param
{
    uint8_t product_id[8];  //产品ID,如:      "12345678"
    uint8_t hw_version[4];  //硬件版本,如: "V100"
    uint8_t fw_version[4];  //软件版本,如: "V100"
    uint32_t file_size;     //可下载固件大小
    uint16_t image_crc;     //固件CRC校验码
    uint16_t customer_code; //客户代码
    uint32_t current_data_addr; //当前下载升级文件地址
    uint16_t current_data_size; //
    uint8_t upgrade_status;     //下载升级文件状态机
    uint8_t upgrade_flag;       //是否已经在下载升级文件,断点续传
    uint8_t err_code;       //升级过程的错误代码
}ble_ota_param_t;


extern ble_ota_param_t ble_ota_param;


void ble_ota_init(void);
uint8_t ble_ota_image_info_check(image_upgrade_info_t *pota_image_info);
void ble_ota_get_current_upfrade_file_info(uint8_t *upgrade_info);
uint8_t ble_ota_upgrade_info(void);
void ble_ota_process(void);
uint8_t ble_ota_system_info_check(image_upgrade_info_t *system_info);
uint8_t  ble_ota_upgrade_info_update(ble_ota_param_t *pble_ota_param);
uint8_t ble_ota_download_upgrade_file_done(void);
uint8_t ble_ota_get_err_code(void);
void ble_ota_upgrade_fw_process(uint8_t *upgrade_data);
uint8_t ble_ota_upgrade_info_check(ble_ota_param_t *pble_ota_param);

#endif //__N32WB452_BLE_OTA_API_H__


