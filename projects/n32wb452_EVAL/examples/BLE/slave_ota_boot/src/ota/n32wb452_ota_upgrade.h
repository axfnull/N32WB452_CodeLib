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
 * @file n32wb452_ble_ota_upgrade.h
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __N32WB452_OTA_UPGRADE_H__
#define __N32WB452_OTA_UPGRADE_H__
#include "n32wb452.h"

#define UPGRADE_IMAGE_FLAG      0xABCDABCD  //升级固件标志
#define CRC16_INITIAL           0xABAB      //CRC16校验初值

#define TYPE_IMAGE_BACKUP   1
#define TYPE_IMAGE_NEW      0

typedef struct image_upgrade_info {
    uint8_t     product_id[8];  /* produce id                   */
    uint8_t     hw_version[4];  /* hardware Version             */
    uint8_t     fw_version[4];  /* Image Version                */
    uint32_t    header_crc;     /* Image Header CRC Checksum    */
    uint32_t    file_size;      /* Image Data Size              */
    uint16_t    image_crc;      /* Image Data CRC Checksum      */
    uint16_t    customer_code;  /* customer type                */
    uint32_t    upgrage_tag;    /* has new image tag            */
//    uint8_t     ih_hardware_info[24]; /* 硬件信息*/
} image_upgrade_info_t;

uint8_t ota_app_system_info_get(image_upgrade_info_t *system_info);
uint8_t ota_app_system_info_update(image_upgrade_info_t *system_info);
uint8_t ota_read_backup_image_info(image_upgrade_info_t *bak_image_info);
uint8_t ota_write_new_image_info(image_upgrade_info_t *new_image_info);
uint8_t ota_read_new_image_info(image_upgrade_info_t *new_image_info);
uint8_t ota_copy_new_image_to_backup_image(void);
uint8_t ota_erase_image_file(uint32_t image_type);
uint8_t ota_write_new_image_file(uint8_t *pdata, uint32_t offset, uint32_t size);
uint8_t ota_image_file_check_crc(image_upgrade_info_t *upgrade_info, uint8_t image_type);
void ota_upgrade_tag_update(void);
void ota_upgrade_tag_clean(void);
uint8_t ota_check_upgrade_tag(image_upgrade_info_t *upgrade_info);
#endif  //__N32WB452_OTA_UPGRADE_H__

