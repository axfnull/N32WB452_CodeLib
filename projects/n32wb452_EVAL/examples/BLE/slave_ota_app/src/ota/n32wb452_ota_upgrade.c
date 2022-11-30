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
 * @file n32wb452_ble_ota_upgrade.c
 * @author Nations
 * @version v1.0.1
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include "string.h"
#include "n32wb452_ota_conf.h"
#include "n32wb452_w25qxx.h"
#include "n32wb452_ble_ota_api.h"
#include "n32wb452_ota_upgrade.h"
#include "crc16.h"
#include "log.h"

uint8_t ota_app_system_info_get(image_upgrade_info_t *system_info)
{
    W25QXX_Read((uint8_t *)system_info, OF_FIRMWARE_HEADER_ADDR, sizeof(image_upgrade_info_t));
    return 1;
}

uint8_t ota_app_system_info_update(image_upgrade_info_t *system_info)
{
    //W25QXX_Erase_Sector(OF_FIRMWARE_HEADER_ADDR);
    W25QXX_Write((uint8_t *)system_info, OF_FIRMWARE_HEADER_ADDR, sizeof(image_upgrade_info_t));
    return 1;
}

uint8_t ota_write_backup_image_info(image_upgrade_info_t *bak_image_info)
{
    //W25QXX_Erase_Sector(SF_BACKUP_IMAGE_INFO_ADDR);
    W25QXX_Write((uint8_t *)bak_image_info, SF_BACKUP_IMAGE_INFO_ADDR, sizeof(image_upgrade_info_t));
    return 1;
}

uint8_t ota_read_backup_image_info(image_upgrade_info_t *bak_image_info)
{
    W25QXX_Read((uint8_t *)bak_image_info, SF_BACKUP_IMAGE_INFO_ADDR, sizeof(image_upgrade_info_t));
    return 1;
}


//把固件信息数据写入下载固件信息区
uint8_t ota_write_new_image_info(image_upgrade_info_t *new_image_info)
{
    //W25QXX_Erase_Sector(SF_NEW_IMAGE_INFO_ADDR);
    W25QXX_Write((uint8_t *)new_image_info, SF_NEW_IMAGE_INFO_ADDR, sizeof(image_upgrade_info_t));
    return 1;
}

//读取固件信息
uint8_t ota_read_new_image_info(image_upgrade_info_t *new_image_info)
{
    W25QXX_Read((uint8_t *)new_image_info, SF_NEW_IMAGE_INFO_ADDR, sizeof(image_upgrade_info_t));
    return 1;
}


//把下载区的固件搬移至备份区
uint8_t ota_copy_new_image_to_backup_image(void)
{
    uint8_t read_buf[SF_WRITE_PACK_SIZE];
    uint8_t check_buf[SF_WRITE_PACK_SIZE];
    uint32_t w_page = 0;
    uint32_t w_addr = SF_BACKUP_IMAGE_ADDR;
    uint32_t r_addr = SF_NEW_IMAGE_ADDR;
    uint8_t times = 0;
    uint32_t total_page = SF_NEW_IMAGE_SIZE/SF_WRITE_PACK_SIZE+1;  //image page +image_info page

    while(w_page < total_page)
    {
        //擦除当前页
        //W25QXX_Erase_Sector(w_addr);
        //读出下载区固件
        W25QXX_Read(read_buf, r_addr, SF_WRITE_PACK_SIZE);

        //写入当前页
        W25QXX_Write(read_buf, w_addr, SF_WRITE_PACK_SIZE);
        W25QXX_Read(check_buf, w_addr, SF_WRITE_PACK_SIZE); 
        //如果本次写不成功, 重新读写
        if (memcmp(read_buf, check_buf, SF_WRITE_PACK_SIZE) != 0)
        {
            log_debug("err write backup image!\r\n");
            //todo:重复3次, 退出搬移
            if (times++ >= 3)
            {
                return 0;
            }
            continue;
        }
//        log_debug("[%d] w %x, r %x, pg %d!\r\n", total_page, w_addr, r_addr, w_page);
        r_addr += SF_WRITE_PACK_SIZE;
        w_addr += SF_WRITE_PACK_SIZE;
        w_page++;
    }

    return 1;
}

//擦除整个下载固件区域
uint8_t ota_erase_image_file(uint32_t image_type)
{
    uint32_t e_addr = SF_NEW_IMAGE_ADDR;
    uint32_t e_size = 0;
    uint32_t total_size = SF_NEW_IMAGE_SIZE+FLASH_PAGE_SIZE;

    if (image_type == TYPE_IMAGE_BACKUP)
    {
        e_addr = SF_BACKUP_IMAGE_ADDR;
        total_size = SF_BACKUP_IMAGE_SIZE+FLASH_PAGE_SIZE;
    }

    while(e_size < total_size)
    {
        W25QXX_Erase_Sector(e_addr);
        e_addr += FLASH_PAGE_SIZE;
        e_size += FLASH_PAGE_SIZE;
    }
    return 1;
}

//把下载固件数据写入下载固件区域
uint8_t ota_write_new_image_file(uint8_t *pdata, uint32_t offset, uint32_t size)
{
    uint32_t sector_addr = offset + SF_NEW_IMAGE_ADDR;  

    //检查地址合法性, 非法地址不写入flash下载固件区域.
    if ((sector_addr < SF_NEW_IMAGE_ADDR) && (sector_addr > SF_NEW_IMAGE_ADDR + SF_NEW_IMAGE_SIZE))
    {
        return 0;
    }

    //每页大小为2K, 每2K擦除一页
    //if (sector_addr % FLASH_PAGE_SIZE == 0)
    //    W25QXX_Erase_Sector(sector_addr);
    W25QXX_Write(pdata, sector_addr, size);
//    log_debug("s_ad %x, of %x\r\n", sector_addr, offset);
    return 1;
}

//对下载固件固件进行crc校验
uint8_t ota_image_file_check_crc(image_upgrade_info_t *upgrade_info, uint8_t image_type)
{
    uint32_t offset = 0;
    uint32_t offset_image = 0;
    uint32_t file_size = upgrade_info->file_size;
    uint8_t check_data[SF_WRITE_PACK_SIZE];
    uint16_t check_crc = 0;
    uint32_t read_size = 0;
    uint8_t ret = 0;

    if (TYPE_IMAGE_NEW == image_type)
    {
        offset_image = SF_NEW_IMAGE_ADDR;
    }
    else if (TYPE_IMAGE_BACKUP == image_type)
    {
        offset_image = SF_BACKUP_IMAGE_ADDR;
    }

    for (; offset < file_size; )
    {
        read_size = SF_WRITE_PACK_SIZE;
        if (file_size - offset < SF_WRITE_PACK_SIZE)
            read_size = file_size - offset;
        W25QXX_Read(check_data, offset + offset_image, read_size);
        check_crc = CRC16(check_crc, check_data, read_size);
        offset += read_size;
    }
    log_debug("c_c %04x, c_i %04x %x, %x\r\n", check_crc, upgrade_info->image_crc,offset, file_size);
    if (check_crc == upgrade_info->image_crc)
        ret = 1;

    return ret;
}


//更新升级标志
void ota_upgrade_tag_update(void)
{
    image_upgrade_info_t upgrade_info;
    ota_read_backup_image_info(&upgrade_info);
    upgrade_info.upgrage_tag = UPGRADE_IMAGE_FLAG;
    upgrade_info.header_crc = CRC16_INITIAL;
    upgrade_info.header_crc = CRC16(0, (uint8_t *)&upgrade_info, sizeof(image_upgrade_info_t));
    ota_write_backup_image_info(&upgrade_info);
}

//清除升级标志
void ota_upgrade_tag_clean(void)
{
    image_upgrade_info_t upgrade_info;
    ota_read_backup_image_info(&upgrade_info);
    upgrade_info.upgrage_tag = (~UPGRADE_IMAGE_FLAG);
    upgrade_info.header_crc = CRC16_INITIAL;
    upgrade_info.header_crc = CRC16(0, (uint8_t *)&upgrade_info, sizeof(image_upgrade_info_t));
    ota_write_backup_image_info(&upgrade_info);
}

//检查升级标志
uint8_t ota_check_upgrade_tag(image_upgrade_info_t *upgrade_info)
{
    uint16_t info_crc, info_crc_in;
    
    ota_read_backup_image_info(upgrade_info);
    if (upgrade_info->upgrage_tag == UPGRADE_IMAGE_FLAG)
    {
        info_crc_in = upgrade_info->header_crc;
        upgrade_info->header_crc = CRC16_INITIAL;
        info_crc = CRC16(0, (uint8_t *)upgrade_info, sizeof(image_upgrade_info_t));
        if (info_crc_in == info_crc)
        {
            log_debug("upgrade tag ok\r\n");
            return 1;
        }
    }
    return 0;
}


