
/***************************************************************************************
****************************************************************************************
* FILE        : boot_conf.h
* Description :
*
* 此版本软件的远程升级功能使用外部的flash, 因此对flash区进行了如下所示的划分。
*
****************************************************************************************
****************************************************************************************/
#ifndef __BOOT_CONF_H__
#define __BOOT_CONF_H__

//#define IMAGE_HEADER_SIZE               sizeof(image_header_t)
#define FLASH_PAGE_SIZE                 2048

/* OF - on flash ------------------------------------------------------------*/
/* bootloader起始位置 */
#define OF_BOOTLOADER_ADDR              (0x08000000)

/* bootloader区预留空间大小 */
#define OF_BOOTLOADER_SIZE              (0x8000)

/* n32wb452 内部flash大小*/
#define OF_INNER_FLSASH_SIZE            (0x80000)

/* 应用代码起始位置 */
#define OF_FIRMWARE_ADDR                (OF_BOOTLOADER_ADDR+OF_BOOTLOADER_SIZE)
#define OF_FIRMWARE_SIZE                (OF_INNER_FLSASH_SIZE-OF_BOOTLOADER_SIZE)

/* 应用代码启动入口 */
#define OF_FIRMWARE_ENTRY_ADDR          (OF_FIRMWARE_ADDR)

/* 应用代码区校验头区的起始位置 */
//#define OF_SYSINFO_SIZE                 128                 //系统信息占用大小
//#define OF_FIRMWARE_HEADER_ADDR         (SYSINFO_ADDR + OF_SYSINFO_SIZE) //放在系统信息的后面

/* 更新使能     */
//#define OF_UPDATE_EN_ADDR               (OF_FIRMWARE_HEADER_ADDR + IMAGE_HEADER_SIZE)
//#define OF_UPDATE_EN_CLOSE              0xAA
//#define OF_UPDATE_EN_OPEN               0xFF

/* 静默升级标志       */
#define OF_QUIET_FLAG_ADDR              (OF_UPDATE_EN_ADDR + 1)
#define OF_QUIET_FLAG_SET               0xAA
#define OF_QUIET_FLAG_RESET             0xFF

/* SPI FLASH 基地址 */
#define SF_FLASH_BASE_ADDR              (0x000000)

/* backup image */
#define SF_BACKUP_IMAGE_ADDR            (SF_FLASH_BASE_ADDR)
#define SF_BACKUP_IMAGE_SIZE            (OF_FIRMWARE_SIZE)
#define SF_BACKUP_IMAGE_INFO_ADDR       (SF_BACKUP_IMAGE_ADDR+SF_BACKUP_IMAGE_SIZE)
#define SF_BACKUP_IMAGE_INFO_SIZE       (FLASH_PAGE_SIZE)

/* new image */
#define SF_NEW_IMAGE_ADDR               (SF_BACKUP_IMAGE_INFO_ADDR+SF_BACKUP_IMAGE_INFO_SIZE)
#define SF_NEW_IMAGE_SIZE               (OF_FIRMWARE_SIZE)
#define SF_NEW_IMAGE_INFO_ADDR          (SF_NEW_IMAGE_ADDR+SF_NEW_IMAGE_SIZE)
#define SF_NEW_IMAGE_INFO_SIZE          (FLASH_PAGE_SIZE)

#define OF_FIRMWARE_HEADER_ADDR         (SF_NEW_IMAGE_INFO_ADDR + SF_NEW_IMAGE_INFO_SIZE) //放在下载固件区的后面
#define OF_FIRMWARE_HEADER_SIZE         (FLASH_PAGE_SIZE) //放在下载固件区的后面

/*other memory define -----------------------------------------------------*/

#define SF_WRITE_PACK_512               512
#define SF_WRITE_PACK_256               256
#define SF_WRITE_PACK_64                64
#define SF_WRITE_PACK_SIZE              SF_WRITE_PACK_512   //拉取升级包的大小

/* types ---------------------------------------------------------------------*/
/* variable ------------------------------------------------------------------*/


#endif /*__BOOT_CONF_H__*/

