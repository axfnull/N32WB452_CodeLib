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
 * @brief 蓝牙协议处理相关头文件
 * @file n32wb452_ble_protocol.h
 * @author Nations
 * @version v1.0.1
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef __N32WB452_BLE_PROTOCOL_H__
#define __N32WB452_BLE_PROTOCOL_H__
#include "app.h"

#define BLE_PKT_MAX_LEN 20
#define BLE_BUF_LEN    1024
#define BLE_TX_BUF_LEN (BLE_BUF_LEN>>1)

//typedef struct app_env_tag ble_packet_tag;

typedef __packed struct 
{
    uint16_t RxTotalLen;
    uint8_t RxCurrentLen;
    uint8_t RxFinishFlag;
    uint8_t RxBuf[BLE_BUF_LEN];
}ble_packet_tag;

typedef struct
{
    uint8_t packet_len : 5; // bit[4:0] BLE通信帧长度
    uint8_t frame_id : 2;   // bit[6:5] BLE通信帧序号,00->11->00循环
    uint8_t frame_end : 1;  // bit[7] BLE通信帧结束位
} BLE_FRAME_HEAD;

typedef struct
{
    BLE_FRAME_HEAD cfhead; // BLE通信帧头
    uint8_t cfid;          // BLE通信帧顺序号
    uint8_t cfpayload[18]; // BLE通信帧数据,max=18
} BLE_FRAME_DATA;

typedef enum
{
    FRAME_END     = 0,
    FRAME_NOT_END = 1,
} BLE_FRAME_END;
    

typedef struct
{
    uint8_t lock_model[8];
    uint8_t hw_ver[3];
    uint8_t firmware_ver[3];
    uint8_t se_ver[3];
    uint8_t motor_driver_ver[3];
} BLE_VER_FRAME_T;

#define APP_CMD_AUTH          0x00 // 验证命令
#define APP_CMD_ADD_USER      0x01 // 在门锁上添加用户的命令
#define APP_CMD_OPENDOOR      0x02 // 开锁命令
#define APP_CMD_GET_BAT       0x03 // 获取电池电量命令
#define APP_CMD_GET_VERSION   0x04 // 获取版本命令
#define APP_CMD_MODIFY_PIN    0x05 // 修改PIN
#define APP_CMD_ISSUE_USER    0x06 // 下发用户权限
#define APP_CMD_DELETE_USER   0x07 // 删除用户
#define APP_CMD_SET_PARA      0x08 // 设置参数
#define APP_CMD_GET_PARA      0x09 // 获取参数
#define APP_CMD_GET_USER_INFO 0x0A // 获取用户信息
#define APP_CMD_SET_TIME      0x0B // 设置时间
#define APP_CMD_RESTORE_SYS   0x0C // 恢复出厂设置
#define APP_CMD_NETWORK       0x0D // 网络设置
#define APP_CMD_GET_RECORD    0x0E // 获取开锁记录

#define APP_CMD_SEND_NOTICE 0x40 // 发送通知命令

#define APP_EVT_AUTH          0x80 // 验证命令回复
#define APP_EVT_ADD_USER      0x81 // 在门锁上添加用户的命令回复
#define APP_EVT_OPENDOOR      0x82 // 开锁命令回复
#define APP_EVT_GET_BAT       0x83 // 开锁电池电量命令回复
#define APP_EVT_GET_VERSION   0x84 // 开锁版本命令回复
#define APP_EVT_MODIFY_PIN    0x85 // 修改PIN
#define APP_EVT_ISSUE_USER    0x86 // 下发用户权限
#define APP_EVT_DELETE_USER   0x87 // 删除用户
#define APP_EVT_SET_PARA      0x88 // 设置参数
#define APP_EVT_GET_PARA      0x89 // 获取参数
#define APP_EVT_GET_USER_INFO 0x8A // 获取用户信息
#define APP_EVT_SET_TIME      0x8B
#define APP_EVT_RESTORE_SYS   0x8C // 恢复出厂设置
#define APP_EVT_NETWORK       0x8D // 网络设置
#define APP_EVT_GET_RECORD    0x8E // 获取开锁记录


#define APP_EVT_SEND_NOTICE 0xC0 // 发送通知命令的回复


//OTA 命令
#define APP_CMD_NEW_FW_RDY              0x10    //服务器提示有新固件可下载 APP->MCU
#define APP_CMD_NEW_FW_RDY_ACK          0x90       //服务器提示有新固件可下载, MCU应答

#define APP_EVT_UPGRADE_FILE_REQ        0x41    //请求升级文件          MCU->APP
#define APP_EVT_UPGRADE_FILE_RESP       0xC1    //请求升级文件应答          MCU<-APP

#define APP_EVT_UPGRADE_FILE_DONE       0x42    //请求升级文件已完成          MCU->APP
#define APP_EVT_UPGRADE_FILE_DONE_RESP  0xC2    //请求升级文件已完成应答          MCU<-APP




typedef enum
{
    HEAD_ENCRYP = 0xff00,   //加密使用SM4加密，加密op+param部分数据
    HEAD_NOENCRYP = 0xff01, //明文数据
    HEAD_AUTH = 0xff02      //只用于鉴权
}BLE_COMM_HEAD;

typedef enum
{
    BLE_PRIVILEGE_ADMIN = 0,        //与app定义逻辑相反 firmware admin user = 1
    BLE_PRIVILEGE_TEMP  = 1,      //与app定义逻辑相反 firmware normal user = 0

    BLE_PRIVILEGE_NONE = 0xFF,
} BLE_PRIVILEGE_T;

typedef enum
{
    AUTH_SUCCESS = 0,   //成功，通信会话继续
    AUTH_FAIL,          //鉴权失败，门锁蓝牙立即主动断开连接
    AUTH_SUCCESS2,     //鉴权成功，但是需要通知后台解绑用户
}BLE_AUTHENTICATE_RESULT;

typedef enum _BLE_CMD_ADDUSER_STATUS
{
    STA_ADDUSER_SUCCESS = 0,        //添加用户成功
    STA_ADDUSER_PERMISSION_DENIED,  //没有权限
    STA_ADDUSER_DATE_MISSMATCHING,  //有效期与用户不匹配
}BLE_CMD_ADDUSER_STATUS;


typedef enum _BLE_CMD_OPENDOOR_STATUS
{
    STA_OPENDOOR_SUCCESS = 0,   //开锁成功
    STA_OPENDOOR_FAIL,          //开锁失败
    STA_OPENDOORING,            //正在开锁
}BLE_CMD_OPENDOOR_STATUS;

typedef enum _BLE_CMD_GET_BATTERY_STATUS
{
    STA_GET_BATTERY_SUCCESS = 0,   //获取电池电量成功
    STA_GET_BATTERY_FAIL,          //获取电池电量失败
}BLE_CMD_GET_BATTERY_STATUS;

typedef enum _BLE_CMD_BATTERY_LEVEL
{
    BATTERY_LEVEL_0 = 0,    //电池电量 0%
    BATTERY_LEVEL_25,       //电池电量 25%
    BATTERY_LEVEL_50,       //电池电量 50%
    BATTERY_LEVEL_75,       //电池电量 75%
    BATTERY_LEVEL_100,      //电池电量 100%
}BLE_CMD_BATTERY_LEVEL;

//收到APP提示有新固件可下载的状态回复
typedef enum _BLE_CMD_NEW_FW_RDY_ACK
{
    STA_NEW_FW_SUCCESS = 0,     //指令执行成功
    STA_CODE_ERROR,             //鉴别码错误
    STA_CMD_ERROR,              //指令包字段错误
    STA_VERIFIED_ERROR,         //验证失败
    STA_PID_ERROR,              //产品ID不一致
    STA_HW_VERSION_ERROR,       //硬件版本不一致
    STA_FW_VERSION_ERROR,       //固件版本一致
    STA_CUSTOMER_CODE_ERROR,    //客户代码不一致
    STA_UPDATE_DONE             //已下载升级文件
}BLE_CMD_NEW_FW_RDY_STA;

typedef enum _BLE_CMD_GET_FW_ERR_CODE
{
    GET_FW_DONE = 0,            //更新完成
    GET_FW_DATA_ERROR,          //数据块出错(地址/大小错误)
    GET_FW_CRC_ERROR,           //数据块CRC错误
    GET_FW_NIMAGE_FLASH_ERROR,  //新固件FLASH错误
    GET_FW_NEWIMAGE_CRC_ERROR,  //新固件CRC错误
    GET_FW_BAKIMAGE_ERROR       //备份固件错误
}BLE_CMD_GET_FW_ERR_CODE;

typedef enum _BLE_CMD_GEI_FW_DONE_STA
{
    GEI_FW_DONE_SUCCESS = 0,        //指令执行成功
    GEI_FW_DONE_CODE_ERROR,         //鉴别码错误
    GEI_FW_DONE_CMD_ERROR,          //指令包字段错误
    GEI_FW_DONE_CMD_FAIL,           //指令执行失败
    GEI_FW_DONE_PID_ERROR,          //产品ID不一致
    GEI_FW_DONE_HW_VERSION_ERROR,   //硬件版本不一致
    GEI_FW_DONE_FW_VERSION_ERROR,   //固件版本不一致
    GEI_FW_DONE_CUSTOMER_CODE_ERROR,//客户代码不一致
    GEI_FW_DONE_FILE_CRC_ERROR,     //升级固件CRC错误
    GEI_FW_DONE_FILE_SIZE_ERROR,    //升级固件大小错误
}BLE_CMD_GEI_FW_DONE_STA;

typedef __packed struct  _ble_comm_authenticate_t            //ble鉴权
{
    uint8_t user_privilege;
    uint8_t user_auth_data[20];
    uint8_t encryp_factor[8];
}ble_comm_authenticate_t;

typedef __packed struct  _ble_comm_authenticate_ack_t            //ble鉴权结果回复
{
    uint8_t op_result;
    uint8_t encryp_factor[8];
}ble_comm_authenticate_ack_t;

typedef __packed struct  _ble_comm_add_user_t //ble添加用户
{
    uint8_t user_type;
    uint8_t time_valid[15];
}ble_comm_add_user_t;

typedef __packed struct  _ble_comm_add_user_ack_t //ble添加用户结果回复
{
    uint8_t op_result;
}ble_comm_add_user_ack_t;

typedef __packed struct  _ble_comm_opendoor_ack_t //ble开门结果回复
{
    uint8_t op_result;
}ble_comm_opendoor_ack_t;

typedef __packed struct  _ble_comm_get_battery_ack_t //ble获取电池电量结果回复
{
    uint8_t op_result;
    uint8_t batt_value_level;
    uint8_t batt_value_percent;         //0-100代表0~100%, 0xff表示无效
}ble_comm_get_battery_ack_t;

typedef __packed struct  _ble_comm_get_version_ack_t //ble获取版本
{
    uint8_t op_result;
    uint8_t lock_model[8];  //门锁型号
    uint8_t hw_version[3];  //硬件版本号
    uint8_t sw_version[3];  //MCU固件版本号
    uint8_t se_version[3];  //SE固件版本号
    uint8_t reserved0[3];
    uint8_t sn[24];         //设备序列号
    uint8_t se_sn[16];      //se序列号
    uint8_t fp_vendor;      //指纹厂商
    uint8_t fp_model[8];    //指纹模块
    uint8_t nb_pid[4];      //NB模块ID
    uint8_t imei[15];
    uint8_t imsi[15];
//    uint8_t hw_info[24];    //硬件信息
}ble_comm_get_version_ack_t;

typedef __packed struct  _ble_comm_set_systime_t //ble设置系统时间
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t week;
}ble_comm_set_systime_t;

typedef __packed struct  _ble_comm_set_systime_ack_t //ble设置系统时间结果回复
{
    uint8_t op_result;
}ble_comm_set_systime_ack_t;

typedef __packed struct  _ble_comm_set_param_t //ble设置系统参数
{
    uint8_t volume;         //音量,0:高音量,1:中音量,2:低音量,3:静音
    uint8_t language;       //语言,0:中文,1:英文
    uint8_t lockmode;       //开锁模式,0:单一开锁,1:组合开锁
    uint8_t alarm_en;       //防撬报警开关,0:关闭,1:打开,0xff:不支持
    uint8_t dynamic_pwd_en; //动态密码开关,0:关闭,1:打开
    uint8_t open_dir;       //开门方向,0:左开,1:右开,0xff:不支持
    uint8_t bluetooth_en;   //蓝牙开关,0:打开,1:关闭,0xff:不支持
    uint8_t network_en;     //网络开关,0:打开,1:关闭,0xff:不支持
    uint8_t open_time;      //开锁间隔:1~99s,0xff:不支持

}ble_comm_set_param_t;

typedef __packed struct  _ble_comm_set_param_ack_t //ble设置系统参数结果回复
{
    uint8_t op_result;      
    uint8_t volume;         //音量,0:高音量,1:中音量,2:低音量,3:静音
    uint8_t language;       //语言,0:中文,1:英文
    uint8_t lockmode;       //开锁模式,0:单一开锁,1:组合开锁
    uint8_t alarm_en;       //防撬报警开关,0:关闭,1:打开,0xff:不支持
    uint8_t dynamic_pwd_en; //动态密码开关,0:关闭,1:打开
    uint8_t open_dir;       //开门方向,0:左开,1:右开,0xff:不支持
    uint8_t bluetooth_en;   //蓝牙开关,0:打开,1:关闭,0xff:不支持
    uint8_t network_en;     //网络开关,0:打开,1:关闭,0xff:不支持
    uint8_t open_time;      //开锁间隔:1~99s,0xff:不支持

}ble_comm_set_param_ack_t;


typedef __packed struct  _ble_comm_get_param_t //ble获取系统参数
{
    uint8_t op_result;      
    uint8_t battery;        //电池电压
    uint8_t volume;         //音量,0:高音量,1:中音量,2:低音量,3:静音
    uint8_t language;       //语言,0:中文,1:英文
    uint8_t lockmode;       //开锁模式,0:单一开锁,1:组合开锁
    uint8_t alarm_en;       //防撬报警开关,0:关闭,1:打开,0xff:不支持
    uint8_t dynamic_pwd_en; //动态密码开关,0:关闭,1:打开
    uint8_t open_dir;       //开门方向,0:左开,1:右开,0xff:不支持
    uint8_t bluetooth_en;   //蓝牙开关,0:打开,1:关闭,0xff:不支持
    uint8_t network_en;     //网络开关,0:打开,1:关闭,0xff:不支持
    uint8_t open_time;      //开锁间隔:1~99s,0xff:不支持
}ble_comm_get_param_t;

typedef __packed struct  _ble_comm_new_fw_rdy_t //app提示有新固件可下载
{
    uint8_t product_id[8];  //产品ID, "12345678"
    uint8_t hw_version[4];  //硬件版本, "V100"
    uint8_t fw_version[4];  //软件版本, "V100"
    uint32_t file_size;     //可下载固件大小
    uint16_t image_crc;     //固件CRC校验码
    uint16_t customer_code; //客户代码
}ble_comm_new_fw_rdy_t;

typedef __packed struct  _ble_comm_new_fw_rdy_ack_t //mcu收到有新固件可下载的回复
{
    uint8_t op_result;      
}ble_comm_new_fw_rdy_ack_t;

typedef __packed struct  _ble_comm_get_upgrade_file_t //ble向app获取更新固件文件
{
    uint8_t product_id[8];  //产品ID, "12345678"
    uint8_t hw_version[4];  //硬件版本, "V100"
    uint8_t fw_version[4];  //软件版本, "V100"
    uint32_t file_size;     //可下载固件大小
    uint16_t image_crc;     //固件CRC校验码
    uint16_t customer_code; //客户代码
    uint32_t data_addr;     //升级文件数据块地址
    uint16_t data_size;     //升级文件数据块长度
}ble_comm_get_upgrade_file_t;

typedef __packed struct  _ble_comm_upgrade_data_t //app下发固件文件数据块
{
    uint32_t data_addr;     //升级文件数据块地址
    uint16_t data_size;     //升级文件数据块长度
    //uint8_t *data_image;    //升级文件数据块数据, 可变长度
    //uint16_t checksum_crc16; //升级文件数据块数据CRC校验码,data_image数据最后2Byte
}ble_comm_upgrade_data_t;

typedef __packed struct  _ble_comm_get_upgrade_file_done_t //mcu通知app下载固件文件完成
{
    uint8_t product_id[8];  //产品ID, "12345678"
    uint8_t hw_version[4];  //硬件版本, "V100"
    uint8_t fw_version[4];  //软件版本, "V100"
    uint32_t file_size;     //可下载固件大小
    uint16_t image_crc;     //固件CRC校验码
    uint16_t customer_code; //客户代码
    uint8_t err_code;       //异常码
}ble_comm_get_upgrade_file_done_t;

typedef __packed struct  _ble_comm_get_upgrade_file_done_ack_t //mcu通知app下载固件文件完成
{
    uint8_t product_id[8];  //产品ID, "12345678"
    uint8_t hw_version[4];  //硬件版本, "V100"
    uint8_t fw_version[4];  //软件版本, "V100"
    uint32_t file_size;     //可下载固件大小
    uint16_t image_crc;     //固件CRC校验码
    uint16_t customer_code; //客户代码
    uint8_t err_code;       //异常码
}ble_comm_get_upgrade_file_done_ack_t;

typedef __packed struct _ble_comm_header_t
{
    uint16_t frame_head;
    uint16_t frame_len;
    uint16_t frame_len_invert;
    uint8_t option;
}ble_comm_header_t;

typedef __packed union _ble_payload_t
{
    ble_comm_authenticate_t ble_comm_authenticate;
    ble_comm_authenticate_ack_t ble_comm_authenticate_ack;
    ble_comm_add_user_t ble_comm_add_user;
    ble_comm_add_user_ack_t ble_comm_add_user_ack;
    ble_comm_opendoor_ack_t ble_comm_opendoor_ack;
    ble_comm_get_battery_ack_t ble_comm_get_battery_ack;
    ble_comm_get_version_ack_t ble_comm_get_version_ack;    
    ble_comm_set_systime_t ble_comm_set_systime;
    ble_comm_set_systime_ack_t ble_comm_set_systime_ack;
    ble_comm_set_param_t ble_comm_set_param;
    ble_comm_set_param_ack_t ble_comm_set_param_ack;
    ble_comm_get_param_t ble_comm_get_param;
    ble_comm_new_fw_rdy_t ble_comm_new_fw_rdy;
    ble_comm_get_upgrade_file_t ble_comm_get_upgrade_file;
    ble_comm_upgrade_data_t ble_comm_upgrade_data;
    ble_comm_get_upgrade_file_done_t ble_comm_get_upgrade_file_done;
    ble_comm_get_upgrade_file_done_ack_t ble_comm_get_upgrade_file_done_ack;
}ble_payload_t;

typedef __packed struct  _ble_comm_frame_t
{
    ble_comm_header_t ble_header;
    ble_payload_t ble_payload;
    //uint16_t chksum_crc16;
}ble_comm_frame_t;


void ble_send_packet(uint16_t* tx_len, uint8_t* pdata);
uint8_t ble_communication_parse(uint8_t* in_data, uint16_t in_len, uint8_t* out_data, uint16_t* out_len);
void ble_packet_parse(uint8_t const* in_data, uint8_t const in_len, uint8_t* out_data);
void ble_nodified_host_process(uint8_t option, uint8_t status, uint8_t *out_data, uint16_t *out_len);
void data_to_smallmode(uint8_t *data, uint8_t len);

#endif //__N32WB452_BLE_PROTOCOL_H__

