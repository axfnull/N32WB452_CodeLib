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
 * @brief N32WB452的ble蓝牙通信协议文件
 * @file n32wb452_ble_protocol.c
 * @author Nations
 * @version v1.0.1
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
/* Scheduler includes. */
#include <stdlib.h>
#include <string.h>
#include "sys_type.h"
#include "n32wb452_ble_api.h"
#include "n32wb452_ble_protocol.h"
#include "ble_encrypt_interface.h"
#include "user.h"
#include "sha1.h"
#include "CRC16.h"
#include "n32wb452_ble_ota_api.h"
#include "log.h"

static vu8 s_cBlePrivilege = BLE_PRIVILEGE_NONE; // 0 为管理员权限，其他为临时权限。 临时权限只能添加临时用户，无法使用其他指令
    
void data_to_bigmode(uint8_t *data, uint8_t len)
{
    uint8_t tmp_data[64];
    uint8_t i = 0;

    for (i = 0; i < len; i++)
    {
        tmp_data[i] = data[len - i - 1];
    }
    memcpy(data, tmp_data, len);
}

void data_to_smallmode(uint8_t *data, uint8_t len)
{
    uint8_t tmp_data[64];
    uint8_t i = 0;

    for (i = 0; i < len; i++)
    {
        tmp_data[len - i - 1] = data[i];
    }
    memcpy(data, tmp_data, len);
}

uint8_t Bcd2ToByte(uint8_t Value)
{
    uint8_t tmp = 0;
    tmp         = ((uint8_t)(Value & (uint8_t)0xF0) >> (uint8_t)0x4) * 10;
    return (tmp + (Value & (uint8_t)0x0F));
}


void ble_send_packet(uint16_t* tx_len, uint8_t* pdata)
{
    uint16_t len = *tx_len;
    BLE_FRAME_DATA ble_tx_frame;
    uint8_t pkt_len;
    static uint8_t tx_frame_num = 0;
    static uint8_t tx_cnt;
    static uint8_t tx_curr_pos = 0;

    if (tx_frame_num == 0)
    {
        tx_curr_pos  = 0;
        tx_cnt       = 0;
        tx_frame_num = len / 18;

        if (len % 18 != 0)
        {
            tx_frame_num += 1;
        }
    }

    if (tx_frame_num == 1)
    {
        ble_tx_frame.cfhead.frame_end = FRAME_END;
        pkt_len                       = len;
    }
    else
    {
        ble_tx_frame.cfhead.frame_end = FRAME_NOT_END;
        pkt_len                       = BLE_PKT_MAX_LEN - 2;
    }
    ble_tx_frame.cfhead.frame_id   = tx_cnt % 4;
    ble_tx_frame.cfid              = tx_cnt;
    ble_tx_frame.cfhead.packet_len = pkt_len + 2;
    memcpy(ble_tx_frame.cfpayload, &pdata[tx_curr_pos], pkt_len);

    bt_snd_data((uint8_t*)&ble_tx_frame, (uint8_t)ble_tx_frame.cfhead.packet_len, USER_IDX_READ_NOTIFY_VAL);
    tx_frame_num -= 1;
    tx_cnt += 1;
    *tx_len = len - pkt_len;
    tx_curr_pos += pkt_len;
}

//用户鉴权
uint8_t ble_host_authenticate(uint8_t* in_data, uint8_t* out_data, uint16_t* out_len)
{
    uint8_t rtv;
    BLE_ENCRYPT_AUTH_DATA_T auth_data;
    uint8_t i;
    uint8_t admin_pwd[20] = {0x31, 0xD3, 0x49, 0x2F, 0x3D, 0x96, 0x2D, 0xA7};//{1, 2, 3, 4, 5, 6, 7, 8};
    uint8_t admin_pwd_len = 8;
    uint8_t random[8] ;//= ;{0x31, 0xD3, 0x49, 0x2F, 0x3D, 0x96, 0x2D, 0xA7}; //31D3492F3D962DA7
//    int32_t ret;
    uint16_t admin_num = 0, total_num = 0;
//    flash_pswd_item admin_pswd = {0};
    ble_comm_frame_t *ble_frame = (ble_comm_frame_t *)in_data;
    ble_comm_authenticate_t *ble_auth = (ble_comm_authenticate_t *)&ble_frame->ble_payload;
    ble_comm_frame_t *ble_frame_out = (ble_comm_frame_t *)out_data;
    ble_comm_authenticate_ack_t *ble_auth_ack = (ble_comm_authenticate_ack_t *)&ble_frame_out->ble_payload;

    rtv             = FALSE;
    s_cBlePrivilege = ble_auth->user_privilege;
    memcpy(auth_data.ble_auth, ble_auth->user_auth_data, sizeof(ble_auth->user_auth_data));
    memcpy(auth_data.ble_rnd, ble_auth->encryp_factor, sizeof(ble_auth->encryp_factor));

    total_num = 0;////////
    admin_num = 0;////////

    //flash_pswd_query_admin_user_count(&total_num, &admin_num, &user_num);
    // 初始化密码状态，使用默认密码鉴权
    if (total_num == 0 && (admin_num == 0))
    {
        rtv = ble_encrypt_authenticate_to_get_session_key(&auth_data, admin_pwd, admin_pwd_len, random);
    }
    //使用当前的管理员密码鉴权
    else
    {
        for (i = 0; i < admin_num; i++)
        {
            // 获取ADIMIN PWD
            //获取一个管理员密码
//            pswd_buf_len = sizeof(admin_pswd.data);
            //if (SYS_RET_SUCCESS != flash_pswd_get_admin_value(i, (uint16_t*)&pswd_buf_len, admin_pswd.data))
            {
            //    break;
            }

//            memset(admin_pwd, 0, sizeof(admin_pwd));
//            admin_pwd_len = pswd_buf_len;

            //ascii转换为hex
//            for (uint8_t i = 0; i < admin_pwd_len; i++)
            {
//                admin_pwd[i] = admin_pswd.data[i] - 0x30;
            }
            //memcpy(admin_pwd, (uint8_t*)admin_pswd.data, admin_pwd_len);
            rtv = ble_encrypt_authenticate_to_get_session_key(&auth_data, admin_pwd, admin_pwd_len, random);
            if (rtv == TRUE)
            {
                break;
            }
        }
    }

    ble_frame_out->ble_header.option = APP_EVT_AUTH;
    rtv = TRUE;

    if (rtv == TRUE)
    {
        // sys_log(TSK_BLE, "APP_CMD_AUTH ok!\r\n");
        ble_auth_ack->op_result = AUTH_SUCCESS; // 状态码
    }
    else // 失败，应该主动断开蓝牙
    {
        // sys_log(TSK_BLE,"APP_CMD_AUTH false!\r\n");
        ble_auth_ack->op_result = AUTH_FAIL; // 状态码
    }
    memcpy(ble_auth_ack->encryp_factor, random, 8); // 随机数

    *out_len = sizeof(ble_comm_authenticate_ack_t) + 1;

    return rtv;
}

void ble_get_battery_info(uint8_t* out_data, uint16_t* out_len)
{
    ble_comm_frame_t *ble_frame_out = (ble_comm_frame_t *)out_data;
    ble_comm_get_battery_ack_t *ble_get_batt_ack = (ble_comm_get_battery_ack_t *)&ble_frame_out->ble_payload;

    ble_frame_out->ble_header.option = APP_EVT_GET_BAT;
    ble_get_batt_ack->op_result = STA_GET_BATTERY_SUCCESS;  // 状态码
    ble_get_batt_ack->batt_value_level = 3;  // g_tBatteryValue.battery_level;
    ble_get_batt_ack->batt_value_percent = 80; // g_tBatteryValue.battery_percent;
    *out_len    = 4;
}

void ble_get_version(uint8_t* out_data, uint16_t* out_len)
{
    ble_comm_frame_t *ble_frame_out = (ble_comm_frame_t *)out_data;
    ble_comm_get_version_ack_t *ble_get_version_ack = (ble_comm_get_version_ack_t *)&ble_frame_out->ble_payload;

    memset((uint8_t *)ble_get_version_ack, 0, sizeof(ble_comm_get_version_ack_t));
    
    ble_frame_out->ble_header.option = APP_EVT_GET_VERSION;
    ble_get_version_ack->op_result = 0; // 状态码
    memcpy(ble_get_version_ack->lock_model, PRODUCE_ID, MIN(sizeof(ble_get_version_ack->lock_model), strlen(PRODUCE_ID)));
    memcpy(ble_get_version_ack->hw_version, HW_VERSION, MIN(sizeof(ble_get_version_ack->hw_version), strlen(HW_VERSION)));
    memcpy(ble_get_version_ack->sw_version, FW_VERSION, MIN(sizeof(ble_get_version_ack->sw_version), strlen(FW_VERSION)));
    memcpy(ble_get_version_ack->se_version, "xxx", MIN(sizeof(ble_get_version_ack->se_version), strlen("xxx")));
    memset(ble_get_version_ack->reserved0, 0xFF, sizeof(ble_get_version_ack->reserved0));
    memcpy(ble_get_version_ack->sn, PRODUCE_SN, MIN(sizeof(ble_get_version_ack->sn), sizeof(PRODUCE_SN)));
    memcpy(ble_get_version_ack->se_sn, "N32WB45298765432", MIN(sizeof(ble_get_version_ack->se_sn), sizeof("N32WB45298765432")));
    ble_get_version_ack->fp_vendor = 2;
    memcpy(ble_get_version_ack->fp_model, "ZFMS-39", sizeof(ble_get_version_ack->fp_model));

    *out_len = sizeof(ble_comm_get_version_ack_t) + 1;
}


//app提示有新固件可下载
void ble_new_fw_ready_notice(uint8_t* in_data, uint8_t* out_data, uint16_t* out_len)
{
//    BLE_CMD_NEW_FW_RDY_STA err_code = STA_NEW_FW_SUCCESS;
    ble_comm_frame_t *ble_frame = (ble_comm_frame_t *)in_data;
    ble_comm_new_fw_rdy_t *ble_new_fw_rdy = (ble_comm_new_fw_rdy_t *)&ble_frame->ble_payload;
    ble_comm_frame_t *ble_frame_out = (ble_comm_frame_t *)out_data;
    ble_comm_new_fw_rdy_ack_t *ble_new_fw_rdy_ack = (ble_comm_new_fw_rdy_ack_t *)&ble_frame_out->ble_payload;
    ble_ota_param_t ota_param;

    data_to_bigmode((uint8_t *)&ble_new_fw_rdy->file_size, sizeof(ble_new_fw_rdy->file_size));
    data_to_bigmode((uint8_t *)&ble_new_fw_rdy->image_crc, sizeof(ble_new_fw_rdy->image_crc));
    data_to_bigmode((uint8_t *)&ble_new_fw_rdy->customer_code, sizeof(ble_new_fw_rdy->customer_code));

    log_debug("new_fw pid %s, hw %s, fw %s si %d, crc 0x%04x, code %d\r\n", 
                            (char *)ble_new_fw_rdy->product_id,
                            (char *)ble_new_fw_rdy->hw_version,
                            (char *)ble_new_fw_rdy->fw_version,
                            ble_new_fw_rdy->file_size,
                            ble_new_fw_rdy->image_crc, 
                            ble_new_fw_rdy->customer_code);

    memcpy(ota_param.product_id, ble_new_fw_rdy->product_id, sizeof(ota_param.product_id));
    memcpy(ota_param.hw_version, ble_new_fw_rdy->hw_version, sizeof(ota_param.hw_version));
    memcpy(ota_param.fw_version, ble_new_fw_rdy->fw_version, sizeof(ota_param.fw_version));
    ota_param.file_size = ble_new_fw_rdy->file_size;
    ota_param.image_crc = ble_new_fw_rdy->image_crc;
    ota_param.customer_code = ble_new_fw_rdy->customer_code;

    //填充应答数据
    ble_new_fw_rdy_ack->op_result = ble_ota_upgrade_info_check(&ota_param);
//    if (ble_new_fw_rdy_ack->op_result == STA_NEW_FW_SUCCESS)
    {
//        ble_ota_upgrade_info_update(&ota_param);
    }
    
    log_debug("STA_NEW_FW_SUCCESS %d\r\n", ble_new_fw_rdy_ack->op_result);
    ble_frame_out->ble_header.option = APP_CMD_NEW_FW_RDY_ACK;
    *out_len = sizeof(ble_comm_new_fw_rdy_ack_t) + 1;
}

//请求升级文件
void ble_comm_upgrade_file_req(uint8_t* out_data, uint16_t* out_len)
{
    ble_comm_frame_t *ble_frame_out = (ble_comm_frame_t *)out_data;
    ble_comm_get_upgrade_file_t *get_upgrade_file = (ble_comm_get_upgrade_file_t *)&ble_frame_out->ble_payload;

    ble_frame_out->ble_header.option = APP_EVT_UPGRADE_FILE_REQ;
    
    ble_ota_get_current_upfrade_file_info((uint8_t *)get_upgrade_file);

    *out_len = sizeof(ble_comm_get_upgrade_file_t) + 1;
}

//回复升级文件
void ble_comm_upgrade_file_resp(uint8_t* in_data)
{
    ble_comm_frame_t *ble_frame = (ble_comm_frame_t *)in_data;
    ble_comm_upgrade_data_t *upgrade_data = (ble_comm_upgrade_data_t *)&ble_frame->ble_payload;

    
    data_to_bigmode((uint8_t *)&upgrade_data->data_addr, sizeof(upgrade_data->data_addr));
    data_to_bigmode((uint8_t *)&upgrade_data->data_size, sizeof(upgrade_data->data_size));
    //log_debug("ad %08x, ds %04x\r\n", upgrade_data->data_addr, upgrade_data->data_size);
    ble_ota_upgrade_fw_process((uint8_t *)upgrade_data);
}

void ble_comm_get_upgrade_file_done_req(uint8_t* out_data, uint16_t* out_len)
{
    ble_comm_frame_t *ble_frame = (ble_comm_frame_t *)out_data;
    ble_comm_get_upgrade_file_done_t *get_upgrade_file_done = (ble_comm_get_upgrade_file_done_t *)&ble_frame->ble_payload;
    ble_comm_get_upgrade_file_t upgrade_file_info;

    ble_frame->ble_header.option = APP_EVT_UPGRADE_FILE_DONE;
    ble_ota_get_current_upfrade_file_info((uint8_t *)&upgrade_file_info);
    memcpy(get_upgrade_file_done->product_id, upgrade_file_info.product_id, sizeof(get_upgrade_file_done->product_id));
    memcpy(get_upgrade_file_done->hw_version, upgrade_file_info.hw_version, sizeof(get_upgrade_file_done->hw_version));
    memcpy(get_upgrade_file_done->fw_version, upgrade_file_info.fw_version, sizeof(get_upgrade_file_done->fw_version));
    get_upgrade_file_done->file_size = upgrade_file_info.file_size;
    get_upgrade_file_done->image_crc = upgrade_file_info.image_crc;
    get_upgrade_file_done->customer_code = upgrade_file_info.customer_code;
    get_upgrade_file_done->err_code = ble_ota_get_err_code();

    * out_len = sizeof(ble_comm_get_upgrade_file_done_t) + 1;
}


void ble_comm_get_upgrade_file_done_ack_resq(uint8_t* in_data)
{
//    ble_comm_frame_t *ble_frame = (ble_comm_frame_t *)in_data;
    
    
}


void ble_packet_reply_frame(u8 channel, u16* len, u8* data) // 输入data只包括op + param， 输出的是从通道到CRC的全数据
{ 
//    u16 order  = 0;
    u16 crc    = 0;
    u16 length = *len;
    u8 encrypt_flag = 0;
    u8 enctypt_data[128];
    ble_comm_frame_t *ble_frame = (ble_comm_frame_t *) data;
    ble_comm_header_t *ble_header = &ble_frame->ble_header;

    if (*len == 0) return;

    if (channel == 0) // 数据需要加密
    {
        ble_header->frame_head = HEAD_ENCRYP;
        data_to_smallmode((uint8_t *)&ble_frame->ble_header.frame_head, sizeof(ble_header->frame_head));
        encrypt_flag      = 1;
    }
    else // 鉴权通道明文
    {
        ble_header->frame_head = HEAD_AUTH;
        data_to_smallmode((uint8_t *)&ble_frame->ble_header.frame_head, sizeof(ble_header->frame_head));
    }
    
    memcpy(enctypt_data, &ble_frame->ble_header.option, length); // op + param
    
    if (encrypt_flag == 1) // 需要加密
    {
        u16 encrypt_len = length; // op + param
        u16 encrypt_add_len = 0;
        u8 n            = 0;

        if ((encrypt_len % 16) != 0)
        {
            encrypt_add_len = 16 - encrypt_len % 16; // 还差多少个字节才到16的倍数
            // 补足16个字节倍数
            enctypt_data[length] = 0x80;

            for (n = 1; n < encrypt_add_len; n++)
            {
                enctypt_data[length+n] = 0;
            }
            // 更新长度
            length += encrypt_add_len;

            encrypt_len = length;
        }

        ble_encrypt_data(enctypt_data, enctypt_data, encrypt_len);
    }
    
    memcpy(&ble_header->option, enctypt_data, length);
    
    
    ble_header->frame_len = length+2; //payload + crc data
    ble_header->frame_len_invert = (ble_header->frame_len ^ 0xFFFF);
    data_to_smallmode((uint8_t *)&ble_header->frame_len, sizeof(ble_header->frame_len));
    data_to_smallmode((uint8_t *)&ble_header->frame_len_invert, sizeof(ble_header->frame_len_invert));

    crc = CRC16(0, (u8*)(&ble_header->frame_len), length+4); // 从dflen ~ param 当前order 从DFAID ~ PARAM 多了两字节 DFAID
    enctypt_data[length++] = crc >> 8;
    enctypt_data[length++] = crc & 0xFF;
    memcpy(&ble_header->option, enctypt_data, length);
    *len = length + sizeof(ble_comm_header_t) - 1;  //frame_len + head data + len
}

void ble_comm_frame_data_to_bigmode(uint8_t* in_data)
{
    ble_comm_frame_t *ble_frame = (ble_comm_frame_t *) in_data;
    ble_comm_header_t *ble_header = &ble_frame->ble_header;
    
    data_to_bigmode((uint8_t*)(&ble_frame->ble_header.frame_head), sizeof(ble_header->frame_head));
    data_to_bigmode((uint8_t*)(&ble_header->frame_len), sizeof(ble_header->frame_len));
    data_to_bigmode((uint8_t*)(&ble_header->frame_len_invert), sizeof(ble_header->frame_len_invert));
}

//处理手机端发送过来的命令或数据
uint8_t ble_communication_parse(uint8_t* in_data, uint16_t in_len, uint8_t* out_data, uint16_t* out_len)
{
    uint8_t rtv = FALSE;
    uint16_t len;
    u16 crc     = 0;
    u16 crc_in = (in_data[in_len - 2]<<8 | in_data[in_len - 1]);
    // u16 order = 0;
    u8 channel = 0;
    u8* text   = NULL;
    ble_comm_frame_t *ble_frame = (ble_comm_frame_t *)in_data;
    ble_comm_header_t *ble_header = &ble_frame->ble_header;

    crc  = CRC16(0, &in_data[2], (in_len - 4)); // DFLEN4 + OP + PARAM   so len + 2
    
    ble_comm_frame_data_to_bigmode(in_data);

    if (ble_header->frame_head == HEAD_ENCRYP) // 透传通道, 密文
    {
        if (s_cBlePrivilege == BLE_PRIVILEGE_NONE)  //没有权限是不允许走密文通道
        {
            return FALSE;
        }
        channel = 0;
    }
    else if (ble_header->frame_head == HEAD_AUTH) // 鉴权通道，不需要解密
    {
        channel = 1;
    }
    else // 其他通道暂时不支持
    {
        return FALSE;
    }

    if ((ble_header->frame_len ^ 0xFFFF) != ble_header->frame_len_invert)
    {
        printf("frame_len %d\r\n", ble_header->frame_len);
        return FALSE;
    }

    if ((in_len - 6) != ble_header->frame_len) // 输入的长度 - 2字节通道 - 4字节DFLEN
    {
        printf("in len %d %d\r\n",in_len - 6,  ble_header->frame_len);
        return FALSE;
    }

    //log_debug("[tsk-ble]:get crc %04x in crc: %04x\r\n", crc, crc_in);

    ////暂时不判定CRC值
    if (crc != crc_in)
    {
        printf("crc %04x, %04x\r\n", crc, crc_in);
        return FALSE;
    }

    if (channel != 1) // 加密通道, 需要解密
    {
        //log_debug("decrypt_data 0x%02x\r\n", text);
        text = &ble_header->option;
        ble_decrypt_data(text, text, (ble_header->frame_len - 2));
        if (s_cBlePrivilege == BLE_PRIVILEGE_TEMP) //临时权限只有添加用户可以进去
        {
            if ((ble_header->option != APP_CMD_ADD_USER) && (ble_header->option != APP_CMD_GET_BAT) && (ble_header->option != APP_CMD_OPENDOOR)) // 操作码
            {
                return FALSE;
            }
        }
    }
    rtv = TRUE;

    //log_debug("BLE CMD:%02x!\r\n", ble_header->option);
    
    switch (ble_header->option)
    {
        case APP_CMD_AUTH: // 验证命令/鉴权
        {
            log_debug("[tsk-ble]:APP_CMD_AUTH ok!\r\n");
            ble_host_authenticate(in_data, out_data, &len);
            break;
        }
        
        case APP_CMD_GET_BAT: // 获取电池电量命令
            log_debug("[tsk-ble]:APP_CMD_GET_BAT ok!\r\n");
            ble_get_battery_info(out_data, &len);
            break;

        case APP_CMD_GET_VERSION: // 获取版本命令
            log_debug("[tsk-ble]:APP_CMD_GET_VERSION ok!\r\n");
            ble_get_version(out_data, &len);
            break;

        case APP_CMD_NEW_FW_RDY:
            log_debug("APP_CMD_NEW_FW_RDY ok!\r\n");
            ble_new_fw_ready_notice(in_data, out_data, &len);
            break;
        case APP_EVT_UPGRADE_FILE_RESP:
            //log_debug("APP_EVT_UPGRADE_FILE_RESP!\r\n");
            ble_comm_upgrade_file_resp(in_data);
            len = 0;
            break;
        case APP_EVT_UPGRADE_FILE_DONE_RESP:
            log_debug("APP_EVT_UPGRADE_FILE_DONE_RESP!\r\n");
            ble_comm_get_upgrade_file_done_ack_resq(in_data);
            len = 0;
            break;
        default:
            log_debug("BLE ERR CMD %d!\r\n", ble_header->option);
            len = 0;
            break;
    }

    ble_packet_reply_frame(channel, &len, out_data);
    *out_len = len;
    return rtv;
}

// ble 通信层数据传输,把host下发的数据重新封装,提供给应用层使用
void ble_packet_parse(uint8_t const* in_data, uint8_t const in_len, uint8_t* out_data)
{
    BLE_FRAME_DATA* ble_frame_data = (BLE_FRAME_DATA*)in_data;
    ble_packet_tag* p_ble_pkt_tag  = (ble_packet_tag*)out_data;
    uint8_t data_len               = 0;

    //接收的数据长度有误,本次通信结束
    if (ble_frame_data->cfhead.packet_len != in_len || ble_frame_data->cfhead.packet_len < 3
        || ble_frame_data->cfhead.packet_len > BLE_PKT_MAX_LEN)
    {
        log_debug("rerr1 pkt len: %d\r\n", ble_frame_data->cfhead.packet_len);
        p_ble_pkt_tag->RxCurrentLen = 0;
        p_ble_pkt_tag->RxTotalLen   = 0;
        return;
    }

    //接收的帧序号与顺序号有冲突,说明有漏帧,本次通信结束
    //if (ble_frame_data->cfid != ble_frame_data->cfid % 4)
    //{
    //    log_debug("rerr2\r\n");
    //    p_ble_pkt_tag->RxCurrentLen = 0;
    //    p_ble_pkt_tag->RxTotalLen   = 0;
    //    return;
    //}

    data_len = ble_frame_data->cfhead.packet_len - 2;
    memcpy(&p_ble_pkt_tag->RxBuf[p_ble_pkt_tag->RxTotalLen], ble_frame_data->cfpayload, data_len);
    p_ble_pkt_tag->RxTotalLen += data_len;

    //接收到的为结束帧
    if (ble_frame_data->cfhead.frame_end == FRAME_END)
    {
        p_ble_pkt_tag->RxFinishFlag = 1;
    }
}

void ble_nodified_host_process(uint8_t option, uint8_t status, uint8_t *out_data, uint16_t *out_len)
{
    uint8_t encryp_chn = 0;    //默认加密通道
    switch(option)
    {
        case APP_EVT_OPENDOOR:
            log_debug("APP_EVT_OPENDOOR %d\r\n",status);
            //ble_open_lock_ack(out_data, out_len, status);
            break;
        case APP_EVT_UPGRADE_FILE_REQ:
            //log_debug("APP_EVT_UPGRADE_FILE_REQ\r\n");
            ble_comm_upgrade_file_req(out_data, out_len);
            break;
        case APP_EVT_UPGRADE_FILE_DONE:
            log_debug("APP_EVT_UPGRADE_FILE_DONE\r\n");
            ble_comm_get_upgrade_file_done_req(out_data, out_len);
            break;
    }

    //鉴权通道,不使用加密
    if (option == APP_EVT_AUTH)
        encryp_chn = 1;
    
    ble_packet_reply_frame(encryp_chn, out_len, out_data);
}

