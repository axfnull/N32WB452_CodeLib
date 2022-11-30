/*
 * ble_encrypt_interface.c
 *
 *  Created on: Apr 4, 2019
 *      Author: andy
 *  Description:

 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "sha1.h"
#include "SecuritySM4.h"
#include "ble_encrypt_interface.h"
#include "sys_type.h"

static volatile char cBleSessionKey[16];

u8 ble_encrypt_authenticate_to_get_session_key(BLE_ENCRYPT_AUTH_DATA_T* auth_data,
                                               u8 admin_pwd[20],
                                               u8 admin_len,
                                               u8 out_rnd[8]) // 使用SHA1算法，获取鉴权结果，输出8字节随机数
{
    u8 rtv = FALSE;
    u8 hash[20];
    u8 temp_buf[28];        // 20B HASH + 8 random
    int temp_len = 28;      //这里不处理的话开优化会有问题
    u8 local_auth_data[20]; //

    if (get_sha1((const char*)admin_pwd, admin_len, hash) == TRUE)
    {   
        //0x31, 0xD3, 0x49, 0x2F, 0x3D, 0x96, 0x2D, 0xA7;

        memcpy(temp_buf, hash, 20);
        memcpy(&temp_buf[20], auth_data->ble_rnd, 8); // 管理员密码sha1后 + RND

        if (get_sha1((const char*)temp_buf, temp_len, local_auth_data) == TRUE) // 计算sha1获取鉴权数据
        {
            if (memcmp(auth_data->ble_auth, local_auth_data, 20) == 0) // 鉴权成功
            {
                rtv = TRUE;
            }
        }
    }

    if (rtv == TRUE) // 鉴权成功，可以继续计算session key
    {
        // if (hub_get_random(8, out_rnd) == FALSE) // 如果获取随机数失败
        {
            memset(out_rnd, 0xF1, 8);
        }

        memcpy(temp_buf, auth_data->ble_rnd, 4);           // app端 随机数高4字节
        memcpy(&temp_buf[4], out_rnd, 4);                  // 锁端  随机数高4字节
        memcpy(&temp_buf[8], &(auth_data->ble_rnd[4]), 4); // app端 随机数低4字节
        memcpy(&temp_buf[12], &out_rnd[4], 4);             // 锁端  随机数低4字节
        temp_len = 16;                                     // 只加密16个字节

        getByteEncryptMessageSM4((char*)temp_buf,
                                 (char*)cBleSessionKey,
                                 (int*)&temp_len,
                                 (const char*)hash); // 使用哈希的前16字节做key对temp_buf里的数据加密获取session key
        // getByteEncryptMessageSM4除非heap不够，否则不会错误，在此不再判断
        //        PRINTF("ble_encrypt_authenticate_to_get_session_key,cBleSessionKey:");
        //        SEGGER_RTT_print_data(cBleSessionKey, 16);
    }
    
    return rtv;
}

// 传入的text请自行补足至16的倍数，这样text和out_encrypt可以使用同一指针
u8 ble_encrypt_data(u8* text, u8* out_encrypt, u32 len) // 输入明文地址和长度，输出密文。 明文请自行保证16字节倍数，不满足补0x80000000
{
    if (getByteEncryptMessageSM4((char*)text, (char*)out_encrypt, (int*)&len, (const char*)cBleSessionKey) == TRUE)
    {
        return TRUE;
    }

    return FALSE;
}

// 传入的 encrypt_data 请自行补足至16的倍数，这样text和 encrypt_data 可以使用同一指针
u8 ble_decrypt_data(u8* encrypted_data,
                    u8* out_decrypt,
                    u32 len) // 输入密文地址和长度，输出明文。 明文请自行保证16字节倍数，不满足补0x80000000
{
    //    PRINTF("ble_decrypt_data,cBleSessionKey:");
    //    SEGGER_RTT_print_data(cBleSessionKey, 16);
    if (getByteDecryptMessageSM4((char*)encrypted_data, (char*)out_decrypt, (int)len, (const char*)cBleSessionKey) == TRUE)
    {
        return TRUE;
    }

    return FALSE;
}
