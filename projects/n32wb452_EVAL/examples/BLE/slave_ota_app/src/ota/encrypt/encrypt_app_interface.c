/*
 * encrypt_app_interface.c
 *
 *  Created on: Oct 12, 2018
 *      Author: andy
 *  Description:

 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "SecuritySM4.h"
#include "encrypt_app_interface.h"
#include "sys_type.h"

// static const char ROOT_KEY[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

static const char ROOT_KEY[] = "NATIONS LOCK KEY";

#if (LOCAL_ALGORITHM == ENCRYPTION_ALGORITHM_SM4)
static const char UNLOCK_KEY[] = "sapw;ldscx32r43rzdfoipnm@BSD";
static u8 cUnlockKeyHasBeenSet = 0;
static char s_cUnlckKey[20]    = {0};

static void get_unlock_discrete_key(const char* input_key, char* output_key)
{
    // 先取出前两字节和后两字节，暂存
    // 去除前两字节和后两字节后，取奇数字节
    // 取出的暂存字节放在最后
    // 所有字节再减1, 不考虑溢出

    u8 buf_head_tail[4] = {0};
    u8 buf[50]          = {0};
    u32 temp_len        = 0;
    u32 input_len       = strlen(input_key);
    u32 n               = 0;
    u8 oup_put_len      = 0;

    if (cUnlockKeyHasBeenSet != 0)
    {
        return;
    }

    cUnlockKeyHasBeenSet = 1;
    buf_head_tail[0]     = input_key[0];
    buf_head_tail[1]     = input_key[1];
    buf_head_tail[2]     = input_key[input_len - 1];
    buf_head_tail[3]     = input_key[input_len - 2];
    temp_len             = input_len - 4;
    u32 order            = 0;

    for (n = 0; n < temp_len; n++)
    {
        order = 3 + 2 * n;

        if (order >= temp_len)
        {
            break;
        }

        buf[n] = input_key[order];
    }

    order       = 0;
    buf[n++]    = buf_head_tail[order++];
    buf[n++]    = buf_head_tail[order++];
    buf[n++]    = buf_head_tail[order++];
    buf[n++]    = buf_head_tail[order++];
    oup_put_len = n;

    for (n = 0; n < oup_put_len; n++)
    {
        buf[n] = buf[n] - 1;
    }

    memcpy(output_key, buf, 16);
}

u8 check_data_is_dummy(u8* data, u32 int_len)
{
    u32 i;

    for (i = 0; i < int_len; ++i)
    {
        if (data[i] != 0xFF)
        {
            return FALSE;
        }
    }

    return TRUE;
}

#endif

// 传入的in_data请自行补足至16的倍数，这样in_data和 out_data可以使用同一指针
u8 encrypt_unlock_data(u8* in_data, u8* out_data, u32 in_len)
{
    char rtv = FALSE;
#if (LOCAL_ALGORITHM == ENCRYPTION_ALGORITHM_SM4)
    u8 tmp_data[32];
    u32 i, page;
    u32 tmp_len;

    if (check_data_is_dummy(in_data, in_len) == FALSE)
    {
        page = in_len / 32;

        for (i = 0; i < page; i++)
        {
            tmp_len = 32;
            memcpy(tmp_data, &in_data[i * 32], 32);
            get_unlock_discrete_key(UNLOCK_KEY, s_cUnlckKey);
            rtv = getByteEncryptMessageSM4((char*)tmp_data, (char*)tmp_data, (int*)&tmp_len, s_cUnlckKey);
            memcpy(&out_data[i * 32], tmp_data, 32);
        }

        in_len -= i * 32;
        tmp_len = in_len % 32;

        if (tmp_len != 0)
        {
            memset(tmp_data, 0, sizeof(tmp_data));
            memcpy(tmp_data, &in_data[i * 32], tmp_len);
            get_unlock_discrete_key(UNLOCK_KEY, s_cUnlckKey);
            rtv = getByteEncryptMessageSM4((char*)tmp_data, (char*)tmp_data, (int*)&tmp_len, s_cUnlckKey);
            memcpy(&out_data[i * 32], tmp_data, tmp_len);
        }
    }
    else
    {
        memcpy(out_data, in_data, in_len); //如果数据全为0XFF的话不加密
    }
#endif
    return rtv;
}

u8 decrypt_unlock_data(u8* in_data, u8* out_data, u32 in_len)
{
    char rtv = FALSE;
#if (LOCAL_ALGORITHM == ENCRYPTION_ALGORITHM_SM4)
    u8 tmp_data[32];
    u32 i, page;
    u32 tmp_len;

    if (check_data_is_dummy(in_data, in_len) == FALSE)
    {
        page = in_len / 32;

        for (i = 0; i < page; i++)
        {
            tmp_len = 32;
            memcpy(tmp_data, &in_data[i * 32], 32);
            get_unlock_discrete_key(UNLOCK_KEY, s_cUnlckKey);
            rtv = getByteDecryptMessageSM4((char*)tmp_data, (char*)tmp_data, (int)tmp_len, s_cUnlckKey);
            memcpy(&out_data[i * 32], tmp_data, 32);
        }

        in_len -= i * 32;
        tmp_len = in_len % 32;

        if (tmp_len != 0)
        {
            memset(tmp_data, 0, sizeof(tmp_data));
            memcpy(tmp_data, &in_data[i * 32], tmp_len);
            get_unlock_discrete_key(UNLOCK_KEY, s_cUnlckKey);
            rtv = getByteDecryptMessageSM4((char*)tmp_data, (char*)tmp_data, (int)tmp_len, s_cUnlckKey);
            memcpy(&out_data[i * 32], tmp_data, tmp_len);
        }
    }
    else
    {
        memcpy(out_data, in_data, in_len); //如果数据全为0XFF的话不解密
    }
#endif
    return rtv;
}

char encrypt_data(MSG_T* encrypted_msg_out, ENCRYPTED_DATA_T* unencrypted_data_in, u8 current_lock_rnd[8])
{
    // 首先通过两个RND计算过程密钥

    char rtv             = FALSE;
    int encrypt_data_len = 0;

    char key[16] = {0};
    char rnd[16] = {0};

    // 准备过程密钥
    memcpy(&rnd[8], unencrypted_data_in->random_from_server, sizeof(unencrypted_data_in->random_from_server));
    memcpy(rnd, unencrypted_data_in->random_from_lock, sizeof(unencrypted_data_in->random_from_lock));
    encrypt_data_len = sizeof(unencrypted_data_in->random_from_server) + sizeof(unencrypted_data_in->random_from_lock);

    rtv = getByteEncryptMessageSM4(rnd, key, &encrypt_data_len, ROOT_KEY); // 获取过程密钥存在key中

    if (rtv == FALSE)
    {
        return NOT_ENOUGH_HEAP_MEMORY;
    }

    u8 rnd_size = sizeof(unencrypted_data_in->random_from_lock);

    encrypt_data_len = unencrypted_data_in->msg.len + rnd_size; // 待加密长度 由 用户数据长度 + 随机数

    //    u8 *temp_encrypt_data = (u8 *)malloc(encrypt_data_len);
    //
    //    if (temp_encrypt_data == NULL)
    //    {
    //        return NOT_ENOUGH_HEAP_MEMORY;
    //    }

    // 真正加密的数据由门锁随机数 + 用户数据组成
    memcpy(encrypted_msg_out->data, current_lock_rnd, rnd_size); // 准备好 rnd数据

    if (unencrypted_data_in->msg.len != 0)
    {
        memcpy(&encrypted_msg_out->data[rnd_size], unencrypted_data_in->msg.data, unencrypted_data_in->msg.len); // 准备好用户数据
    }

    // 使用key加密temp_encrypt_data，加密后的数据保存在encrypted_msg_out->data，加密后的长度保存在encrypt_data_len
    rtv = getByteEncryptMessageSM4((char*)encrypted_msg_out->data, (char*)encrypted_msg_out->data, &encrypt_data_len, key);
    encrypted_msg_out->len = encrypt_data_len; // 转存密文长度

    //    free(temp_encrypt_data);

    return rtv;
}

// char decrypt_data(MSG_T *decrypted_msg_out, ENCRYPTED_DATA_T *encrypted_data_in, u8 current_server_rnd[8])
//{
//    // 首先通过两个RND计算过程密钥
//
//    char rtv = FALSE;
//
//    int encrypt_data_len = 0;
//    char key[16] = {0};
//    char rnd[16] = {0};
//
//    memcpy(&rnd[8], encrypted_data_in->random_from_server, sizeof(encrypted_data_in->random_from_server));
//    memcpy(rnd, encrypted_data_in->random_from_lock, sizeof(encrypted_data_in->random_from_lock));
//
//    encrypt_data_len = sizeof(encrypted_data_in->random_from_server) + sizeof(encrypted_data_in->random_from_lock);
//
//    rtv = getByteEncryptMessageSM4(rnd, key, &encrypt_data_len , ROOT_KEY);  // 获取过程密钥存在key中
//
//    if (rtv == FALSE)
//    {
//        return NOT_ENOUGH_HEAP_MEMORY;
//    }
//
//    encrypt_data_len = encrypted_data_in->msg.len; // 待解密长度 由 用户数据长度 + 随机数
//
//    u8 *temp_encrypt_data = (u8 *)malloc(encrypt_data_len);
//
//    if (temp_encrypt_data == NULL)
//    {
//        return NOT_ENOUGH_HEAP_MEMORY;
//    }
//
//    // 加密后的数据长度保证为16的倍数
//    // 解密 由key加密的encrypted_data_in->msg.data密文，解密后的数据保存在temp_encrypt_data, 长度encrypt_data_len
//    rtv = getByteDecryptMessageSM4((char *)encrypted_data_in->msg.data, (char *)temp_encrypt_data, encrypt_data_len , key);
//
//    if (rtv == TRUE)    // 解密OK，且内存足够
//    {
//        u8 rnd_size = sizeof(encrypted_data_in->random_from_server);    // 获取服务器随机数size
//
//        if (memcmp(temp_encrypt_data, current_server_rnd, rnd_size) == 0) // 比对成功，证明数据合法
//        {
//            rtv = TRUE;
//            encrypt_data_len -= rnd_size;
//
//            if (encrypt_data_len != 0)
//            {
//                memcpy(decrypted_msg_out->data, &temp_encrypt_data[rnd_size], encrypt_data_len); // 明文数据只返回用户数据
//            }
//            decrypted_msg_out->len = encrypt_data_len;  // 转存明文长度——明文用户数据长度
//        }
//        else
//        {
//            rtv = DECRYPTED_FALSE;
//        }
//    }
//
//    free(temp_encrypt_data);
//
//    return rtv;
//
//}

char decrypt_data(MSG_T* decrypted_msg_out, ENCRYPTED_DATA_T* encrypted_data_in, u8 current_server_rnd[8])
{
    // 首先通过两个RND计算过程密钥

    char rtv = FALSE;

    int encrypt_data_len = 0;
    char key[16]         = {0};
    char rnd[16]         = {0};

    memcpy(&rnd[8], encrypted_data_in->random_from_server, sizeof(encrypted_data_in->random_from_server));
    memcpy(rnd, encrypted_data_in->random_from_lock, sizeof(encrypted_data_in->random_from_lock));

    encrypt_data_len = sizeof(encrypted_data_in->random_from_server) + sizeof(encrypted_data_in->random_from_lock);

    rtv = getByteEncryptMessageSM4(rnd, key, &encrypt_data_len, ROOT_KEY); // 获取过程密钥存在key中

    if (rtv == FALSE)
    {
        return NOT_ENOUGH_HEAP_MEMORY;
    }

    encrypt_data_len = encrypted_data_in->msg.len; // 待解密长度 由 用户数据长度 + 随机数

    //    u8 *temp_encrypt_data = (u8 *)malloc(encrypt_data_len);
    //
    //    if (temp_encrypt_data == NULL)
    //    {
    //        return NOT_ENOUGH_HEAP_MEMORY;
    //    }

    // 加密后的数据长度保证为16的倍数
    // 解密 由key加密的encrypted_data_in->msg.data密文，解密后的数据保存在temp_encrypt_data, 长度encrypt_data_len
    //    rtv = getByteDecryptMessageSM4((char *)encrypted_data_in->msg.data, (char *)temp_encrypt_data, encrypt_data_len , key);
    rtv = getByteDecryptMessageSM4((char*)encrypted_data_in->msg.data, (char*)decrypted_msg_out->data, encrypt_data_len, key);

    if (rtv == TRUE) // 解密OK，且内存足够
    {
        u8 rnd_size = sizeof(encrypted_data_in->random_from_server); // 获取服务器随机数size

        if (memcmp(decrypted_msg_out->data, current_server_rnd, rnd_size) == 0) // 比对成功，证明数据合法
        {
            rtv = TRUE;
            encrypt_data_len -= rnd_size;

            if (encrypt_data_len != 0)
            {
                memcpy(decrypted_msg_out->data, &decrypted_msg_out->data[rnd_size], encrypt_data_len); // 明文数据只返回用户数据
            }
            decrypted_msg_out->len = encrypt_data_len; // 转存明文长度——明文用户数据长度
        }
        else
        {
            rtv = DECRYPTED_FALSE;
        }
    }

    //    free(temp_encrypt_data);

    return rtv;
}
