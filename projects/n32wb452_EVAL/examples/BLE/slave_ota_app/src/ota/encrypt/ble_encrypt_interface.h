#ifndef BLE_ENCRYPT_INTERFACE_H_
#define BLE_ENCRYPT_INTERFACE_H_

#include "n32wb452.h"

typedef struct
{
    u8 ble_rnd[8];
    u8 ble_auth[20];
} BLE_ENCRYPT_AUTH_DATA_T;

u8 ble_encrypt_authenticate_to_get_session_key(BLE_ENCRYPT_AUTH_DATA_T* auth_data,
                                               u8 admin_pwd[20],
                                               u8 admin_len,
                                               u8 out_rnd[8]); // 使用SHA1算法，获取鉴权结果，输出8字节随机数
u8 ble_encrypt_data(u8* text, u8* out_encrypt, u32 len); // 输入明文地址和长度，输出密文。 明文请自行保证16字节倍数，不满足补0x80000000
u8 ble_decrypt_data(u8* encrypted_data, u8* out_decrypt, u32 len); // 输入密文地址和长度，输出明文。

#endif
