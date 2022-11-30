/**
 * \file SecuritySM4.h
 */

#ifndef SECURITY_SM4_H_
#define SECURITY_SM4_H_

char getByteEncryptMessageSM4(char* text, char* encrptout, int* len, const char* key);

char getByteDecryptMessageSM4(char* text, char* output, int len, const char* key);

#endif
