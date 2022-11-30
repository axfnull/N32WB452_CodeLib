/*
 * SM4/SMS4 algorithm test programme
 * 2012-4-21
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "sm4.h"
#include "SecuritySM4.h"
#include "sys_type.h"


char getByteEncryptMessageSM4(char* text, char* encrptout, int* len, const char* key)
{
    //  int i;

    // cal output len
    int output_len = *len;
    unsigned char mkeychar[16];
    unsigned char* mtextchar = NULL;
    //  unsigned char * output = NULL;
    sm4_context ctx;

    //printf("***waring:getByteEncryptMessageSM4, pvPortMalloc output_len=%d\r\n", output_len);
    mtextchar = malloc(output_len + 32);

    if (mtextchar == NULL)
    {
        // PRINTF("getByteEncryptMessageSM4,mtextchar == NULL\r\n");
        return FALSE;
    }
    //  output = malloc(output_len+32);
    //
    //    if (output == NULL)
    //    {
    //        free(mtextchar);
    //        return FALSE;
    //    }

    if (output_len % 16 != 0)
    {
        output_len = (output_len / 16 + 1) * 16;
    }
    else
    {
        output_len = output_len;
    }

    memset(mtextchar, 0, output_len);

    memcpy(mtextchar, text, MIN(*len, output_len));

    memcpy(mkeychar, key, 16);

    sm4_setkey_enc(&ctx, mkeychar);
    sm4_crypt_ecb(&ctx, 1, output_len, mtextchar, (unsigned char*)encrptout);
    //    memcpy(encrptout,output,output_len);
    *len = output_len;
    //    free(output);
    free(mtextchar);
    return TRUE;
}

char getByteDecryptMessageSM4(char* text, char* decrptout, int len, const char* key)
{
    //  int i;

    //  unsigned char *mtextchar=NULL;
    unsigned char mkeychar[16];
    //  unsigned char * output =NULL;
    sm4_context ctx;
    //  mtextchar = malloc(len);
    //
    //    if (mtextchar == NULL)
    //    {
    //        return FALSE;
    //    }
    //    memcpy(mtextchar,text,len);
    //    output = malloc(len);
    //
    //    if (output == NULL)
    //    {
    //        free(mtextchar);
    //        return FALSE;
    //    }
    memcpy(mkeychar, key, 16);
    sm4_setkey_dec(&ctx, mkeychar);
    sm4_crypt_ecb(&ctx, 0, len, (unsigned char*)text, (unsigned char*)decrptout);
    //    memcpy(decrptout,output,len);
    //  decrptout[len] = 0;     //这里越界，会造成内存溢出
    //  free(output);
    //    free(mtextchar);
    return TRUE;
}

