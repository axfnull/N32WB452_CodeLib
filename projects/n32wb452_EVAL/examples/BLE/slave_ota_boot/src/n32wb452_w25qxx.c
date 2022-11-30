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
 ******************************************************************************
 * @file n32wb452_w25qxx.c
 * @author Nations
 * @version v1.0.1
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 * @date    2019-12-20
 * @brief  文件提供w25qxx系列读写擦操作功能源文件
 *
 ******************************************************************************
 */
#include "n32wb452.h"
#include "n32wb452_w25qxx.h"
#include "main.h"
#include "log.h"
#include "bsp_spi.h"
#include "string.h"

#define SPI_ReadWrite(dat)    SPI3_ReadWriteByte(dat)

uint16_t W25QXX_TYPE = W25Q32; //默认是W25Q32
//0XEF13,表示芯片型号为W25Q80
//0XEF14,表示芯片型号为W25Q16
//0XEF15,表示芯片型号为W25Q32
//0XEF16,表示芯片型号为W25Q64
//0XEF17,表示芯片型号为W25Q128


/**
 * @brief W25QX系列字符串
 */
typedef struct w25qxx_type_t
{
    uint16_t val;   /* 类型 */
    uint8_t *pstr;   /* 字符串描述 */
} w25qxx_type;


const w25qxx_type spi_flash_string[] = {
    {0xEF13, "W25Q80"},
    {0xEF14, "W25Q16"},
    {0xEF15, "W25Q32"},
    {0xEF16, "W25Q64"},
    {0xEF17, "W25Q128"}
};

//4Kbytes为一个Sector
//16个扇区为1个Block
//W25Q32
//容量??M字节,共有64个Block
//初始化SPI FLASH的IO??

void W25QXX_Init(void)
{
    uint32_t i;
    GPIO_InitType GPIO_InitStructure;

    //CLOSE JTAG/OPEN SWD, PA15功能复用
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO | RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB, ENABLE);
    AFIO->RMP_CFG &= 0xF8FFFFFF;
    AFIO->RMP_CFG |= 0x02000000;

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);

    /* 使能SPI引脚相关的时钟 */
    RCC_EnableAPB2PeriphClk(SPI_FLASH_RCCCLK , ENABLE);
    
    /* 配置SPI的 CS引脚，普通IO即可 */
    GPIO_InitStructure.Pin   = SPI_FLASH_CS_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitPeripheral(SPI_FLASH_CS_PORT, &GPIO_InitStructure);
    
    /* 停止信号 FLASH: CS引脚高电平*/
    __SPI_FLASH_CS_SET();
    
    SPI_Configuration();

    W25QXX_TYPE = W25QXX_ReadID(); //读取FLASH ID.
    W25QXX_TYPE = W25QXX_ReadID(); //读取FLASH ID.

    /* 查看FLASHID CHECK SPI读写正常 */
    for (i = 0; i < (sizeof(spi_flash_string) / sizeof(spi_flash_string[0])); i++) {
        if (W25QXX_TYPE == spi_flash_string[i].val) {
            log_debug("(W25Qxx)spi flash id = %s.\r\n", spi_flash_string[i].pstr);
            break;
        }
    }
    if (i >= (sizeof(spi_flash_string) / sizeof(spi_flash_string[0]))) {
        log_debug("(W25Qxx)spi flash id = %s.\r\n", "none");
    }

}

//读取W25QXX的状态寄存器
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:默认0,状态寄存器保护??配合WP使用
//TB,BP2,BP1,BP0:FLASH区域写保护设??
//WEL:写使能锁??
//BUSY:忙标记位(1,??0,空闲)
//默认:0x00
uint8_t W25QXX_ReadSR(void)
{
    uint8_t byte = 0;
    uint8_t snd  = 0;

    __SPI_FLASH_CS_CLR(); //使能器件

    snd = W25X_ReadStatusReg; //发送读取状态寄存器命令
    SPI_ReadWrite(snd);

    snd = 0Xff; //读取一个字??
    byte = SPI_ReadWrite(snd);

    __SPI_FLASH_CS_SET(); //取消片选
    return byte;
}
//写W25QXX状态寄存器
//只有SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)可以??!!
void W25QXX_Write_SR(uint8_t sr)
{
    uint8_t snd = 0;

    __SPI_FLASH_CS_CLR(); //使能器件
    snd = W25X_WriteStatusReg;   //发送写取状态寄存器命令
    SPI_ReadWrite(snd);
    
    snd = sr; //读取一个字??
    SPI_ReadWrite(snd);
    __SPI_FLASH_CS_SET(); //取消片选
}
//W25QXX写使??
//将WEL置位
void W25QXX_Write_Enable(void)
{
    uint8_t snd = 0;

    __SPI_FLASH_CS_CLR(); //使能器件
    snd = W25X_WriteEnable;      //发送写使能
    SPI_ReadWrite(snd);
    __SPI_FLASH_CS_SET(); //取消片选
}
//W25QXX写禁??
//将WEL清零
void W25QXX_Write_Disable(void)
{
    uint8_t snd = 0;

    __SPI_FLASH_CS_CLR(); //使能器件
    snd = W25X_WriteDisable;     //发送写禁止指令
    SPI_ReadWrite(snd);
    __SPI_FLASH_CS_SET(); //取消片选
}
//读取芯片ID
//返回值如??
//0XEF13,表示芯片型号为W25Q80
//0XEF14,表示芯片型号为W25Q16
//0XEF15,表示芯片型号为W25Q32
//0XEF16,表示芯片型号为W25Q64
//0XEF17,表示芯片型号为W25Q128
uint16_t W25QXX_ReadID(void)
{
    uint8_t snd;
    uint8_t rcv;
    uint16_t Temp = 0;

    //参考W25Q读取ID时序，首先拉底片??
    __SPI_FLASH_CS_CLR();

    snd = 0x90; //发送读取ID命令
    rcv = SPI_ReadWrite(snd);


    snd = 0x00;
    rcv = SPI_ReadWrite(snd);


    snd = 0x00;
    rcv = SPI_ReadWrite(snd);


    snd = 0x00;
    rcv = SPI_ReadWrite(snd);


    snd = 0xFF;
    rcv = SPI_ReadWrite(snd);

    Temp |= rcv << 8;

    snd = 0xFF;
    rcv = SPI_ReadWrite(snd);
    Temp |= rcv;

    __SPI_FLASH_CS_SET();

    return Temp;
}
//读取SPI FLASH
//在指定地址开始读取指定长度的数据
//pBuffer:数据存储??
//ReadAddr:开始读取的地址(24bit)
//NumByteToRead:要读取的字节??最??5535)
void W25QXX_Read(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead)
{
    uint8_t snd = 0;
    uint8_t rcv = 0;
    uint16_t i;

    __SPI_FLASH_CS_CLR(); //使能器件
    snd = W25X_ReadData;         //发送读取命??
    rcv = SPI_ReadWrite(snd);


    snd = ((uint8_t)((ReadAddr) >> 16)); //发??4bit地址
    rcv = SPI_ReadWrite(snd);


    snd = ((uint8_t)((ReadAddr) >> 8));
    rcv = SPI_ReadWrite(snd);


    snd = ((uint8_t)ReadAddr);
    rcv = SPI_ReadWrite(snd);

    for (i = 0; i < NumByteToRead; i++)
    {
        snd = 0XFF; //循环读数
        rcv = SPI_ReadWrite(snd);

        pBuffer[i] = rcv;
        //log_debuginfo("<%02x>,", pBuffer[i]);
    }

    __SPI_FLASH_CS_SET(); //取消片选
}
//SPI在一??0~65535)内写入少??56个字节的数据
//在指定地址开始写入最??56字节的数??
//pBuffer:数据存储??
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节??最??56),该数不应该超过该页的剩余字节??!!
void W25QXX_Write_Page(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint16_t i;
    uint8_t snd = 0;

    W25QXX_Write_Enable(); //SET WEL

    __SPI_FLASH_CS_CLR(); //使能器件

    snd = W25X_PageProgram; //发送写页命??
    SPI_ReadWrite(snd);


    snd = ((uint8_t)((WriteAddr) >> 16)); //发??4bit地址
    SPI_ReadWrite(snd);


    snd = ((uint8_t)((WriteAddr) >> 8));
    SPI_ReadWrite(snd);

    snd = ((uint8_t)WriteAddr);
    SPI_ReadWrite(snd);


    for (i = 0; i < NumByteToWrite; i++)
    {
        SPI_ReadWrite(pBuffer[i]);
    }
    __SPI_FLASH_CS_SET(); //取消片选
    W25QXX_Wait_Busy();           //等待写入结束
}
//无检验写SPI FLASH
//必须确保所写的地址范围内的数据全部??XFF,否则在非0XFF处写入的数据将失??
//具有自动换页功能
//在指定地址开始写入指定长度的数据,但是要确保地址不越??
//pBuffer:数据存储??
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节??最??5535)
//CHECK OK
void W25QXX_Write_NoCheck(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint16_t pageremain;
    pageremain = 256 - WriteAddr % 256; //单页剩余的字节数
    if (NumByteToWrite <= pageremain)
        pageremain = NumByteToWrite; //不大??56个字??
    while (1)
    {
        W25QXX_Write_Page(pBuffer, WriteAddr, pageremain);
        if (NumByteToWrite == pageremain)
            break; //写入结束??
        else       //NumByteToWrite>pageremain
        {
            pBuffer += pageremain;
            WriteAddr += pageremain;

            NumByteToWrite -= pageremain; //减去已经写入了的字节??
            if (NumByteToWrite > 256)
                pageremain = 256; //一次可以写??56个字??
            else
                pageremain = NumByteToWrite; //不够256个字节了
        }
    };
}
//写SPI FLASH
//在指定地址开始写入指定长度的数据
//该函数带擦除操作!
//pBuffer:数据存储??
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节??最??5535)
static uint8_t W25QXX_BUFFER[4096];
void W25QXX_Write(uint8_t* pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite)
{
    uint32_t secpos;
    uint16_t secoff;
    uint16_t secremain;
    uint16_t i;
    uint8_t* W25QXX_BUF;
    W25QXX_BUF = W25QXX_BUFFER;
    secpos     = WriteAddr / 4096; //扇区地址
    secoff     = WriteAddr % 4096; //在扇区内的偏??
    secremain  = 4096 - secoff;    //扇区剩余空间大小
    //printf("ad:%X,nb:%X\r\n",WriteAddr,NumByteToWrite);//测试??
    if (NumByteToWrite <= secremain)
        secremain = NumByteToWrite; //不大??096个字??
    while (1)
    {
        W25QXX_Read(W25QXX_BUF, secpos * 4096, 4096); //读出整个扇区的内??
        for (i = 0; i < secremain; i++)               //校验数据
        {
            if (W25QXX_BUF[secoff + i] != 0XFF)
                break; //需要擦??
        }
        if (i < secremain) //需要擦??
        {
            W25QXX_Erase_Sector(secpos);    //擦除这个扇区
            for (i = 0; i < secremain; i++) //复制
            {
                W25QXX_BUF[i + secoff] = pBuffer[i];
            }
            W25QXX_Write_NoCheck(W25QXX_BUF, secpos * 4096, 4096); //写入整个扇区
        }
        else
            W25QXX_Write_NoCheck(pBuffer, WriteAddr, secremain); //写已经擦除了??直接写入扇区剩余区间.
        if (NumByteToWrite == secremain)
            break; //写入结束??
        else       //写入未结??
        {
            secpos++;   //扇区地址??
            secoff = 0; //偏移位置??

            pBuffer += secremain;        //指针偏移
            WriteAddr += secremain;      //写地址偏移
            NumByteToWrite -= secremain; //字节数递减
            if (NumByteToWrite > 4096)
                secremain = 4096; //下一个扇区还是写不完
            else
                secremain = NumByteToWrite; //下一个扇区可以写完了
        }
    };
}

void W25QXX_Clear(u32 ReadAddr, uint32_t NumByteToWrite)
{
    memset(W25QXX_BUFFER, 0, sizeof(W25QXX_BUFFER));

    W25QXX_Write(W25QXX_BUFFER, ReadAddr, NumByteToWrite);
}


//擦除整个芯片
//等待时间超长...
void W25QXX_Erase_Chip(void)
{
    uint8_t snd = 0;

    W25QXX_Write_Enable(); //SET WEL
    W25QXX_Wait_Busy();
    __SPI_FLASH_CS_CLR();     //使能器件
    snd = ((uint8_t)W25X_ChipErase); //发送片擦除命令
    SPI_ReadWrite(snd);

    __SPI_FLASH_CS_SET(); //取消片选
    W25QXX_Wait_Busy();           //等待芯片擦除结束
}
//擦除一个扇??
//Dst_Addr:扇区地址 根据实际容量设置
//擦除一个山区的最少时??150ms
void W25QXX_Erase_Sector(uint32_t Dst_Addr)
{
    uint8_t snd = 0;
    //监视falsh擦除情况,测试??
    //printf("fe:%x\r\n",Dst_Addr);
    Dst_Addr *= 4096;
    W25QXX_Write_Enable(); //SET WEL
    W25QXX_Wait_Busy();

    __SPI_FLASH_CS_CLR(); //使能器件

    SPI_ReadWrite(W25X_SectorErase);
    snd = ((uint8_t)((Dst_Addr) >> 16)); //发??4bit地址
    SPI_ReadWrite(snd);

    snd = ((uint8_t)((Dst_Addr) >> 8));
    SPI_ReadWrite(snd);

    snd = ((uint8_t)Dst_Addr);
    SPI_ReadWrite(snd);

    __SPI_FLASH_CS_SET(); //取消片选
    W25QXX_Wait_Busy();           //等待擦除完成
}
//等待空闲
void W25QXX_Wait_Busy(void)
{
    while ((W25QXX_ReadSR() & 0x01) == 0x01)
        ; // 等待BUSY位清??
}
//进入掉电模式
void W25QXX_PowerDown(void)
{
    __SPI_FLASH_CS_CLR(); //使能器件
    SPI_ReadWrite(W25X_PowerDown);
    __SPI_FLASH_CS_SET(); //取消片选

    delay_ms(1);                  //等待TPD
}
//唤醒
void W25QXX_WAKEUP(void)
{
    __SPI_FLASH_CS_CLR(); //使能器件
    SPI_ReadWrite(W25X_ReleasePowerDown);
    __SPI_FLASH_CS_SET(); //取消片选

    delay_ms(3);                  //等待TRES1
}
