/* USER CODE BEGIN Header */
/**
  * 混合版 user_diskio.c
  * 1. 初始化：使用 HAL 库 (稳)
  * 2. 读写数据：使用寄存器直操 (快)
  * 3. 速度：4.5MHz (Prescaler 16) - 最佳平衡点
  */
/* USER CODE END Header */

#include <string.h>
#include <stdio.h> 
#include "ff_gen_drv.h"
#include "user_diskio.h"
#include "main.h"
#include "spi.h"
#include "usart.h" // 确保能打印调试信息

// 调试打印
#define SD_LOG(format, ...)  { char buf[64]; snprintf(buf, 64, format, ##__VA_ARGS__); HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100); }

#define SD_CS_PORT   GPIOA
#define SD_CS_PIN    GPIO_PIN_4

extern SPI_HandleTypeDef hspi1;
static volatile DSTATUS Stat = STA_NOINIT;
static uint8_t CardType;

// =========================================================
// 1. 基础函数 (混合模式)
// =========================================================

// 【安全模式】用于命令发送，带重试
static uint8_t SPI_RxByte_Safe(void) {
    uint8_t tx = 0xFF;
    uint8_t rx;
    // 使用 HAL 库确保兼容性
    if(HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 100) != HAL_OK) return 0xFF;
    return rx;
}

static void SPI_TxByte_Safe(uint8_t data) {
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, 100);
}

// 【极速模式】用于大量数据读写，直接操作寄存器
static uint8_t SPI_ReadWrite_Fast(uint8_t data) {
    // 1. 等待发送空
    while((hspi1.Instance->SR & SPI_FLAG_TXE) == RESET);
    // 2. 发送
    hspi1.Instance->DR = data;
    // 3. 等待接收非空
    while((hspi1.Instance->SR & SPI_FLAG_RXNE) == RESET);
    // 4. 读取
    return hspi1.Instance->DR;
}

static void SPI_SetSpeed(uint32_t prescaler) {
    __HAL_SPI_DISABLE(&hspi1);
    hspi1.Init.BaudRatePrescaler = prescaler;
    HAL_SPI_Init(&hspi1);
    __HAL_SPI_ENABLE(&hspi1);
}

static void SD_SPI_SpeedLow(void) {
    SPI_SetSpeed(SPI_BAUDRATEPRESCALER_256);
}

static void SD_SPI_SpeedHigh(void) {
    // 【关键】使用 16 (4.5MHz)。配合寄存器写入，足以应付 TXT
    SPI_SetSpeed(SPI_BAUDRATEPRESCALER_16); 
}

static void SD_CS_HIGH(void) {
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET);
}

static void SD_CS_LOW(void) {
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET);
}

// =========================================================
// 2. 命令发送 (安全模式)
// =========================================================
static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg, uint8_t crc) {
    uint8_t r1, retry;

    SD_CS_HIGH();
    SPI_TxByte_Safe(0xFF);
    SPI_TxByte_Safe(0xFF);
    SD_CS_LOW();

    SPI_TxByte_Safe(cmd | 0x40);
    SPI_TxByte_Safe(arg >> 24);
    SPI_TxByte_Safe(arg >> 16);
    SPI_TxByte_Safe(arg >> 8);
    SPI_TxByte_Safe(arg);
    SPI_TxByte_Safe(crc);

    if (cmd == 12) SPI_TxByte_Safe(0xFF);

    retry = 10;
    do {
        r1 = SPI_RxByte_Safe();
    } while ((r1 & 0x80) && retry--);

    return r1;
}

// =========================================================
// 3. FATFS 接口
// =========================================================

DSTATUS USER_initialize (BYTE pdrv) {
    uint8_t n, r1, buff[4];
    uint16_t retry;

    if (pdrv) return STA_NOINIT;

    // 初始化过程完全使用 HAL 库 (求稳)
    SD_CS_HIGH();
    for (n = 0; n < 10; n++) SPI_TxByte_Safe(0xFF);

    SD_SPI_SpeedLow(); 

    retry = 200;
    do { r1 = SD_SendCmd(0, 0, 0x95); } while ((r1 != 0x01) && retry--);

    if (r1 != 0x01) {
        SD_CS_HIGH(); SD_SPI_SpeedHigh(); 
        return STA_NOINIT;
    }

    CardType = 0;
    if (SD_SendCmd(8, 0x1AA, 0x87) == 1) { 
        for (n = 0; n < 4; n++) buff[n] = SPI_RxByte_Safe();
        if (buff[2] == 0x01 && buff[3] == 0xAA) {
            retry = 20000;
            do {
                SD_SendCmd(55, 0, 0x01); r1 = SD_SendCmd(41, 1UL << 30, 0x01);
            } while (r1 && retry--);
            if (retry && SD_SendCmd(58, 0, 0x01) == 0) {
                for (n = 0; n < 4; n++) buff[n] = SPI_RxByte_Safe();
                CardType = (buff[0] & 0x40) ? 6 : 4; 
            }
        }
    } else {
        SD_SendCmd(55, 0, 0x01); r1 = SD_SendCmd(41, 0, 0x01);
        if (r1 <= 1) {
            CardType = 2; retry = 20000;
            do { SD_SendCmd(55, 0, 0x01); r1 = SD_SendCmd(41, 0, 0x01); } while (r1 && retry--);
        } else {
            CardType = 1; retry = 20000;
            do { r1 = SD_SendCmd(1, 0, 0x01); } while (r1 && retry--);
        }
        if (retry == 0 || SD_SendCmd(16, 512, 0x01) != 0) CardType = 0;
    }

    SD_CS_HIGH(); SPI_TxByte_Safe(0xFF);
    
    // 初始化完成后，切换到高速
    SD_SPI_SpeedHigh(); 

    if (CardType) Stat &= ~STA_NOINIT;
    else Stat = STA_NOINIT;

    return Stat;
}

DSTATUS USER_status (BYTE pdrv) { return Stat; }

DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count) {
    if (pdrv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    if (CardType != 6 && CardType != 4) sector *= 512;

    SD_CS_LOW();

    if (count == 1) {
        if (SD_SendCmd(17, sector, 0x01) == 0) {
            uint32_t timer = 50000;
            while (SPI_RxByte_Safe() != 0xFE && timer--);
            if (timer) {
                // 【极速读取】循环内使用寄存器操作
                for (int i = 0; i < 512; i++) {
                    buff[i] = SPI_ReadWrite_Fast(0xFF);
                }
                SPI_ReadWrite_Fast(0xFF); SPI_ReadWrite_Fast(0xFF); 
                count = 0; // 【修复】必须清零
            }
        }
    } else {
        if (SD_SendCmd(18, sector, 0x01) == 0) {
            do {
                uint32_t timer = 50000;
                while (SPI_RxByte_Safe() != 0xFE && timer--);
                if (!timer) break;
                // 【极速读取】
                for (int i = 0; i < 512; i++) {
                    buff[i] = SPI_ReadWrite_Fast(0xFF);
                }
                SPI_ReadWrite_Fast(0xFF); SPI_ReadWrite_Fast(0xFF);
                buff += 512;
            } while (--count);
            SD_SendCmd(12, 0, 0x01);
        }
    }

    SD_CS_HIGH(); SPI_TxByte_Safe(0xFF);
    return count ? RES_ERROR : RES_OK;
}

#if _USE_WRITE == 1
DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count) {
    if (pdrv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;
    if (Stat & STA_PROTECT) return RES_WRPRT;

    if (CardType != 6 && CardType != 4) sector *= 512;

    SD_CS_LOW();

    if (count == 1) {
        if (SD_SendCmd(24, sector, 0x01) == 0) {
            SPI_TxByte_Safe(0xFF); SPI_TxByte_Safe(0xFF); SPI_TxByte_Safe(0xFF);
            SPI_TxByte_Safe(0xFE); 
            
            // 【极速写入】循环内使用寄存器操作
            // 这里是写入数据的瓶颈所在，使用 Direct Register Access 提速
            for (int i = 0; i < 512; i++) {
                SPI_ReadWrite_Fast(buff[i]);
            }
            
            SPI_ReadWrite_Fast(0xFF); SPI_ReadWrite_Fast(0xFF);
            
            if ((SPI_RxByte_Safe() & 0x1F) == 0x05) count = 0; // 【修复】必须清零
        }
    } else {
        if (CardType & 2) { SD_SendCmd(55, 0, 0x01); SD_SendCmd(23, count, 0x01); }
        if (SD_SendCmd(25, sector, 0x01) == 0) {
            do {
                SPI_TxByte_Safe(0xFF); SPI_TxByte_Safe(0xFF); SPI_TxByte_Safe(0xFC);
                
                // 【极速写入】
                for (int i = 0; i < 512; i++) {
                    SPI_ReadWrite_Fast(buff[i]);
                }

                SPI_TxByte_Safe(0xFF); SPI_TxByte_Safe(0xFF);
                if ((SPI_RxByte_Safe() & 0x1F) != 0x05) break;
                buff += 512;
            } while (--count);
            SPI_TxByte_Safe(0xFD);
        }
    }

    uint32_t timer = 200000;
    while (SPI_RxByte_Safe() != 0xFF && timer--);

    SD_CS_HIGH(); SPI_TxByte_Safe(0xFF);
    return count ? RES_ERROR : RES_OK;
}
#endif

DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff) {
    DRESULT res = RES_ERROR;
    if (pdrv) return RES_PARERR;

    switch (cmd) {
        case CTRL_SYNC:
            SD_CS_LOW();
            { uint32_t t = 0xFFFFFF; while(SPI_RxByte_Safe() != 0xFF && t--); if(t) res = RES_OK; }
            SD_CS_HIGH();
            break;
        case GET_SECTOR_COUNT: *(DWORD*)buff = 1024 * 1024 * 8; res = RES_OK; break;
        case GET_SECTOR_SIZE: *(WORD*)buff = 512; res = RES_OK; break;
        case GET_BLOCK_SIZE: *(DWORD*)buff = 1; res = RES_OK; break;
    }
    return res;
}

Diskio_drvTypeDef USER_Driver = {
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif
};