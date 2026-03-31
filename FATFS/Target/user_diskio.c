/* USER CODE BEGIN Header */
/**
  * 调试版 DMA user_diskio.c
  */
/* USER CODE END Header */

#include <string.h>
#include <stdio.h> 
#include "ff_gen_drv.h"
#include "user_diskio.h"
#include "main.h"
#include "spi.h"
#include "usart.h"

// 引用 huart2 用于暴力调试
extern UART_HandleTypeDef huart2;
void Debug_Log(char* str) {
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 10);
}

#define SD_CS_PORT   GPIOA
#define SD_CS_PIN    GPIO_PIN_4

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_tx;

static volatile DSTATUS Stat = STA_NOINIT;
static uint8_t CardType;
volatile uint8_t SpiTxCplt = 0; 

static void SPI_SetSpeed(uint32_t prescaler) {
    __HAL_SPI_DISABLE(&hspi1);
    hspi1.Init.BaudRatePrescaler = prescaler;
    HAL_SPI_Init(&hspi1);
    __HAL_SPI_ENABLE(&hspi1);
}

static uint8_t SPI_ReadWriteByte(uint8_t txData) {
    uint8_t rxData;
    HAL_SPI_TransmitReceive(&hspi1, &txData, &rxData, 1, 10); 
    return rxData;
}

static void SD_CS_HIGH(void) { HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET); }
static void SD_CS_LOW(void) { HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET); }

static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg, uint8_t crc) {
    uint8_t r1, retry;
    SD_CS_HIGH(); SPI_ReadWriteByte(0xFF); SD_CS_LOW();
    SPI_ReadWriteByte(cmd | 0x40); SPI_ReadWriteByte(arg >> 24); SPI_ReadWriteByte(arg >> 16);
    SPI_ReadWriteByte(arg >> 8); SPI_ReadWriteByte(arg); SPI_ReadWriteByte(crc);
    if (cmd == 12) SPI_ReadWriteByte(0xFF);
    retry = 20; do { r1 = SPI_ReadWriteByte(0xFF); } while ((r1 & 0x80) && retry--);
    return r1;
}

DSTATUS USER_initialize (BYTE pdrv) {
    uint8_t n, r1, buff[4];
    uint16_t retry;
    if (pdrv) return STA_NOINIT;
    SD_CS_HIGH(); for (n = 0; n < 10; n++) SPI_ReadWriteByte(0xFF);
    SPI_SetSpeed(SPI_BAUDRATEPRESCALER_256);
    retry = 500; do { r1 = SD_SendCmd(0, 0, 0x95); } while ((r1 != 0x01) && retry--);
    if (r1 != 0x01) { SD_CS_HIGH(); SPI_SetSpeed(SPI_BAUDRATEPRESCALER_8); return STA_NOINIT; }
    CardType = 0;
    if (SD_SendCmd(8, 0x1AA, 0x87) == 1) { 
        for (n = 0; n < 4; n++) buff[n] = SPI_ReadWriteByte(0xFF);
        if (buff[2] == 0x01 && buff[3] == 0xAA) {
            retry = 20000; do { SD_SendCmd(55, 0, 0x01); r1 = SD_SendCmd(41, 1UL << 30, 0x01); } while (r1 && retry--);
            if (retry && SD_SendCmd(58, 0, 0x01) == 0) {
                for (n = 0; n < 4; n++) buff[n] = SPI_ReadWriteByte(0xFF); CardType = (buff[0] & 0x40) ? 6 : 4; 
            }
        }
    } else {
        SD_SendCmd(55, 0, 0x01); r1 = SD_SendCmd(41, 0, 0x01);
        if (r1 <= 1) { CardType = 2; retry = 20000; do { SD_SendCmd(55, 0, 0x01); r1 = SD_SendCmd(41, 0, 0x01); } while (r1 && retry--); } 
        else { CardType = 1; retry = 20000; do { r1 = SD_SendCmd(1, 0, 0x01); } while (r1 && retry--); }
        if (retry == 0 || SD_SendCmd(16, 512, 0x01) != 0) CardType = 0;
    }
    SD_CS_HIGH(); SPI_ReadWriteByte(0xFF);
    SPI_SetSpeed(SPI_BAUDRATEPRESCALER_8); 
    if (CardType) Stat &= ~STA_NOINIT; else Stat = STA_NOINIT;
    return Stat;
}

DSTATUS USER_status (BYTE pdrv) { return Stat; }
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count) {
    if (pdrv || !count) return RES_PARERR; if (Stat & STA_NOINIT) return RES_NOTRDY;
    if (CardType != 6 && CardType != 4) sector *= 512;
    SD_CS_LOW();
    if (count == 1) {
        if (SD_SendCmd(17, sector, 0x01) == 0) {
            uint32_t timer = 50000; while (SPI_ReadWriteByte(0xFF) != 0xFE && timer--);
            if (timer) { for (int i = 0; i < 512; i++) buff[i] = SPI_ReadWriteByte(0xFF); SPI_ReadWriteByte(0xFF); SPI_ReadWriteByte(0xFF); } 
            else { SD_CS_HIGH(); return RES_ERROR; }
        }
    } else return RES_ERROR; 
    SD_CS_HIGH(); SPI_ReadWriteByte(0xFF); return RES_OK;
}

#if _USE_WRITE == 1
DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count) {
    if (pdrv || !count) return RES_PARERR; if (Stat & STA_NOINIT) return RES_NOTRDY; if (Stat & STA_PROTECT) return RES_WRPRT;
    if (CardType != 6 && CardType != 4) sector *= 512;
    SD_CS_LOW();
    if (count == 1) {
        if (SD_SendCmd(24, sector, 0x01) == 0) {
            SPI_ReadWriteByte(0xFF); SPI_ReadWriteByte(0xFF); SPI_ReadWriteByte(0xFF); SPI_ReadWriteByte(0xFE); 
            
            // --- DMA START ---
            SpiTxCplt = 0; 
            
            // 调试打印：DMA启动前
            // Debug_Log("[D]"); 
            
            if(HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)buff, 512) == HAL_OK) {
                uint32_t timeout = 0xFFFFFF; 
                while(SpiTxCplt == 0) {
                    if(timeout-- == 0) {
                        HAL_SPI_Abort(&hspi1);
                        Debug_Log("[TIMEOUT]"); // 打印超时
                        return RES_ERROR;
                    }
                }
                // Debug_Log("[OK]"); // 打印成功
            } else {
                Debug_Log("[HAL_ERR]"); // 打印启动失败
                return RES_ERROR; 
            }
            // --- DMA END ---
            
            SPI_ReadWriteByte(0xFF); SPI_ReadWriteByte(0xFF); 
            if ((SPI_ReadWriteByte(0xFF) & 0x1F) == 0x05) {
                uint32_t timer = 500000; while (SPI_ReadWriteByte(0xFF) != 0xFF && timer--);
                if (timer) count = 0; 
            }
        }
    } else return RES_ERROR;
    SD_CS_HIGH(); SPI_ReadWriteByte(0xFF); return count ? RES_ERROR : RES_OK;
}
#endif

DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff) {
    DRESULT res = RES_ERROR; if (pdrv) return RES_PARERR;
    switch (cmd) {
        case CTRL_SYNC: SD_CS_LOW(); { uint32_t t = 500000; while(SPI_ReadWriteByte(0xFF) != 0xFF && t--); if(t) res = RES_OK; } SD_CS_HIGH(); break;
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
	#if _USE_WRITE 
	USER_write, 
	#endif 
	#if _USE_IOCTL == 1 
	USER_ioctl, 
	#endif
};

