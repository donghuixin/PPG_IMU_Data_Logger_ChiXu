/* USER CODE BEGIN Header */
/**
  * 信号量触发版 main.c (伪线程模式)
  * 1. 机制：数据积累 -> 触发信号量 -> 主循环执行写入。
  * 2. 优势：解耦数据接收和数据写入，写入逻辑不再阻塞数据搬运。
  */
/* USER CODE END Header */
#include "main.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>

/* USER CODE BEGIN PV */
// --- MPU6050 ---
#define MPU_SAMPLES     50
#define IMU_TOTAL_SIZE  (MPU_SAMPLES * 16) 
typedef struct {
    int16_t Accel_X; int16_t Accel_Y; int16_t Accel_Z;
    int16_t Temp;
    int16_t Gyro_X;  int16_t Gyro_Y;  int16_t Gyro_Z;
} MPU6050_Data_t;
MPU6050_Data_t MpuBuffer[MPU_SAMPLES];
uint8_t MpuSendPacket[IMU_TOTAL_SIZE]; 

// --- 接收缓冲 ---
#define RX_BUFFER_SIZE  4096 
uint8_t RxRingBuffer[RX_BUFFER_SIZE]; 
uint32_t write_ptr = 0; 
uint32_t read_ptr = 0;  

// --- SD 卡缓存 ---
// 双倍缓冲大小，防止写入时溢出 (虽然我们用单缓冲+信号量也能跑)
#define SD_CACHE_SIZE   4096 
uint8_t SDCacheBuf[SD_CACHE_SIZE];
uint32_t sd_buf_idx = 0;

// --- 文件系统变量 ---
FATFS fs;
FIL fil;
FRESULT fres;
uint32_t bytes_written;
uint8_t is_sd_ready = 0;
uint8_t is_file_open = 0; 
volatile uint8_t sys_state = 0; 

// --- 信号量 (伪线程触发器) ---
volatile uint8_t sd_write_semaphore = 0; // 0:空闲, 1:请求写入
uint32_t sd_write_len = 0; // 记录需要写入的长度

// --- 串口指令 ---
uint8_t rx_byte_temp;          
#define CMD_MAX_LEN 32         
char rx_cmd_buffer[CMD_MAX_LEN]; 
uint8_t rx_cmd_index = 0;      

uint8_t mpu_index = 0;
volatile uint8_t mpu_timer_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN 0 */
void Uart_Print(char* str) {
    if(huart2.Instance != NULL) HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 100);
}

// 数据分发器 (生产者)
void Data_Distribute(uint8_t* data, uint16_t len) {
    if (len == 0) return;
    
    // 1. 转发到串口2 (可选，调试用)
    // HAL_UART_Transmit(&huart2, data, len, 100);
    
    // 2. 存入 SD 卡缓存
    if (sys_state == 2 && is_file_open && is_sd_ready) {
        char text_tmp[8]; 
        for (int i = 0; i < len; i++) {
            int str_len = sprintf(text_tmp, "%02X", data[i]);
            
            // 检查缓存是否越界
            if (sd_buf_idx + str_len < SD_CACHE_SIZE) {
                memcpy(&SDCacheBuf[sd_buf_idx], text_tmp, str_len);
                sd_buf_idx += str_len;
            } else {
                // 缓存满了！强制触发写入 (保护机制)
                sd_write_semaphore = 1;
                sd_write_len = sd_buf_idx;
                break; 
            }
        }
        
        // 【信号量触发逻辑】
        // 当缓存积累到 2048 字节 (一半) 时，请求写入
        // 这样可以留出一半空间给写入期间进来的新数据
        if (sd_buf_idx >= 2048 && sd_write_semaphore == 0) {
            sd_write_semaphore = 1; // 举旗：请求写入
            sd_write_len = sd_buf_idx; // 锁定当前长度
        }
    }
}

// SD 卡写入任务 (消费者) - 被信号量触发
void SD_Write_Thread(void) {
    // 只有当信号量为 1 时才执行
    if (sd_write_semaphore == 1) {
        
        if (sys_state == 2 && is_file_open && is_sd_ready) {
            
            // 执行耗时的 SPI 写入
            fres = f_write(&fil, SDCacheBuf, sd_write_len, &bytes_written);
            
            if (fres == FR_OK) {
                Uart_Print("."); // 心跳
                
                // 【关键】数据搬移 (Ring Buffer 逻辑)
                // 如果刚才写入期间有新数据进来 (sd_buf_idx > sd_write_len)，
                // 我们需要把新数据搬到缓存头部，防止丢失。
                if (sd_buf_idx > sd_write_len) {
                    // 把尾部剩余的数据搬到头部
                    uint32_t remaining = sd_buf_idx - sd_write_len;
                    memmove(SDCacheBuf, &SDCacheBuf[sd_write_len], remaining);
                    sd_buf_idx = remaining;
                } else {
                    // 写入完美同步，清零即可
                    sd_buf_idx = 0;
                }
            } else {
                char msg[32]; sprintf(msg, "E:%d", fres); Uart_Print(msg);
                // 出错也清零，不死鸟逻辑
                sd_buf_idx = 0; 
            }
        }
        
        // 任务完成，放下信号量
        sd_write_semaphore = 0;
    }
}

void MPU6050_Init(void) {
    uint8_t Data = 0;
    if (HAL_I2C_IsDeviceReady(&hi2c2, 0xD0, 2, 100) == HAL_OK) {
        HAL_I2C_Mem_Write(&hi2c2, 0xD0, 0x6B, 1, &Data, 1, 100);
    }
}

uint8_t MPU6050_Read(MPU6050_Data_t *data) {
    uint8_t Rec_Data[14];
    if(HAL_I2C_Mem_Read(&hi2c2, 0xD0, 0x3B, 1, Rec_Data, 14, 2) == HAL_OK) {
        data->Accel_X = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        data->Accel_Y = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
        data->Accel_Z = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
        data->Temp    = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
        data->Gyro_X  = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
        data->Gyro_Y  = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
        data->Gyro_Z  = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);
        return 1; 
    }
    return 0; 
}

void Package_IMU_Data(void) {
    for (int i = 0; i < MPU_SAMPLES; i++) {
        uint16_t offset = i * 16;
        MpuSendPacket[offset]     = 0xAA;
        MpuSendPacket[offset + 1] = 0x55;
        memcpy(&MpuSendPacket[offset + 2], &MpuBuffer[i], 14);
    }
}

void Create_New_File(char* filename) {
    char full_name[32];
    if (strlen(filename) > 8) { Uart_Print("\r\n[ERR] Name too long.\r\n"); return; }
    if (strstr(filename, ".TXT") == NULL && strstr(filename, ".txt") == NULL) sprintf(full_name, "%s.TXT", filename);
    else strcpy(full_name, filename);

    Uart_Print("\r\nCreating: "); Uart_Print(full_name); Uart_Print("...\r\n");

    fres = f_open(&fil, full_name, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres == FR_OK) {
        is_sd_ready = 1; is_file_open = 1; sd_buf_idx = 0; sys_state = 2; 
        
        char* header = "START\r\n";
        f_write(&fil, header, strlen(header), &bytes_written);
        f_sync(&fil); 
        Uart_Print("File Created!\r\n");
    } else {
        char msg[32]; sprintf(msg, "Err: %d\r\n", fres); Uart_Print(msg);
        sys_state = 0; 
    }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();

  /* USER CODE BEGIN 2 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_3; 
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT; 
  GPIO_InitStruct.Pull = GPIO_PULLUP; 
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_Delay(2000); 

  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  
  sys_state = 0; 
  memset(rx_cmd_buffer, 0, CMD_MAX_LEN); 
  rx_cmd_index = 0;
  Uart_Print("\r\n=== READY ===\r\n");

  fres = f_mount(&fs, "", 1);
  if (fres == FR_OK) { is_sd_ready = 1; Uart_Print("Mount OK.\r\n"); } 
  else { is_sd_ready = 0; Uart_Print("Mount Fail!\r\n"); }

  MPU6050_Init();
  HAL_UART_Receive_IT(&huart2, &rx_byte_temp, 1);
  HAL_TIM_Base_Start_IT(&htim3);
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  /* USER CODE END 2 */

  /* USER CODE BEGIN 1 */
  static uint8_t is_dma_running = 0; 
  /* USER CODE END 1 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    
    // 【消费者线程】检查信号量，有请求才执行
    SD_Write_Thread();

    // 【生产者逻辑】处理 DMA 数据和 MPU 数据
    if (sys_state == 2) { 
        if (is_dma_running == 0) {
            __HAL_UART_CLEAR_OREFLAG(&huart1); __HAL_UART_CLEAR_NEFLAG(&huart1);
            read_ptr = 0; write_ptr = 0;
            HAL_UART_Receive_DMA(&huart1, RxRingBuffer, RX_BUFFER_SIZE);
            is_dma_running = 1; 
        }
        
        // 搬运 DMA 数据
        write_ptr = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
        if (write_ptr != read_ptr) {
            if (write_ptr > read_ptr) { Data_Distribute(&RxRingBuffer[read_ptr], write_ptr - read_ptr); read_ptr = write_ptr; } 
            else { Data_Distribute(&RxRingBuffer[read_ptr], RX_BUFFER_SIZE - read_ptr); if (write_ptr > 0) Data_Distribute(&RxRingBuffer[0], write_ptr); read_ptr = write_ptr; }
        }
        
        // 搬运 IMU 数据
        if (mpu_timer_flag) {
            mpu_timer_flag = 0;
            if (MPU6050_Read(&MpuBuffer[mpu_index]) == 1) mpu_index++;
            if (mpu_index >= MPU_SAMPLES) { mpu_index = 0; Package_IMU_Data(); Data_Distribute(MpuSendPacket, IMU_TOTAL_SIZE); }
        }
    } else { 
        // 停止/空闲状态处理
        if (is_dma_running == 1) {
            HAL_UART_DMAStop(&huart1); is_dma_running = 0;
            // 刷入剩余数据
            uint32_t last_pos = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            if (last_pos > read_ptr) Data_Distribute(&RxRingBuffer[read_ptr], last_pos - read_ptr);
            
            // 写入最后缓存
            if(is_file_open && sd_buf_idx > 0) {
                 f_write(&fil, SDCacheBuf, sd_buf_idx, &bytes_written);
            }

            if(is_file_open) {
                Uart_Print("\r\nClosing...\r\n");
                // 停止时屏蔽中断，确保安全
                HAL_NVIC_DisableIRQ(USART1_IRQn); HAL_NVIC_DisableIRQ(TIM3_IRQn);
                HAL_Delay(50); 
                fres = f_close(&fil); 
                HAL_NVIC_EnableIRQ(USART1_IRQn); HAL_NVIC_EnableIRQ(TIM3_IRQn);

                if(fres == FR_OK) Uart_Print("CLOSED.\r\n");
                else { char msg[32]; sprintf(msg, "Err: %d\r\n", fres); Uart_Print(msg); }
                is_file_open = 0; 
            }
        }
    }
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
// 回调函数区 (保持不变)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (HAL_GetTick() < 3000) { HAL_UART_Receive_IT(&huart2, &rx_byte_temp, 1); return; }
        if (rx_byte_temp == '\n' || rx_byte_temp == '\r') {
            if (rx_cmd_index > 0) { 
                rx_cmd_buffer[rx_cmd_index] = '\0'; 
                if (strstr(rx_cmd_buffer, "stop") != NULL) { sys_state = 0; }
                else if (strstr(rx_cmd_buffer, "reset") != NULL) { Uart_Print("\r\nRebooting...\r\n"); NVIC_SystemReset(); }
                else if (sys_state == 0 && strstr(rx_cmd_buffer, "start") != NULL) { sys_state = 1; Uart_Print("\r\n[CMD] Enter Name:\r\n"); }
                else if (sys_state == 1 && strstr(rx_cmd_buffer, "start") == NULL) { Create_New_File(rx_cmd_buffer); }
            }
            rx_cmd_index = 0; memset(rx_cmd_buffer, 0, CMD_MAX_LEN);
        } else {
            if (rx_cmd_index < CMD_MAX_LEN - 1) {
                if ((rx_byte_temp >= '0' && rx_byte_temp <= 'z') || rx_byte_temp == '.') rx_cmd_buffer[rx_cmd_index++] = rx_byte_temp;
            }
        }
        HAL_UART_Receive_IT(&huart2, &rx_byte_temp, 1);
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) { if (huart->Instance == USART1) { __HAL_UART_CLEAR_OREFLAG(huart); __HAL_UART_CLEAR_NEFLAG(huart); __HAL_UART_CLEAR_FEFLAG(huart); } }
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) { if (htim->Instance == TIM3) mpu_timer_flag = 1; }
/* USER CODE END 4 */

void Error_Handler(void) { __disable_irq(); while (1) {} }
void SystemClock_Config(void) {
  // 请保留你原来的时钟配置
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}