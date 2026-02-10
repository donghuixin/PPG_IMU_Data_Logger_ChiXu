/* USER CODE BEGIN Header */
/**
  * 双缓冲 (Ping-Pong) + DMA 极速写入版 main.c
  * 核心逻辑：
  * 1. 定义 DoubleBuf[2][512]。
  * 2. 串口2发什么，我们就往 DoubleBuf 里存什么。
  * 3. 凑够 512 字节，触发一次 Block Write (调用底层 DMA)。
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

// --- UART 接收缓冲 (2KB) ---
#define RX_BUFFER_SIZE  2048 
uint8_t RxRingBuffer[RX_BUFFER_SIZE]; 
uint32_t write_ptr = 0; 
uint32_t read_ptr = 0;  

// --- 【双缓冲核心定义】 ---
#define BLOCK_SIZE 800
uint8_t DoubleBuf[2][BLOCK_SIZE]; // 两个 512 字节的桶 (Ping-Pong)
uint8_t buf_fill_idx = 0;         // 当前正在填哪个桶 (0 或 1)
uint16_t buf_pos = 0;             // 当前桶填了多少

// --- 写入任务标记 ---
// 0xFF: 无任务
// 0: 桶0满了，请写桶0
// 1: 桶1满了，请写桶1
volatile uint8_t ready_to_write_idx = 0xFF; 

// --- 文件系统 ---
FATFS fs;
FIL fil;
FRESULT fres;
uint32_t bytes_written;
uint8_t is_sd_ready = 0;
uint8_t is_file_open = 0; 
volatile uint8_t sys_state = 0; 

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

// -------------------------------------------------------------
// 数据分发器 (生产者)
// -------------------------------------------------------------
void Data_Distribute(uint8_t* data, uint16_t len) {
    if (len == 0) return;
    
    // 1. 串口回显 (你要的功能：发送给串口2)
//    HAL_UART_Transmit(&huart2, data, len, 10); 
    
    // 2. 双缓冲填充 (存入 SD 卡)
    if (sys_state == 2 && is_file_open && is_sd_ready) {
        
        for (int i = 0; i < len; i++) {
            // 填入当前正在工作的桶
            DoubleBuf[buf_fill_idx][buf_pos++] = data[i];
            
            // 如果桶满了 (512字节)
            if (buf_pos >= BLOCK_SIZE) {
                // A. 发布任务：标记这个桶需要写入
                ready_to_write_idx = buf_fill_idx;
                
                // B. 切换到另一个桶 (0->1, 1->0)
                buf_fill_idx = !buf_fill_idx; 
                
                // C. 重置新桶的指针
                buf_pos = 0;
                
                // D. 调试打印 (可选，证明触发了)
                // Uart_Print("[TRIG]"); 
            }
        }
    }
}

// -------------------------------------------------------------
// SD 卡写入任务 (消费者) - 在主循环调用
// -------------------------------------------------------------
void SD_Write_Thread(void) {
    // 检查是否有“满桶”需要写入
    if (ready_to_write_idx != 0xFF) {
        
        if (sys_state == 2 && is_file_open && is_sd_ready) {
            
            // 锁定要写的 Buffer
            uint8_t* pBuf = DoubleBuf[ready_to_write_idx];
            
            // 直接调用 f_write 写入 512 字节
            // 因为大小正好是 512，user_diskio 会自动启动 DMA
            fres = f_write(&fil, pBuf, BLOCK_SIZE, &bytes_written);
            
            if (fres == FR_OK) {
                Uart_Print("."); // 打印一个点表示写入成功
            } else {
                char msg[32]; sprintf(msg, "[Err:%d]", fres); Uart_Print(msg);
            }
        }
        
        // 清除任务标记
        ready_to_write_idx = 0xFF;
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
    
    // 使用 .BIN 后缀，因为我们存的是原始二进制数据
    sprintf(full_name, "%s.BIN", filename); 

    Uart_Print("\r\nCreating: "); Uart_Print(full_name); Uart_Print("...\r\n");
    fres = f_open(&fil, full_name, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres == FR_OK) {
        is_sd_ready = 1; is_file_open = 1; 
        
        // 重置双缓冲状态
        buf_fill_idx = 0;
        buf_pos = 0;
        ready_to_write_idx = 0xFF;
        
        sys_state = 2; 
        Uart_Print("File Created!\r\n");
    } else {
        char msg[32]; sprintf(msg, "Err: %d\r\n", fres); Uart_Print(msg);
        sys_state = 0; 
    }
}

// 上电自检函数 (你要的功能)
void Boot_Test(void) {
    Uart_Print("\r\n>>> Boot Test (DMA Check) <<<\r\n");
    fres = f_open(&fil, "BOOT_TST.TXT", FA_CREATE_ALWAYS | FA_WRITE);
    if(fres == FR_OK) {
        // 借用 DoubleBuf[0] 做测试
        memset(DoubleBuf[0], 'A', 512); 
        fres = f_write(&fil, DoubleBuf[0], 512, &bytes_written);
        f_close(&fil);
        if(fres == FR_OK && bytes_written == 512) Uart_Print("Test Write OK!\r\n");
        else Uart_Print("Test Write Fail!\r\n");
    } else {
        Uart_Print("Test Open Fail!\r\n");
    }
    Uart_Print(">>> End Test <<<\r\n\r\n");
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
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

  // 开启 DMA 中断 (必须！)
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 0); 
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);         

  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  
  sys_state = 0; 
  memset(rx_cmd_buffer, 0, CMD_MAX_LEN); 
  rx_cmd_index = 0;
  Uart_Print("\r\n=== READY (DoubleBuf + DMA) ===\r\n");

  fres = f_mount(&fs, "", 1);
  if (fres == FR_OK) { 
      is_sd_ready = 1; 
      Uart_Print("Mount OK.\r\n"); 
      
      // 上电自检
      Boot_Test();
      
  } else { 
      is_sd_ready = 0; 
      Uart_Print("Mount Fail!\r\n"); 
  }

  MPU6050_Init();
  HAL_UART_Receive_IT(&huart2, &rx_byte_temp, 1);
  HAL_TIM_Base_Start_IT(&htim3);
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  /* USER CODE END 2 */

  /* USER CODE BEGIN 1 */
  static uint8_t is_dma_running = 0; 
  static uint32_t heartbeat_tick = 0;
  /* USER CODE END 1 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    
    // 心跳
    if (HAL_GetTick() - heartbeat_tick > 1000) {
        heartbeat_tick = HAL_GetTick();
        Uart_Print("*"); 
    }

    // 1. 写卡线程 (Ping-Pong 处理)
    SD_Write_Thread();

    // 2. 数据采集线程
    if (sys_state == 2) { 
        if (is_dma_running == 0) {
            __HAL_UART_CLEAR_OREFLAG(&huart1); __HAL_UART_CLEAR_NEFLAG(&huart1);
            read_ptr = 0; write_ptr = 0;
            HAL_UART_Receive_DMA(&huart1, RxRingBuffer, RX_BUFFER_SIZE);
            is_dma_running = 1; 
            Uart_Print("[Start Recv]");
        }
        
        write_ptr = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
        
        if (write_ptr != read_ptr) {
            if (write_ptr > read_ptr) { 
                Data_Distribute(&RxRingBuffer[read_ptr], write_ptr - read_ptr); 
                read_ptr = write_ptr; 
            } else { 
                Data_Distribute(&RxRingBuffer[read_ptr], RX_BUFFER_SIZE - read_ptr); 
                if (write_ptr > 0) Data_Distribute(&RxRingBuffer[0], write_ptr); 
                read_ptr = write_ptr; 
            }
        }
        
        if (mpu_timer_flag) {
            mpu_timer_flag = 0;
            if (MPU6050_Read(&MpuBuffer[mpu_index]) == 1) mpu_index++;
            if (mpu_index >= MPU_SAMPLES) { 
                mpu_index = 0; 
                Package_IMU_Data(); 
                Data_Distribute(MpuSendPacket, IMU_TOTAL_SIZE); 
            }
        }
    } else { 
        if (is_dma_running == 1) {
            HAL_UART_DMAStop(&huart1); is_dma_running = 0;
            // 处理残余数据
            uint32_t last_pos = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            if (last_pos > read_ptr) Data_Distribute(&RxRingBuffer[read_ptr], last_pos - read_ptr);
            
            // 将最后一个不满512字节的Buffer也写入
            if (buf_pos > 0) {
                 f_write(&fil, DoubleBuf[buf_fill_idx], buf_pos, &bytes_written);
            }

            if(is_file_open) {
                Uart_Print("\r\nClosing...\r\n");
                fres = f_close(&fil); 
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (HAL_GetTick() < 3000) { HAL_UART_Receive_IT(&huart2, &rx_byte_temp, 1); return; }
        if (rx_byte_temp == '\n' || rx_byte_temp == '\r') {
            if (rx_cmd_index > 0) { 
                rx_cmd_buffer[rx_cmd_index] = '\0'; 
                if (strstr(rx_cmd_buffer, "stop") != NULL) { sys_state = 0; }
                else if (strstr(rx_cmd_buffer, "reset") != NULL) { NVIC_SystemReset(); }
                else if (sys_state == 0 && strstr(rx_cmd_buffer, "start") != NULL) { sys_state = 1; Uart_Print("\r\n[CMD] Name?\r\n"); }
                else if (sys_state == 1 && strstr(rx_cmd_buffer, "start") == NULL) { Create_New_File(rx_cmd_buffer); }
            }
            rx_cmd_index = 0; memset(rx_cmd_buffer, 0, CMD_MAX_LEN);
        } else {
            if (rx_cmd_index < CMD_MAX_LEN - 1) rx_cmd_buffer[rx_cmd_index++] = rx_byte_temp;
        }
        HAL_UART_Receive_IT(&huart2, &rx_byte_temp, 1);
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) { if (huart->Instance == USART1) { __HAL_UART_CLEAR_OREFLAG(huart); __HAL_UART_CLEAR_NEFLAG(huart); __HAL_UART_CLEAR_FEFLAG(huart); } }
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) { if (htim->Instance == TIM3) mpu_timer_flag = 1; }

// ==========================================================
// 核心回调：只处理 Callback，不写 ISR (防止重定义)
// ==========================================================
extern volatile uint8_t SpiTxCplt; 
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        SpiTxCplt = 1; // 告知 user_diskio DMA 完成
    }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
