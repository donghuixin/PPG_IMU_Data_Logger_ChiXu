/* USER CODE BEGIN Header */
/**
  * 工业级防掉帧版 (Ring Buffer Strategy)
  * 核心原理：
  * 1. UART1 DMA 开启 Circular 模式，永不停止，像录像带一样循环录制。
  * 2. 主循环检测 DMA 指针位置，只要有新数据，立刻通过 UART2 转发。
  * 3. 彻底消除 "等待满一包才发" 造成的阻塞和溢出风险。
  */
/* USER CODE END Header */
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>

// ================= 定义区域 =================

// --- MPU6050 (IMU) ---
#define MPU_SAMPLES  50
#define IMU_TOTAL_SIZE  (MPU_SAMPLES * 16) 

typedef struct {
    int16_t Accel_X; int16_t Accel_Y; int16_t Accel_Z;
    int16_t Temp;
    int16_t Gyro_X;  int16_t Gyro_Y;  int16_t Gyro_Z;
} MPU6050_Data_t;

MPU6050_Data_t MpuBuffer[MPU_SAMPLES];
uint8_t MpuSendPacket[IMU_TOTAL_SIZE]; 

// --- BMD101 (关键修改) ---
// 缓冲区开大一点，用作环形缓冲
#define RX_BUFFER_SIZE  10240 
uint8_t RxRingBuffer[RX_BUFFER_SIZE]; 

// 环形缓冲指针
volatile uint32_t write_ptr_hw = 0; // DMA 硬件当前写到的位置
uint32_t read_ptr_sw = 0;           // 软件当前发到的位置

// --- 全局变量 ---
volatile uint8_t run_mode = 0;
uint8_t rx_cmd_byte;

// MPU 变量
uint8_t mpu_index = 0;
volatile uint8_t mpu_timer_flag = 0;

void SystemClock_Config(void);

// --- 辅助函数 ---

// 阻塞发送 (最稳，且因为是追赶式发送，每次发的数据量小，不会卡顿)
void Uart_Send(uint8_t* data, uint16_t len) {
    if (len == 0) return;
    HAL_UART_Transmit(&huart2, data, len, 100);
}

// MPU 初始化
void MPU6050_Init(void) {
    uint8_t Data = 0;
    if (HAL_I2C_IsDeviceReady(&hi2c2, 0xD0, 2, 100) == HAL_OK) {
        HAL_I2C_Mem_Write(&hi2c2, 0xD0, 0x6B, 1, &Data, 1, 100);
    }
}

// MPU 读取
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

// 打包 IMU
void Package_IMU_Data(void) {
    for (int i = 0; i < MPU_SAMPLES; i++) {
        uint16_t offset = i * 16;
        MpuSendPacket[offset]     = 0xAA;
        MpuSendPacket[offset + 1] = 0x55;
        memcpy(&MpuSendPacket[offset + 2], &MpuBuffer[i], 14);
    }
}

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

  /* USER CODE BEGIN 2 */
  HAL_Delay(200);
  Uart_Send((uint8_t*)"\r\n=== RING BUFFER SYSTEM READY！ ===\r\n", 34);

  MPU6050_Init();
  
  HAL_UART_Receive_IT(&huart2, &rx_cmd_byte, 1);
  HAL_TIM_Base_Start_IT(&htim3);
  
  // ==========================================================
  // 核心：启动 UART1 DMA Circular 模式
  // 哪怕 run_mode=0，它也在后台收数据，保证数据流不断
  // ==========================================================
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  HAL_UART_Receive_DMA(&huart1, RxRingBuffer, RX_BUFFER_SIZE);
  /* USER CODE END 2 */

  while (1)
  {
    // =========================================================
    // 任务1：BMD101 数据处理 (环形缓冲追赶逻辑)
    // =========================================================
    
    // 1. 获取 DMA 当前写到哪了 (硬件位置)
    // __HAL_DMA_GET_COUNTER 返回的是"倒计数"，所以要转换一下
    write_ptr_hw = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    
    // 2. 只有在模式 2 下，我们才转发数据
    if (run_mode == 2) {
        
        // 情况 A: 硬件写指针在软件读指针后面 (正常追赶)
        if (write_ptr_hw > read_ptr_sw) {
            uint16_t len = write_ptr_hw - read_ptr_sw;
            // 直接发送这一段新数据
            Uart_Send(&RxRingBuffer[read_ptr_sw], len);
            // 更新读指针
            read_ptr_sw = write_ptr_hw;
        }
        
        // 情况 B: 硬件写指针跑到了前面 (回卷了/Wrap Around)
        // 比如 Buffer大小10000，读到9900，写指针回到了 100
        else if (write_ptr_hw < read_ptr_sw) {
            // 第一步：先发 9900 到 10000 (尾部)
            uint16_t len_tail = RX_BUFFER_SIZE - read_ptr_sw;
            Uart_Send(&RxRingBuffer[read_ptr_sw], len_tail);
            
            // 第二步：再发 0 到 100 (头部)
            if (write_ptr_hw > 0) {
                Uart_Send(&RxRingBuffer[0], write_ptr_hw);
            }
            
            // 更新读指针
            read_ptr_sw = write_ptr_hw;
        }
    } 
    else {
        // 如果不是模式2，我们需要即使更新 read_ptr，假装我们读过了
        // 这样当你切回模式2时，不会把之前积压的几万字节旧数据一股脑发出来
        read_ptr_sw = write_ptr_hw;
    }

    // =========================================================
    // 任务2：IMU 数据处理 (模式 1)
    // =========================================================
    if (run_mode == 1) {
        if (mpu_timer_flag) {
            mpu_timer_flag = 0;
            if (MPU6050_Read(&MpuBuffer[mpu_index]) == 1) {
                mpu_index++;
            }
            if (mpu_index >= MPU_SAMPLES) {
                mpu_index = 0;
                Package_IMU_Data();
                Uart_Send(MpuSendPacket, IMU_TOTAL_SIZE);
            }
        }
    } else {
        mpu_index = 0; // 非IMU模式清零
    }
  }
}

// --- 中断回调 ---
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (rx_cmd_byte == '1') run_mode = 1;
        else if (rx_cmd_byte == '2') run_mode = 2;
        else if (rx_cmd_byte == '3') run_mode = 0;
        HAL_UART_Receive_IT(&huart2, &rx_cmd_byte, 1);
    }
    // 注意：这里不需要处理 UART1 的回调了，因为我们是主循环轮询指针
}

// 防止 ORE 错误导致 DMA 锁死 (这是掉帧的最大杀手)
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // 仅仅清除标志，绝不重启DMA，因为Circular模式下DMA会自动恢复
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) mpu_timer_flag = 1;
}

void Error_Handler(void) { __disable_irq(); while (1) {} }
void SystemClock_Config(void) {
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