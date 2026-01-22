/* USER CODE BEGIN Header */
/**
  * 最终极简版 (Combined)
  * * 前置条件 (CubeMX配置)：
  * 1. USART1 DMA RX -> Mode: Circular (必须是循环模式！！)
  * 2. USART2 Baud Rate -> 921600 (波特率要够快)
  * * 逻辑：
  * - 收到 '1': 开启 混合传输 (BMD转发 + IMU采集发送)
  * - 收到 '3': 停止
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

/* USER CODE BEGIN PV */
// --- MPU6050 (IMU) ---
#define MPU_SAMPLES     50
#define IMU_TOTAL_SIZE  (MPU_SAMPLES * 16) 

typedef struct {
    int16_t Accel_X; int16_t Accel_Y; int16_t Accel_Z;
    int16_t Temp;
    int16_t Gyro_X;  int16_t Gyro_Y;  int16_t Gyro_Z;
} MPU6050_Data_t;

MPU6050_Data_t MpuBuffer[MPU_SAMPLES];
uint8_t MpuSendPacket[IMU_TOTAL_SIZE]; 

// --- BMD101 (环形缓冲) ---
// 只要是 Circular 模式，这个 buffer 就会像传送带一样永远转下去
#define RX_BUFFER_SIZE  4096 
uint8_t RxRingBuffer[RX_BUFFER_SIZE]; 

// 指针
uint32_t write_ptr = 0; // DMA 当前写到哪了
uint32_t read_ptr = 0;  // 我们发送到哪了

// 控制
volatile uint8_t run_mode = 0; // 0=Stop, 1=Start
uint8_t rx_cmd_byte;

// MPU 变量
uint8_t mpu_index = 0;
volatile uint8_t mpu_timer_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN 0 */
// 极简发送函数：直接调用 HAL 阻塞发送，依靠内部超时防止死锁
// 921600波特率下，发送几个字节只需要微秒级，不会卡顿
void Uart_Send(uint8_t* data, uint16_t len) {
    if (len == 0) return;
    HAL_UART_Transmit(&huart2, data, len, 100); 
}

void MPU6050_Init(void) {
    uint8_t Data = 0;
    // 不管初始化成不成功，都不要卡死在这里
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

  /* USER CODE BEGIN 2 */
  HAL_Delay(500); // 等待上电稳定
  
  // 1. 打印个招呼，确保串口是通的
  Uart_Send((uint8_t*)"\r\n=== SYSTEM READY ===\r\n", 22);

  MPU6050_Init();
  
  // 2. 开启指令接收
  HAL_UART_Receive_IT(&huart2, &rx_cmd_byte, 1);
  // 3. 开启定时器
  HAL_TIM_Base_Start_IT(&htim3);
  
  // 4. 开启 BMD101 接收 (Circular模式下，它会一直在后台跑，永不停止)
  // 清除标志位，防止一上来就报错
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  HAL_UART_Receive_DMA(&huart1, RxRingBuffer, RX_BUFFER_SIZE);
  /* USER CODE END 2 */

  while (1)
  {
    // =========================================================
    // 逻辑：run_mode = 1 时，同时处理 BMD 转发 和 IMU 采集
    // =========================================================
    if (run_mode == 1) {
        
        // --- 任务A: BMD101 实时转发 (防掉帧核心) ---
        
        // 1. 获取 DMA 硬件写到了哪里
        // 转换公式：Buffer总大小 - DMA剩余计数
        write_ptr = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
        
        // 2. 如果有新数据 (写指针 != 读指针)
        if (write_ptr != read_ptr) {
            
            // 情况1: 正常追赶 (Write > Read)
            if (write_ptr > read_ptr) {
                Uart_Send(&RxRingBuffer[read_ptr], write_ptr - read_ptr);
                read_ptr = write_ptr;
            }
            // 情况2: 发生回卷 (Write < Read)
            else {
                // 先把尾巴发完
                Uart_Send(&RxRingBuffer[read_ptr], RX_BUFFER_SIZE - read_ptr);
                // 再把头部的发完
                if (write_ptr > 0) {
                    Uart_Send(&RxRingBuffer[0], write_ptr);
                }
                read_ptr = write_ptr;
            }
        }

        // --- 任务B: IMU 采集与发送 ---
        if (mpu_timer_flag) {
            mpu_timer_flag = 0;
            
            // 采集一个点
            if (MPU6050_Read(&MpuBuffer[mpu_index]) == 1) {
                mpu_index++;
            }
            
            // 攒够了50个点 (1秒)，直接发出去
            if (mpu_index >= MPU_SAMPLES) {
                mpu_index = 0;
                Package_IMU_Data();
                
                // 发送这800字节
                // 由于 Uart_Send 是阻塞的，它发完才会继续
                // 这期间 BMD 数据会被 DMA 自动存在 Ring Buffer 里，绝不会丢
                Uart_Send(MpuSendPacket, IMU_TOTAL_SIZE);
            }
        }
    } 
    else {
        // 如果是停止模式，我们需要同步指针，防止数据堆积
        // 这样下次启动时，从最新的数据开始发，而不是发旧数据
        write_ptr = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
        read_ptr = write_ptr;
        mpu_index = 0;
    }
  }
}

/* USER CODE BEGIN 4 */
// 极简回调：只处理模式切换
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (rx_cmd_byte == '1') {
            run_mode = 1;
            // 不需要打印 "Start"，防止干扰波形
        }
        else if (rx_cmd_byte == '3') {
            run_mode = 0;
        }
        HAL_UART_Receive_IT(&huart2, &rx_cmd_byte, 1);
    }
}

// 错误处理：防止 DMA 停止
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // 只清标志，Circular 模式下 DMA 通常会自动恢复
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        mpu_timer_flag = 1;
    }
}
/* USER CODE END 4 */

void Error_Handler(void) { __disable_irq(); while (1) {} }
void SystemClock_Config(void) {
  // 请保留 CubeMX 生成的时钟配置
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