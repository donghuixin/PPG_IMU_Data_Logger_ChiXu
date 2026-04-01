/* USER CODE BEGIN Header */
/**
  * ˫���� (Ping-Pong) + DMA ����д��� main.c
  * �����߼���
  * 1. ���� DoubleBuf[2][512]��
  * 2. ����2��ʲô�����Ǿ��� DoubleBuf ���ʲô��
  * 3. �չ� 512 �ֽڣ�����һ�� Block Write (���õײ� DMA)��
  * 
  * Double Buffer (Ping-Pong) + DMA writing main.c
  * Logic:
  * 1. Define DoubleBuf[2][512]
  * 2. Receive data via USART2 and store it in DoubleBuf
  * 3. When 512 bytes are collected, trigger a Block Write (using underlying DMA)
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
// --- MPU6050 / MPU9250 ---
#define MPU_SAMPLES     50
// һ֡ = AA 55(2) + ���ٶ�(6) + ������(6) + \r \n(2) = 16 �ֽ�
// 1 Frame = AA 55(2) + Accel(6) + Gyro(6) + \r \n(2) = 16 Bytes
#define IMU_TOTAL_SIZE  (MPU_SAMPLES * 16) 

typedef struct __attribute__((packed)) {
    int16_t Accel_X; 
    int16_t Accel_Y; 
    int16_t Accel_Z;
    // Temp �ѳ���ɾ��
    // Temp has been removed
    int16_t Gyro_X;  
    int16_t Gyro_Y;  
    int16_t Gyro_Z;
} MPU6050_Data_t; // ռ�� 12 �ֽ�
                  // Takes 12 Bytes

MPU6050_Data_t MpuBuffer[MPU_SAMPLES];
uint8_t MpuSendPacket[IMU_TOTAL_SIZE];

// --- UART ���ջ��� (2KB) ---
// --- UART Receive Buffer (2KB) ---
#define RX_BUFFER_SIZE  2048 
uint8_t RxRingBuffer[RX_BUFFER_SIZE]; 
uint32_t write_ptr = 0; 
uint32_t read_ptr = 0;  

// --- ��˫������Ķ��塿 ---
// --- Double Buffer Definition ---
// HuixinUpdate: 拆分 PPG 和 IMU 的双缓冲定义，以支持双文件独立写入
// HuixinUpdate: Split double buffer for PPG and IMU to support dual-file independent writing
/* 原代码保留：
#define BLOCK_SIZE 800
uint8_t DoubleBuf[2][BLOCK_SIZE]; // ���� 512 �ֽڵ�Ͱ (Ping-Pong)
uint8_t buf_fill_idx = 0;         // ��ǰ�������ĸ�Ͱ (0 �� 1)
uint16_t buf_pos = 0;             // ��ǰͰ���˶���

// --- д�������� ---
// 0xFF: ������
// 0: Ͱ0���ˣ���дͰ0
// 1: Ͱ1���ˣ���дͰ1
volatile uint8_t ready_to_write_idx = 0xFF; 
*/

#define PPG_BLOCK_SIZE 512
uint8_t PpgDoubleBuf[2][PPG_BLOCK_SIZE]; 
uint8_t ppg_buf_fill_idx = 0;         
uint16_t ppg_buf_pos = 0;             
volatile uint8_t ppg_ready_to_write_idx = 0xFF; 

#define IMU_BLOCK_SIZE 800
uint8_t ImuDoubleBuf[2][IMU_BLOCK_SIZE]; 
uint8_t imu_buf_fill_idx = 0;         
uint16_t imu_buf_pos = 0;             
volatile uint8_t imu_ready_to_write_idx = 0xFF; 

// --- �ļ�ϵͳ ---
// --- File System ---
FATFS fs;
// HuixinUpdate: 定义两个独立的文件对象，分别存储 PPG 和 IMU 数据
// HuixinUpdate: Define two independent file objects to store PPG and IMU data separately
/* 原代码保留：
FIL fil;
*/
FIL fil_ppg;
FIL fil_imu;
FRESULT fres;
uint32_t bytes_written;
uint8_t is_sd_ready = 0;
// HuixinUpdate: 拆分文件打开状态标志
/* 原代码保留：
uint8_t is_file_open = 0; 
*/
uint8_t is_ppg_file_open = 0; 
uint8_t is_imu_file_open = 0; 
volatile uint8_t sys_state = 0; 

// --- ����ָ�� ---
// --- Receive Command ---
uint8_t rx_byte_temp;          
#define CMD_MAX_LEN 32         
char rx_cmd_buffer[CMD_MAX_LEN]; 
uint8_t rx_cmd_index = 0;      

uint8_t mpu_index = 0;
volatile uint8_t mpu_timer_flag = 0;

// HuixinUpdate: 记录采集批次，用于自动生成序号文件夹
// HuixinUpdate: Record collection batch, used to automatically generate sequential folders
uint16_t session_id = 1; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN 0 */
void Uart_Print(char* str) {
    if(huart2.Instance != NULL) HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 100);
}

// -------------------------------------------------------------
// HuixinUpdate: 处理 PPG 数据，写入 PPG 专用的双缓冲区
// HuixinUpdate: Process PPG data and write to dedicated PPG double buffer
// -------------------------------------------------------------
/* 原代码保留：
// -------------------------------------------------------------
// ���ݷַ��� (������)
// -------------------------------------------------------------
void Data_Distribute(uint8_t* data, uint16_t len) {
    if (len == 0) return;
    
    // 1. ���ڻ��� (��Ҫ�Ĺ��ܣ����͸�����2)
//    HAL_UART_Transmit(&huart2, data, len, 10); 
    
    // 2. ˫������� (���� SD ��)
    if (sys_state == 2 && is_file_open && is_sd_ready) {
        
        for (int i = 0; i < len; i++) {
            // ���뵱ǰ���ڹ�����Ͱ
            DoubleBuf[buf_fill_idx][buf_pos++] = data[i];
            
            // ���Ͱ���� (512�ֽ�)
            if (buf_pos >= BLOCK_SIZE) {
                // A. �������񣺱�����Ͱ��Ҫд��
                ready_to_write_idx = buf_fill_idx;
                
                // B. �л�����һ��Ͱ (0->1, 1->0)
                buf_fill_idx = !buf_fill_idx; 
                
                // C. ������Ͱ��ָ��
                buf_pos = 0;
                
                // D. ���Դ�ӡ (��ѡ��֤��������)
                // Uart_Print("[TRIG]"); 
            }
        }
    }
}
*/

void PPG_Data_Distribute(uint8_t* data, uint16_t len) {
    if (len == 0) return;
    
    if (sys_state == 2 && is_ppg_file_open && is_sd_ready) {
        for (int i = 0; i < len; i++) {
            PpgDoubleBuf[ppg_buf_fill_idx][ppg_buf_pos++] = data[i];
            
            if (ppg_buf_pos >= PPG_BLOCK_SIZE) {
                ppg_ready_to_write_idx = ppg_buf_fill_idx;
                ppg_buf_fill_idx = !ppg_buf_fill_idx; 
                ppg_buf_pos = 0;
            }
        }
    }
}

// -------------------------------------------------------------
// HuixinUpdate: 处理 IMU 数据，写入 IMU 专用的双缓冲区
// HuixinUpdate: Process IMU data and write to dedicated IMU double buffer
// -------------------------------------------------------------
void IMU_Data_Distribute(uint8_t* data, uint16_t len) {
    if (len == 0) return;
    
    if (sys_state == 2 && is_imu_file_open && is_sd_ready) {
        for (int i = 0; i < len; i++) {
            ImuDoubleBuf[imu_buf_fill_idx][imu_buf_pos++] = data[i];
            
            if (imu_buf_pos >= IMU_BLOCK_SIZE) {
                imu_ready_to_write_idx = imu_buf_fill_idx;
                imu_buf_fill_idx = !imu_buf_fill_idx; 
                imu_buf_pos = 0;
            }
        }
    }
}

// -------------------------------------------------------------
// SD ��д������ (������) - ����ѭ������
// SD card write thread (non-blocking) - put in main loop
// HuixinUpdate: 拆分 SD 卡写入线程，分别检查 PPG 和 IMU 的缓冲区并写入
// HuixinUpdate: Split SD card write thread, check and write PPG and IMU buffers separately
// -------------------------------------------------------------
/* 原代码保留：
// -------------------------------------------------------------
// SD ��д������ (������) - ����ѭ������
// -------------------------------------------------------------
void SD_Write_Thread(void) {
    // ����Ƿ��С���Ͱ����Ҫд��
    if (ready_to_write_idx != 0xFF) {
        
        if (sys_state == 2 && is_file_open && is_sd_ready) {
            
            // ����Ҫд�� Buffer
            uint8_t* pBuf = DoubleBuf[ready_to_write_idx];
            
            // ֱ�ӵ��� f_write д�� 512 �ֽ�
            // ��Ϊ��С������ 512��user_diskio ���Զ����� DMA
            fres = f_write(&fil, pBuf, BLOCK_SIZE, &bytes_written);
            
            if (fres == FR_OK) {
                Uart_Print("."); // ��ӡһ�����ʾд��ɹ�
            } else {
                char msg[32]; sprintf(msg, "[Err:%d]", fres); Uart_Print(msg);
            }
        }
        
        // ���������
        ready_to_write_idx = 0xFF;
    }
}
*/

void SD_Write_Thread(void) {
    // 1. 写 PPG 数据
    // 1. Write PPG Data
    if (ppg_ready_to_write_idx != 0xFF) {
        if (sys_state == 2 && is_ppg_file_open && is_sd_ready) {
            uint8_t* pBuf = PpgDoubleBuf[ppg_ready_to_write_idx];
            fres = f_write(&fil_ppg, pBuf, PPG_BLOCK_SIZE, &bytes_written);
            
            if (fres == FR_OK) {
                // HuixinUpdate: 每次写入后立即同步，防止断电导致数据丢失
                // HuixinUpdate: Sync immediately after each write to prevent data loss on power failure
                f_sync(&fil_ppg);
            } else {
                char msg[32]; sprintf(msg, "[P_Err:%d]", fres); Uart_Print(msg);
            }
        }
        ppg_ready_to_write_idx = 0xFF;
    }
    
    // 2. 写 IMU 数据
    // 2. Write IMU Data
    if (imu_ready_to_write_idx != 0xFF) {
        if (sys_state == 2 && is_imu_file_open && is_sd_ready) {
            uint8_t* pBuf = ImuDoubleBuf[imu_ready_to_write_idx];
            fres = f_write(&fil_imu, pBuf, IMU_BLOCK_SIZE, &bytes_written);
            
            if (fres == FR_OK) {
                // HuixinUpdate: 每次写入后立即同步，防止断电导致数据丢失
                // HuixinUpdate: Sync immediately after each write to prevent data loss on power failure
                f_sync(&fil_imu);
            } else {
                char msg[32]; sprintf(msg, "[I_Err:%d]", fres); Uart_Print(msg);
            }
        }
        imu_ready_to_write_idx = 0xFF;
    }
}

void MPU6050_Init(void) {
    // ================= ���������޸����Ｔ�ɸı䴫�������� =================
    // ================= User can modify these parameters to change sensor characteristics =================
    
    uint8_t SMPLRT_DIV   = 19;      // ������ = 1000 / (1 + 19) = 50Hz (ʮ����д19��ֱ��)
                                    // Sample Rate = 1000 / (1 + 19) = 50Hz (decimal 19)
    uint8_t DLPF_CFG     = 0x03;    // ��ͨ�˲� (Լ42Hz����)
                                    // Low Pass Filter (approx 42Hz bandwidth)
    
    // ����������: 0x00(��250��/s), 0x08(��500��/s), 0x10(��1000��/s), 0x18(��2000��/s)
    // Gyro Range: 0x00(±250°/s), 0x08(±500°/s), 0x10(±1000°/s), 0x18(±2000°/s)
    uint8_t GYRO_RANGE   = 0x18;    
    
    // ���ٶȼ�����: 0x00(��2g), 0x08(��4g), 0x10(��8g), 0x18(��16g)
    // Accel Range: 0x00(±2g), 0x08(±4g), 0x10(±8g), 0x18(±16g)
    uint8_t ACCEL_RANGE  = 0x18;    
    
    // =====================================================================

    uint8_t check;
    uint8_t data;

    // 1. ȷ���豸�Ƿ����
    // 1. Check if device is present
    HAL_I2C_Mem_Read(&hi2c2, 0xD0, 0x75, 1, &check, 1, 100);
    if (check == 0x68) {
        
        // 2. ���ѵ�Դ
        // 2. Wake up sensor
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c2, 0xD0, 0x6B, 1, &data, 1, 100);
        
        // ��Ҫ����Ӳ��һ��ʱ���˯��������
        // Need to give hardware some time to wake up
        HAL_Delay(10); 

        // 3. ���ò�����
        // 3. Set sample rate
        HAL_I2C_Mem_Write(&hi2c2, 0xD0, 0x19, 1, &SMPLRT_DIV, 1, 100);

        // 4. ���õ�ͨ�˲�
        // 4. Set low pass filter
        HAL_I2C_Mem_Write(&hi2c2, 0xD0, 0x1A, 1, &DLPF_CFG, 1, 100);

        // 5. ��������������
        // 5. Set gyro range
        HAL_I2C_Mem_Write(&hi2c2, 0xD0, 0x1B, 1, &GYRO_RANGE, 1, 100);

        // 6. ���ü��ٶȼ�����
        // 6. Set accel range
        HAL_I2C_Mem_Write(&hi2c2, 0xD0, 0x1C, 1, &ACCEL_RANGE, 1, 100);
    }
}

uint8_t MPU6050_Read(MPU6050_Data_t *data) {
    uint8_t Rec_Data[14];
    // ��ȡ 14 �ֽڣ�Accel(6) + Temp(2) + Gyro(6)
    // Read 14 bytes: Accel(6) + Temp(2) + Gyro(6)
    if(HAL_I2C_Mem_Read(&hi2c2, 0xD0, 0x3B, 1, Rec_Data, 14, 2) == HAL_OK) {
        uint8_t *pOut = (uint8_t*)data;
        
        // �������ٶ� (ֱ�ӿ�����������ˣ����� Excel �� HxD �鿴)
        // Copy Accel (Direct copy for faster performance, parsing in Excel or HxD)
        pOut[0] = Rec_Data[0]; pOut[1] = Rec_Data[1]; 
        pOut[2] = Rec_Data[2]; pOut[3] = Rec_Data[3]; 
        pOut[4] = Rec_Data[4]; pOut[5] = Rec_Data[5]; 
        
        // ���� Rec_Data[6] �� [7] (�¶�����)
        // Skip Rec_Data[6] and [7] (Temperature Data)
        
        // ���������� 
        // Copy Gyro
        pOut[6] = Rec_Data[8];  pOut[7] = Rec_Data[9];  
        pOut[8] = Rec_Data[10]; pOut[9] = Rec_Data[11]; 
        pOut[10] = Rec_Data[12]; pOut[11] = Rec_Data[13]; 
        
        return 1; 
    }
    return 0; 
}

void Package_IMU_Data(void) {
    for (int i = 0; i < MPU_SAMPLES; i++) {
        uint16_t offset = i * 16; // ÿ֡ 16 �ֽ�
                                  // Each frame is 16 bytes
        
        // 1. ��ͷ
        // 1. Frame Header
        MpuSendPacket[offset]     = 0xAA;
        MpuSendPacket[offset + 1] = 0x55;
        
        // 2. ���� (12�ֽ�)
        // 2. Data (12 bytes)
        memcpy(&MpuSendPacket[offset + 2], &MpuBuffer[i], 12);
        
        // 3. ��β���������з�
        // 3. Frame Tail (Carriage Return & Line Feed)
        MpuSendPacket[offset + 14] = 0x0D; // \r (�س�)
        MpuSendPacket[offset + 15] = 0x0A; // \n (����)
    }
}

/* 原代码保留：
void Create_New_File(char* filename) {
    char full_name[256];
    if (strlen(filename) > 64) { Uart_Print("\r\n[ERR] Name too long.\r\n"); return; }
    
    // ʹ�� .BIN ��׺����Ϊ���Ǵ����ԭʼ����������
    sprintf(full_name, "%s.BIN", filename); 

    Uart_Print("\r\nCreating: "); Uart_Print(full_name); Uart_Print("...\r\n");
    fres = f_open(&fil, full_name, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres == FR_OK) {
        is_sd_ready = 1; is_file_open = 1; 
        
        // ����˫����״̬
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
*/

/* 原代码保留：
void Create_New_File(char* filename) {
    // HuixinUpdate: 修改为创建两个文件，并使用 f_expand 预分配空间（10MB）以解决交替写入造成的卡顿
    char name_ppg[256], name_imu[256];
    if (strlen(filename) > 64) { Uart_Print("\r\n[ERR] Name too long.\r\n"); return; }
    
    sprintf(name_ppg, "%s_PPG.BIN", filename); 
    sprintf(name_imu, "%s_IMU.BIN", filename); 

    Uart_Print("\r\nCreating PPG & IMU files...\r\n");
    
    FRESULT fres_ppg = f_open(&fil_ppg, name_ppg, FA_CREATE_ALWAYS | FA_WRITE);
    FRESULT fres_imu = f_open(&fil_imu, name_imu, FA_CREATE_ALWAYS | FA_WRITE);
    
    if (fres_ppg == FR_OK && fres_imu == FR_OK) {
        // HuixinUpdate: 预分配 10MB 空间 (需要 FatFs 配置 _USE_EXPAND == 1，如果不支持可忽略)
        // FRESULT res1 = f_expand(&fil_ppg, 10 * 1024 * 1024, 1);
        // FRESULT res2 = f_expand(&fil_imu, 10 * 1024 * 1024, 1);
        
        is_sd_ready = 1; 
        is_ppg_file_open = 1; 
        is_imu_file_open = 1;
        
        // HuixinUpdate: 重置双缓冲状态
        ppg_buf_fill_idx = 0;
        ppg_buf_pos = 0;
        ppg_ready_to_write_idx = 0xFF;
        
        imu_buf_fill_idx = 0;
        imu_buf_pos = 0;
        imu_ready_to_write_idx = 0xFF;
        
        sys_state = 2; 
        Uart_Print("Files Created & Pre-allocated!\r\n");
    } else {
        char msg[64]; sprintf(msg, "Err: PPG=%d, IMU=%d\r\n", fres_ppg, fres_imu); Uart_Print(msg);
        sys_state = 0; 
    }
}
*/

void Create_New_File(char* filename) {
    // HuixinUpdate: 以文件夹形式存储。每次采集生成独立的序号文件夹
    // HuixinUpdate: Store as folders. Generate independent sequential folder for each collection
    char dir_name[128];
    char path_ppg[256];
    char path_imu[256];
    
    if (strlen(filename) > 64) { Uart_Print("\r\n[ERR] Name too long.\r\n"); return; }
    
    // 1. 生成文件夹名，例如: "001_Data"
    // 1. Generate folder name, e.g., "001_Data"
    sprintf(dir_name, "%03d_%s", session_id, filename);
    session_id++; // 序号自增，为下一次采集做准备
                  // Increment sequence ID for the next collection
    
    Uart_Print("\r\nCreating Directory: "); 
    Uart_Print(dir_name); 
    Uart_Print("\r\n");
    
    // 2. 创建文件夹 (如果已经存在，f_mkdir会返回FR_EXIST，我们忽略这个错误继续创建文件)
    // 2. Create folder (if exists, f_mkdir returns FR_EXIST, ignore and continue)
    FRESULT res_dir = f_mkdir(dir_name);
    if (res_dir != FR_OK && res_dir != FR_EXIST) {
        char msg[64]; sprintf(msg, "[ERR] mkdir failed: %d\r\n", res_dir); Uart_Print(msg);
        sys_state = 0;
        return;
    }
    
    // 3. 拼接完整的文件路径 (放在刚才创建的文件夹内)
    // 3. Concatenate full file paths (inside the newly created folder)
    sprintf(path_ppg, "%s/PPG.BIN", dir_name); 
    sprintf(path_imu, "%s/IMU.BIN", dir_name); 

    Uart_Print("Creating PPG & IMU files inside dir...\r\n");
    
    // 4. 创建并打开文件
    // 4. Create and open files
    FRESULT fres_ppg = f_open(&fil_ppg, path_ppg, FA_CREATE_ALWAYS | FA_WRITE);
    FRESULT fres_imu = f_open(&fil_imu, path_imu, FA_CREATE_ALWAYS | FA_WRITE);
    
    if (fres_ppg == FR_OK && fres_imu == FR_OK) {
        // HuixinUpdate: 预分配 10MB 空间 (需要 FatFs 配置 _USE_EXPAND == 1，如果不支持可忽略)
        // HuixinUpdate: Pre-allocate 10MB space (Requires FatFs _USE_EXPAND == 1)
        // FRESULT res1 = f_expand(&fil_ppg, 10 * 1024 * 1024, 1);
        // FRESULT res2 = f_expand(&fil_imu, 10 * 1024 * 1024, 1);
        
        is_sd_ready = 1; 
        is_ppg_file_open = 1; 
        is_imu_file_open = 1;
        
        // 重置双缓冲状态
        // Reset double buffer state
        ppg_buf_fill_idx = 0;
        ppg_buf_pos = 0;
        ppg_ready_to_write_idx = 0xFF;
        
        imu_buf_fill_idx = 0;
        imu_buf_pos = 0;
        imu_ready_to_write_idx = 0xFF;
        
        sys_state = 2; 
        Uart_Print("Files Created Successfully!\r\n");
        Uart_Print("Start testing\r\n");
    } else {
        if (!is_sd_ready) {
            Uart_Print("\r\n[ERROR] Start timeout (15s): SD card not detected or mounted failed!\r\n");
            Ble_Print("\r\n[ERROR] SD Card NOT detected!\r\n"); // SD卡未插入
        } else {
            char msg[128]; 
            sprintf(msg, "\r\n[ERROR] Start timeout (15s): File creation failed! PPG_Err=%d, IMU_Err=%d\r\n", fres_ppg, fres_imu); 
            Uart_Print(msg);
            Ble_Print("\r\n[ERROR] File creation failed!\r\n"); // 文件创建失败
        }
        sys_state = 0; 
    }
}

// �ϵ��Լ캯�� (��Ҫ�Ĺ���)
// Boot test function (Check DMA functionality)
/* 原代码保留：
void Boot_Test(void) {
    Uart_Print("\r\n>>> Boot Test (DMA Check) <<<\r\n");
    fres = f_open(&fil, "BOOT_TST.TXT", FA_CREATE_ALWAYS | FA_WRITE);
    if(fres == FR_OK) {
        // ���� DoubleBuf[0] ������
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
*/
void Boot_Test(void) {
    Uart_Print("\r\n>>> Boot Test (DMA Check) <<<\r\n");
    // HuixinUpdate: Boot_Test 中临时使用 fil_ppg 进行测试
    fres = f_open(&fil_ppg, "BOOT_TST.TXT", FA_CREATE_ALWAYS | FA_WRITE);
    if(fres == FR_OK) {
        // ���� PpgDoubleBuf[0] ������
        memset(PpgDoubleBuf[0], 'A', 512); 
        fres = f_write(&fil_ppg, PpgDoubleBuf[0], 512, &bytes_written);
        f_close(&fil_ppg);
        if(fres == FR_OK && bytes_written == 512) Uart_Print("Test Write OK!\r\n");
        else Uart_Print("Test Write Fail!\r\n");
    } else {
        Uart_Print("Test Open Fail!\r\n");
    }
    Uart_Print(">>> End Test <<<\r\n\r\n");
}

// ====================================================================
// HuixinUpdate: 新增蓝牙专属发送函数，解决指令回传网页收不到的问题
// ====================================================================
void Ble_Print(char* str) {
    // 蓝牙模块连接在 USART2 上
    HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 100);
}
// ====================================================================

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

  // ���� DMA �ж� (���룡)
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
      
      // �ϵ��Լ�
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
  static uint32_t start_cmd_tick = 0;
  /* USER CODE END 1 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    
    // 超时检测: 当处于等待命名状态(sys_state==1)时，如果超过15秒没进入 sys_state==2，则报错
    if (sys_state == 1) {
        if (start_cmd_tick == 0) {
            start_cmd_tick = HAL_GetTick();
        } else if (HAL_GetTick() - start_cmd_tick > 5000) { // 5 seconds timeout
            Uart_Print("\r\n[ERROR] Start command timeout (5s). Did not receive file name.\r\n");
            sys_state = 0;
            start_cmd_tick = 0; // Reset tick
        }
    } else {
        start_cmd_tick = 0; // Reset tick if not in state 1
    }
    
    // ����
    // Main Loop


    // 1. д���߳� (Ping-Pong ����)
    // 1. Write Thread (Ping-Pong processing)
    SD_Write_Thread();

    // 2. ���ݲɼ��߳�
    // 2. Data Collection Thread
    if (sys_state == 0) { 
        if (is_dma_running == 0) {
					    if (HAL_GetTick() - heartbeat_tick > 5000) {
            heartbeat_tick = HAL_GetTick();
            Uart_Print("===CMD LIST===\r\n");				
						Uart_Print("START:START THE TASK\r\n");	
						Uart_Print("STOP:STOP THE TASK\r\n");	
						Uart_Print("RESET:RESET THE SYSTEM\r\n");									
    }}
        }		
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
            /* 原代码保留：
            if (write_ptr > read_ptr) { 
                Data_Distribute(&RxRingBuffer[read_ptr], write_ptr - read_ptr); 
                read_ptr = write_ptr; 
            } else { 
                Data_Distribute(&RxRingBuffer[read_ptr], RX_BUFFER_SIZE - read_ptr); 
                if (write_ptr > 0) Data_Distribute(&RxRingBuffer[0], write_ptr); 
                read_ptr = write_ptr; 
            }
            */
            if (write_ptr > read_ptr) { 
                // HuixinUpdate: 替换为专门的 PPG 分发函数
                // HuixinUpdate: Replace with dedicated PPG distribution function
                PPG_Data_Distribute(&RxRingBuffer[read_ptr], write_ptr - read_ptr); 
                read_ptr = write_ptr; 
            } else { 
                PPG_Data_Distribute(&RxRingBuffer[read_ptr], RX_BUFFER_SIZE - read_ptr); 
                if (write_ptr > 0) PPG_Data_Distribute(&RxRingBuffer[0], write_ptr); 
                read_ptr = write_ptr; 
            }
        }
        
        if (mpu_timer_flag) {
            mpu_timer_flag = 0;
            if (MPU6050_Read(&MpuBuffer[mpu_index]) == 1) mpu_index++;
            if (mpu_index >= MPU_SAMPLES) { 
                mpu_index = 0; 
                Package_IMU_Data(); 
                /* 原代码保留：
                Data_Distribute(MpuSendPacket, IMU_TOTAL_SIZE); 
                */
                // HuixinUpdate: 替换为专门的 IMU 分发函数
                // HuixinUpdate: Replace with dedicated IMU distribution function
                IMU_Data_Distribute(MpuSendPacket, IMU_TOTAL_SIZE); 
            }
        }
    } else { 
        if (is_dma_running == 1) {
            HAL_UART_DMAStop(&huart1); is_dma_running = 0;
            // ������������
            // Process remaining data
            uint32_t last_pos = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            /* 原代码保留：
            if (last_pos > read_ptr) Data_Distribute(&RxRingBuffer[read_ptr], last_pos - read_ptr);
            
            // �����һ������512�ֽڵ�BufferҲд��
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
            */
            // HuixinUpdate: 停止时处理残留 PPG 数据
            // HuixinUpdate: Process remaining PPG data when stopped
            if (last_pos > read_ptr) PPG_Data_Distribute(&RxRingBuffer[read_ptr], last_pos - read_ptr);
            
            // HuixinUpdate: 写入最后不满一块的残留数据
            // HuixinUpdate: Write remaining data that does not fill a whole block
            if (ppg_buf_pos > 0) {
                 f_write(&fil_ppg, PpgDoubleBuf[ppg_buf_fill_idx], ppg_buf_pos, &bytes_written);
            }
            if (imu_buf_pos > 0) {
                 f_write(&fil_imu, ImuDoubleBuf[imu_buf_fill_idx], imu_buf_pos, &bytes_written);
            }

            if(is_ppg_file_open || is_imu_file_open) {
                Uart_Print("\r\nClosing...\r\n");
                FRESULT f_ppg = f_close(&fil_ppg); 
                FRESULT f_imu = f_close(&fil_imu); 
                if(f_ppg == FR_OK && f_imu == FR_OK) Uart_Print("CLOSED.\r\n");
                else { char msg[32]; sprintf(msg, "Err: P%d, I%d\r\n", f_ppg, f_imu); Uart_Print(msg); }
                is_ppg_file_open = 0; 
                is_imu_file_open = 0; 
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
                if (strstr(rx_cmd_buffer, "stop") != NULL) { 
                    sys_state = 0; 
                    Uart_Print("\r\n[CMD] Stopped\r\n");
                    Ble_Print("\r\n[CMD] Stopped\r\n"); // Notify webpage that it has stopped
                }
                else if (strstr(rx_cmd_buffer, "reset") != NULL) { 
                    Uart_Print("\r\n[CMD] Resetting...\r\n");
                    Ble_Print("\r\n[CMD] Resetting...\r\n"); // Notify webpage that it is resetting
                    NVIC_SystemReset(); 
                }
                else if (sys_state == 0 && strstr(rx_cmd_buffer, "start") != NULL) { 
                    sys_state = 1; 
                    Uart_Print("\r\n[CMD] Name?\r\n"); 
                    Ble_Print("\r\n[CMD] Name?\r\n"); // HuixinUpdate: Send to webpage via Bluetooth simultaneously
                }
                else if (sys_state == 1 && strstr(rx_cmd_buffer, "start") == NULL) { 
                    Create_New_File(rx_cmd_buffer); 
                    Ble_Print("\r\n[CMD] Files Created\r\n"); // Notify webpage that file creation is successful
                }
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
// ���Ļص���ֻ���� Callback����д ISR (��ֹ�ض���)
// System callbacks should only call Callback, not write ISR (prevent redefinition)
// ==========================================================
extern volatile uint8_t SpiTxCplt; 
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        SpiTxCplt = 1; // ��֪ user_diskio DMA ���
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
