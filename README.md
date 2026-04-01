# PPG & IMU Data Logger

本项目是一个基于 STM32F1 系列微控制器的多传感器数据采集系统，专门用于同步采集 **PPG (光电容积脉搏波)** 和 **IMU (惯性测量单元, MPU6050)** 数据，并将其高速、稳定地记录到 SD 卡中。

## 🌟 核心特性与功能逻辑

### 1. 独立双文件存储 (Dual-File Storage)
为了方便后期的数据分析与处理，系统将 PPG 数据和 IMU 数据拆分存储到两个独立的文件中。
- **PPG 数据**：存储于 `PPG.BIN`
- **IMU 数据**：存储于 `IMU.BIN`
两类数据各自拥有独立的缓冲区和文件句柄（`fil_ppg` 和 `fil_imu`），在 SD 卡写入时交替进行，互不干扰。

### 2. Ping-Pong 缓冲 + DMA 架构 (Ping-Pong Buffer & DMA)
为了解决 SD 卡写入过程中的长延迟导致的“丢数”问题，系统采用了 **双缓冲 (Ping-Pong Buffer) 配合 DMA** 技术：
- **PPG 双缓冲**：`PpgDoubleBuf[2][512]`，每次累积 512 字节后触发一次 Block Write。
- **IMU 双缓冲**：`ImuDoubleBuf[2][800]`，每次累积 800 字节后触发一次 Block Write。
- **逻辑机制**：当缓冲池 A 正在被 FatFs（文件系统）写入 SD 卡时，新的传感器数据会被 DMA 自动存入缓冲池 B 中。当缓冲池 B 满时，两者角色互换。这种架构完美掩盖了 SD 卡的写入延迟。

### 3. 蓝牙控制与状态机 (Bluetooth Control & State Machine)
系统通过蓝牙模块（USART2）进行无线控制，内部实现了一个由 `sys_state` 变量驱动的稳健状态机：
- **`sys_state == 0` (Idle/停止)**：系统处于待机状态，不记录任何数据。蓝牙发送 `start` 指令可唤醒系统，进入状态 1。蓝牙发送 `reset` 可复位重启 MCU。
- **`sys_state == 1` (Wait for Name/等待命名)**：系统通过串口向外发送 `[CMD] Name?`，等待用户通过蓝牙发送本次采集的被试者名称或标识（例如 `Tom`）。
- **`sys_state == 2` (Recording/收集中)**：收到名称后，系统自动创建文件夹并开始将数据写入 SD 卡。发送 `stop` 指令即可结束本次采集并安全关闭文件，回到状态 0。

### 4. 智能序列化文件夹命名 (Smart Folder Management)
为了防止多次采集导致文件被覆盖或混淆，系统引入了 `session_id` 的自增机制结合蓝牙指令命名：
- 当在 `sys_state == 1` 状态下收到字符串（如 `ZhangSan`）时，系统会自动将当前的序号与字符串拼接。
- 自动在 SD 卡根目录创建类似 `001_ZhangSan` 的文件夹。
- 在该文件夹内部，生成对应的 `PPG.BIN` 和 `IMU.BIN`。
- 完成一次采集后，`session_id` 自动 `+1`。下一次发送 `LiSi` 时，会自动生成 `002_LiSi` 文件夹，以此类推。

### 5. SD卡空间预分配 (SD Card Pre-allocation) - 可选
FatFs 文件系统在文件不断增长时频繁分配簇（Cluster）会增加耗时。系统中保留了 `f_expand` 函数的代码（按需开启），允许在创建文件时直接在 SD 卡上预先分配出 10MB 的连续空间，从而把写卡延迟降到最低，实现“终极防卡顿”。

## 🛠 硬件接口分配

- **USART1**: PPG 传感器数据输入 (RX) + DMA
- **USART2**: 蓝牙模块通信 (RX/TX)，波特率依配置而定，用于接收控制指令 (`start`, `stop`, `reset`, `[文件名]`)
- **I2C2**: MPU6050 传感器通信 (SCL, SDA)
- **SPI1**: SD 卡读写模块 (SCK, MISO, MOSI, CS)
- **TIM3**: 触发 IMU (MPU6050) 的定时读取，默认采样率 50Hz。

## 🚀 如何使用

1. **上电准备**：确保 SD 卡已插入且格式化为 FAT32，蓝牙模块和传感器连接正常。
2. **连接蓝牙**：使用手机蓝牙串口助手或 PC 串口终端连接设备的蓝牙。
3. **开始任务**：
   - 发送指令 `start`。
   - 收到单片机回复 `[CMD] Name?`。
   - 发送当前被试者的名称（如 `Walking_Test`）。
   - 单片机会回复 `Creating Directory: 001_Walking_Test` 和 `Files Created Successfully!`，采集正式开始。
4. **停止任务**：
   - 发送指令 `stop`。
   - 单片机将缓存中剩余的数据刷入 SD 卡，并回复 `CLOSED.`。此时 SD 卡可以安全拔出。
5. **重置系统**：
   - 发送指令 `reset`，单片机将执行软件复位。

## 📁 生成文件结构示例

```text
SD_CARD_ROOT/
├── BOOT_TST.TXT             # 开机自检 DMA 测试文件
├── 001_Tom/
│   ├── PPG.BIN              # 包含第一批次的 PPG 原始数据
│   └── IMU.BIN              # 包含第一批次的 IMU 数据 (AA 55 帧头)
├── 002_Jerry/
│   ├── PPG.BIN
│   └── IMU.BIN
└── 003_Walking_Test/
    ├── PPG.BIN
    └── IMU.BIN
```

## 📜 历史版本与原代码说明
在主代码（`main.c`）中，为了追溯之前的单文件存储逻辑，我们使用了 `/* 原代码保留： */` 的注释块保留了历史代码。带有 `HuixinUpdate` 前缀的注释表示最近为了双文件和文件夹自动编号而进行的优化改动。
