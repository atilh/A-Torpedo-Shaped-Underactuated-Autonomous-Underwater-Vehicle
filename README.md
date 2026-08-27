# IMUDMA

基于 STM32H750 的水下航行器多传感器采集、组合导航与运动控制固件。工程使用 FreeRTOS 管理传感器读取、数据融合、上位机通信和执行机构控制任务，支持 IMU、DVL、GPS、深度传感器、USB 虚拟串口及多路 UART。

> 当前仓库主要保存嵌入式端源码与 Keil 工程。项目名称、设备用途和接口功能根据现有代码整理；接线和下水测试前，请以实际硬件原理图及传感器手册为准。

## 主要功能

- 读取并解析 MTi 系列 IMU 的加速度、自由加速度、角速度和欧拉角数据。
- 读取 DVL 的速度、位置、姿态等数据。
- 解析 GPS/RTK 定位与航向信息。
- 读取串口深度传感器数据。
- 使用 EKF 进行 IMU 与 DVL 的位置、速度融合。
- 使用 PID 和周期控制任务生成航向、深度及推进控制量。
- 通过 PWM 和串口向推进器、舵机或下级控制器输出指令。
- 通过 USB CDC 或 USART2 与上位机交换 ATKP 格式的数据包。
- 通过 SPI4 + DMA 驱动 WS2812 状态灯。

## 硬件与软件环境

| 项目 | 配置 |
| --- | --- |
| MCU | STM32H750VBT6，Cortex-M7，LQFP100 |
| 主频 | 480 MHz（HCLK 240 MHz） |
| 固件库 | STM32Cube FW_H7 V1.12.1 |
| RTOS | FreeRTOS Kernel V10.3.1，CMSIS-RTOS v2 接口 |
| USB | USB Device CDC FS（虚拟串口） |
| 工程工具 | Keil MDK-ARM；工程配置记录为 MDK-ARM V5.32 |
| 配置工具 | STM32CubeMX（`IMUDMA.ioc`） |

## 目录结构

```text
IM-7-3/
└─ IM/IMUDM/IMUDMA_New/
   ├─ Core/             # STM32 初始化、中断与 FreeRTOS 对象
   ├─ HARDWARE/         # IMU、深度传感器及公共数据类型
   ├─ DVL/              # DVL 数据接收与解析
   ├─ GPS/              # GPS/RTK 数据接收与解析
   ├─ EKF/              # 惯导参数、矩阵运算及融合任务
   ├─ PID/              # PID 算法与参数结构
   ├─ stabilizer/       # 稳定控制主循环
   ├─ moter/            # 推进器和舵机输出（原工程目录名）
   ├─ COMMUNICATE/      # ATKP 协议、USB 与 USART2 链路
   ├─ WS2812/           # WS2812 灯带驱动
   ├─ USB_DEVICE/       # USB CDC 设备代码
   ├─ Drivers/          # CMSIS 与 STM32 HAL
   ├─ Middlewares/      # FreeRTOS 与 USB Device 中间件
   ├─ MDK-ARM/          # Keil 工程和构建产物
   └─ IMUDMA.ioc        # STM32CubeMX 配置文件
```

## 接口分配

以下映射来自当前固件配置和业务代码。

| 外设 | 引脚 | 参数 | 当前用途 |
| --- | --- | --- | --- |
| UART4 | TX: PC10，RX: PA1 | 2,000,000，8N1 | IMU 数据 |
| UART8 | TX: PE1，RX: PE0 | 115,200，8N1 | 深度传感器 |
| USART1 | TX: PA9，RX: PA10 | 115,200，8N1 | DVL |
| USART2 | TX: PA2，RX: PA3 | 57,600，8N1 | 上位机/外部 ATKP 链路 |
| USART3 | TX: PB10，RX: PB11 | 115,200，8N1 | 舵机/下级控制器指令 |
| USART6 | TX: PC6，RX: PC7 | 115,200，8N1 | GPS/RTK |
| LPUART1 | TX: PB6，RX: PB7 | 209,700，8N1 | 调试输出与数据转发 |
| USB FS | PA11 / PA12 | CDC | 上位机虚拟串口 |
| SPI4 | 见 `IMUDMA.ioc` | 主机、单向发送、3.75 Mbit/s | WS2812（DMA） |
| TIM1 CH1–CH4 | PE9、PE11、PE13、PE14 | PWM | 已配置航向/深度输出，但当前启动路径未启用这些通道 |
| TIM8 CH3–CH4 | PC8、PC9 | PWM | CH3 用于推进输出；CH4 已启动并预留给深度输出 |

PWM 参数集中在 `moter/moter.h`：

- 航向输出范围：1000–2000 μs。
- 推进输出范围：1100–1900 μs。
- 深度输出范围：1200–1800 μs。

当前 `Control_PWM_Set()` 实际调用 `thrust_control()` 和串口形式的 `rudder_control()`；`Heading_control()`、`Depth_control()` 以及 TIM1 PWM 通道未接入当前主控制路径。如需切换为直接 PWM 驱动，须同时启用相应定时器通道并核对输出极性。

## 软件流程

1. `Core/Src/main.c` 初始化时钟、GPIO、DMA、UART、SPI 和定时器，然后启动 FreeRTOS。
2. 默认任务初始化 USB CDC，并启动 TIM17 运行时间计数。
3. IMU、DVL、GPS 和深度任务通过 UART 空闲中断/DMA 接收数据，并写入消息队列。
4. EKF 任务消费 IMU 与 DVL 数据，更新融合后的位置和速度。
5. `stabilizerTask` 以 1 kHz 主循环运行，当前在 500 Hz 节拍下刷新控制输出。
6. ATKP 发送任务周期上报传感器、位置、速度和控制量；接收任务解析上位机控制命令。

主要 FreeRTOS 任务包括：

| 任务 | 职责 |
| --- | --- |
| `IMUReadTask` | 接收并缓存 IMU 数据 |
| `DvlReadTask` | 接收、解析 DVL 数据 |
| `GpsReadTask` | 接收、解析 GPS/RTK 数据 |
| `DepthReadTask` | 接收深度数据 |
| `EKF_UpdataTask` / `EKF_FusionTask` | 惯导更新与多传感器融合 |
| `stabilizerTask` | PID/执行机构控制循环 |
| `atpkTxTask` / `atpkRxTask` | 上位机协议数据生成与命令处理 |
| `usblinktxTask` / `usblink_RxTask` | USB CDC 收发 |
| `Uart2linktxTask` / `Uart2link_RxTask` | USART2 收发 |
| `ledTask11` | WS2812 状态灯 |

## 编译与下载

### 使用 Keil MDK

1. 安装 Keil MDK-ARM，并确保已安装 STM32H7 Device Family Pack。
2. 打开：

   ```text
   IM/IMUDM/IMUDMA_New/MDK-ARM/IMUDMA.uvprojx
   ```

3. 在 Keil 中选择 `IMUDMA` Target。
4. 执行 Build/Rebuild。
5. 使用 ST-Link 连接目标板，确认芯片型号为 `STM32H750VBTx` 后下载固件。

工程已配置生成 HEX 文件。默认输出位置为：

```text
IM/IMUDM/IMUDMA_New/MDK-ARM/IMUDMA/IMUDMA.hex
```

### 使用 STM32CubeMX

可使用 `IMUDMA.ioc` 查看和调整引脚、时钟及外设配置。重新生成代码时应保留 `USER CODE BEGIN/END` 区域，并在提交前确认自定义目录仍被 Keil 工程包含。

## 首次运行

1. 断开推进器动力电源，先仅给控制板和传感器供电。
2. 检查所有 UART 电平、电源和公共地，确认引脚与波特率一致。
3. 编译并下载固件，复位控制板。
4. 等待系统启动；默认任务约 8 秒后进入常规循环。
5. 电脑端应识别到 `STM32 Virtual ComPort`，可通过 USB CDC 接收上报数据或发送 ATKP 控制帧。
6. 分别验证 IMU、DVL、GPS 和深度数据，再连接舵机与推进器动力。

## 通信协议

USB CDC 与 USART2 使用相同的 ATKP 风格帧：

```text
上行：0xAA 0xFF MSG_ID DATA_LEN DATA... CHECKSUM ADD_SUM
下行：0xAA 0xFF MSG_ID DATA_LEN DATA... CHECKSUM ADD_SUM
```

- 最大有效载荷：30 字节。
- `CHECKSUM` 为从帧头到数据区的逐字节累加和。
- `ADD_SUM` 为每一步 `CHECKSUM` 的再次累加和。
- 消息 ID 及控制命令定义位于 `COMMUNICATE/atpk.h`。
- 当前周期上报包含 IMU 加速度/角速度、DVL 速度、DVL 位置和 PWM 控制量等数据。

建议为上位机协议另行维护一份字段级文档，明确每个消息 ID 的字节序、缩放比例、单位和有效范围。

## 常用修改位置

| 需求 | 文件 |
| --- | --- |
| 修改时钟和外设初始化 | `Core/Src/main.c`、`IMUDMA.ioc` |
| 修改串口参数 | `Core/Src/usart.c`、`IMUDMA.ioc` |
| 修改 RTOS 任务、队列和优先级 | `Core/Src/freertos.c` |
| 修改上位机消息及控制命令 | `COMMUNICATE/atpk.c`、`COMMUNICATE/atpk.h` |
| 修改 PID 参数 | `stabilizer/stabilizer.c`、`PID/` |
| 修改 PWM 范围和执行机构映射 | `moter/moter.h`、`moter/moter.c` |
| 修改 EKF 参数和融合逻辑 | `EKF/EKF.c`、`EKF/EKF.h` |

## 注意事项

- **上电后执行机构可能动作。** `stabilizerTask` 启动阶段包含舵面渐变动作，随后持续刷新推进和舵机输出。调试时应拆除桨叶或断开执行机构动力。
- STM32H7 使用 D-Cache 时，DMA 缓冲区需要特别注意缓存一致性；修改内存布局或启用缓存后，应重新检查 UART、SPI 和 USB DMA 数据。
- 多个串口接收使用 Receive-to-Idle + DMA，修改缓冲区长度或回调逻辑时，应同时检查消息队列容量和重新启动接收的位置。
- `printf` 当前重定向到 LPUART1。高频打印可能阻塞实时任务，不建议在控制环内大量输出。
- 源文件中存在不同编码的历史中文注释；编辑时建议保持原编码，或一次性统一为 UTF-8 后完整编译验证。
- 仓库包含部分历史构建产物。发布版本前建议重新构建，并以新生成的 HEX/AXF 为准。

## 许可证

STM32 HAL、CMSIS、FreeRTOS 和 USB Device 等第三方组件分别受其目录内许可证约束。当前工程未提供覆盖自定义业务代码的顶层许可证；在复制、分发或商用前，请先向项目维护者确认授权范围。
