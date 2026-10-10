# k230_to_stm32 —— K230 视觉接收模块

配合 K230 侧的人体检测脚本，把「人体框中心坐标」从串口收进来，校验后交给
云台控制环使用。本文档按开发规范 §14.3 编写。

---

## 1. 功能说明

模块做四件事：

1. **收**：逐字节喂入解析状态机，从串口字节流里切出 13 字节定长帧。
2. **校验**：帧头同步 + 前 12 字节异或校验，校验不过的整帧丢弃。
3. **换算**：把画面像素坐标换成「相对画面中心的偏差」，控制环拿来就能用。
4. **兜底**：目标丢失或视觉链路超时后，把 `data_valid` 清零，
   禁止控制环沿用上一次的坐标（规范 §12.1）。

分层上本模块属于 **Module 层**，不直接碰 HAL；串口与 DMA/中断由
`bsp/usart/usart_k230.c` 负责（规范 §2.4 只允许 BSP 直接调用 HAL）。

---

## 2. 文件结构

| 文件 | 层次 | 作用 |
|---|---|---|
| `modules/k230_to_stm32.h` | Module | 协议常量、`K230_Data_s`、外部接口声明 |
| `modules/k230_to_stm32.c` | Module | 状态机解析、校验、坐标换算、超时判定 |
| `bsp/usart/usart_k230.h` | BSP | 串口「空闲中断不定长接收」驱动接口 |
| `bsp/usart/usart_k230.c` | BSP | 中断 / DMA 两种接收方式的实现 + HAL 回调 |
| `../person_detection_gimbal.py` | K230 | 发送端（人体检测 + 打包发帧），`main.py` 是同一份 |

数据流：

```text
K230 脚本 --USART2 115200--> BSP(usart_k230) --中断回调--> 本模块状态机
     --> K230_Data_s 快照 --主循环读取--> Application（云台 PID）
```

---

## 3. 通信协议（13 字节，大端序）

| 字节 | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0-3 | 帧头 | — | `0xAA 0x55 0x01 0x08` |
| 4-5 | `x` | int16 | 人体框中心像素 X，`-1` = 这一帧没检测到人 |
| 6-7 | `y` | int16 | 人体框中心像素 Y，`-1` = 这一帧没检测到人 |
| 8-9 | `reserved_1` | uint16 | 保留，当前固定 0 |
| 10-11 | `reserved_2` | uint16 | 保留，当前固定 0 |
| 12 | `checksum` | uint8 | 前 12 字节逐字节异或 |

约定：

- 坐标系是 K230 检测画面（`rgb888p_size`，默认 1280x960）的像素坐标，
  原点在左上角，`x` 向右增、`y` 向下增。
- 有目标时 K230 约 30ms 发一帧；没目标时每 200ms 补发一帧 `-1`，
  让 STM32 及时知道「人没了」，而不是拿着旧坐标继续追。
- 两端必须一致：改帧格式要同时改
  `modules/k230_to_stm32.h` 和 `person_detection_gimbal.py`。

---

## 4. 画面坐标与云台偏差

K230 发的是绝对像素坐标，控制环要的是「离画面中心多远」，所以本模块
顺手算好两个偏差：

```text
offset_x_px = x_px - K230_IMAGE_WIDTH_PX  / 2
offset_y_px = y_px - K230_IMAGE_HEIGHT_PX / 2
```

画面尺寸写在 `K230_IMAGE_WIDTH_PX` / `K230_IMAGE_HEIGHT_PX`，
**必须和 K230 脚本 main 里的 `rgb888p_size` 一致**。比如 K230 侧改成
`[640, 480]`，这里也要改成 640 / 480，否则偏差会整体偏掉。

符号约定（默认摄像头不镜像时）：

- `offset_x_px > 0`：人在画面中心右侧 → 云台需要往右转。
- `offset_y_px > 0`：人在画面中心下方 → 云台需要往下转。

如果装上云台后发现方向相反，先确认摄像头有没有开镜像/翻转，
再在控制环里取负号，不要在模块里改。

---

## 5. 外部接口

### 初始化和生命周期

```c
/* 一步到位：注册 + 启动接收。必须在 MX_USART2_UART_Init() 之后调用。 */
uint8_t k230_to_stm32_Init(UART_HandleTypeDef *uart_handle);

/* 取默认实例。后续所有接口都要拿它，初始化失败时返回 NULL。 */
K230ToStm32Instance *K230ToStm32GetInstance(void);

/* 分步用法（多路视觉或需要自己管实例时）。 */
K230ToStm32Instance *K230ToStm32Register(const K230ToStm32_Init_Config_s *config);
uint8_t K230ToStm32Start(K230ToStm32Instance *instance);
```

返回值：`k230_to_stm32_Init()` / `K230ToStm32Start()` 返回 `1` 表示已在后台
接收，`0` 表示注册或启动失败（实例池满、串口状态异常等）。

### 取数据（给控制环用）

```c
uint8_t K230ToStm32GetData(K230ToStm32Instance *instance, K230_Data_s *data_out);
```

返回 `1` = 坐标可信，可以闭环；返回 `0` = 目标丢失或视觉离线，**不能**用。

`K230_Data_s` 字段：

| 字段 | 单位 | 说明 |
|---|---|---|
| `x_px` | px | 人体框中心像素 X，丢失时为负值 |
| `y_px` | px | 人体框中心像素 Y，丢失时为负值 |
| `offset_x_px` | px | `x_px - 画面中心 X`，可直接当 yaw 误差 |
| `offset_y_px` | px | `y_px - 画面中心 Y`，可直接当 pitch 误差 |
| `target_detected` | — | 1 = 本帧检测到人，0 = 目标丢失 |
| `data_valid` | — | 1 = 坐标可信且未超时，可用于闭环 |
| `last_frame_time_ms` | ms | 最近一帧合法数据的 `HAL_GetTick()` |

注意：返回 `0` 时 `x_px` / `y_px` / `offset_*` 仍然是有效的调试值，
用来区分「目标丢了」和「视觉根本没在发」两种情况。

### 链路状态与统计

```c
/* 单查链路在线（200ms 内收到过合法帧）。联调时用来区分
   「协议不对」还是「串口没数据」。 */
uint8_t K230ToStm32IsOnline(K230ToStm32Instance *instance);

/* 帧计数，可在调试器里 Watch，或打到 OLED / 日志上。 */
void K230ToStm32GetStats(K230ToStm32Instance *instance, K230ToStm32_Stats_s *stats_out);
```

| 统计字段 | 含义 |
|---|---|
| `frame_count` | 通过校验的合法帧总数 |
| `checksum_error_count` | 校验失败丢弃的帧数 |
| `lost_frame_count` | 合法帧里「没检测到人」的帧数 |

---

## 6. 初始化示例

```c
#include "main.h"
#include "usart.h"
#include "k230_to_stm32.h"

/* CubeMX 生成的初始化 */
MX_GPIO_Init();
MX_DMA_Init();
MX_USART2_UART_Init();      /* PA2 = USART2_TX，PA3 = USART2_RX */

/* USER CODE BEGIN 2 */
/* 注册并启动 K230 视觉接收。失败也不阻塞启动，靠 IsOnline() 判断链路。 */
(void)k230_to_stm32_Init(&huart2);
/* USER CODE END 2 */
```

---

## 7. 典型使用流程

控制任务里按固定周期（规范 §8.5：周期必须显式定义）取一次快照，
用 `data_valid` 决定「跟」还是「停」：

```c
K230_Data_s vision;

if (K230ToStm32GetData(K230ToStm32GetInstance(), &vision) != 0u) {
    /* 有目标：偏差量送 PID，输出限幅后再给舵机 */
    float yaw_err_px   = (float)vision.offset_x_px;
    float pitch_err_px = (float)vision.offset_y_px;
    (void)yaw_err_px;
    (void)pitch_err_px;
} else {
    /* 目标丢失 / 视觉离线：停止跟踪（回中或保持安全姿态），
       禁止沿用上一次的坐标，否则会朝空处一直追 */
}
```

---

## 8. 注意事项

1. **初始化顺序**：必须在 `MX_USART2_UART_Init()` 之后调用
   `k230_to_stm32_Init()`。
2. **接收方式**：BSP 会自动判断——该串口挂了 RX DMA 就走 IDLE + DMA，
   没挂就跑 IDLE + 中断。本工程 USART2 只配了 TX DMA，所以走中断路径，
   不需要改 CubeMX。
3. **中断里要短**：解析在串口中断里执行，只做状态机和校验，不做打印、
   不延时（规范 §8.1）。
4. **只读快照**：`data` 由中断写、主循环读，请统一走
   `K230ToStm32GetData()`（内部有临界区），不要自己 extern 实例字段。
5. **超时就是失效**：超过 200ms 没收到合法帧，`data_valid` 会变 0，
   这是设计行为，不是 bug。
6. **画面尺寸两边一致**：`K230_IMAGE_WIDTH_PX` / `K230_IMAGE_HEIGHT_PX`
   要和 K230 侧 `rgb888p_size` 对上。
7. **当前工程状态**：CubeMX 里目前只有 USART2，**舵机 PWM 的定时器还没配**，
   云台输出那一环需要先在 CubeMX 里补 TIM + PWM 通道。

---

## 9. 常见错误

| 现象 | 可能原因 |
|---|---|
| `frame_count` 一直是 0，`checksum_error_count` 也是 0 | 接线问题：TX/RX 没交叉、没共地、K230 没在跑；或 USART2 中断没使能 |
| `checksum_error_count` 一直涨 | 协议不对：K230 那边还在跑旧的 8 字节脚本；或波特率不是 115200 |
| `frame_count` 涨但 `data_valid` 恒为 0 | 人在画面外/太小被过滤；或 `rgb888p_size` 和 `K230_IMAGE_*` 不一致，坐标越界被丢 |
| 联调正常，一上云台就丢帧 | 主循环里有长延时或被高优先级中断长时间抢占，超过了超时判定 |
| `IsOnline()` 一直 0 但帧在涨 | 时间戳没更新（说明校验一直失败），看 `checksum_error_count` |
| 云台往反方向追 | 摄像头镜像，或 PID 误差符号取反了 |

---

## 10. 单元测试

模块级测试不接 K230，用串口助手（或 USB-TTL）直接发手工帧即可：

```text
正中心、检测到人（x=640, y=480，偏差 0/0）：
AA 55 01 08 02 80 01 E0 00 00 00 00 95

目标丢失（x=-1, y=-1）：
AA 55 01 08 FF FF FF FF 00 00 00 00 F6
```

验收判据：

1. 发「正中心」帧后，`frame_count` +1，`data_valid` = 1，
   `offset_x_px` / `offset_y_px` 都是 0。
2. 发「丢失」帧后，`target_detected` = 0，`data_valid` = 0，
   `lost_frame_count` +1。
3. 停止发送 200ms 后，`K230ToStm32IsOnline()` 变 0，且
   `K230ToStm32GetData()` 返回 0。
4. 故意发一个错校验帧，`checksum_error_count` +1，`frame_count` 不变。
