# RoboMaster 电控 C/C++ 开发规范 v0.1

> 适用范围：RoboMaster / 嵌入式机器人电控代码，包括 STM32、FreeRTOS、CAN 电机、云台、底盘、发射、裁判系统、视觉通信、超级电容等模块。  
> 设计目标：让代码更安全、更容易联调、更容易传承、更适合比赛现场维护。

本规范综合参考：

- Google C++ Style Guide 的 C/C++ 工程习惯
- 《阿里巴巴 Java 开发手册》的“强制 / 推荐 / 参考”分级思想
- 湖南大学跃鹿 basic_framework 的 RoboMaster 电控工程分层经验
- 嵌入式实时控制中的安全性、实时性、可调试性要求

---

## 0. 规则等级


| 等级 | 含义 |
|---|---|
| **强制** | 必须遵守，违反后容易造成 bug、联调困难、比赛风险 |
| **推荐** | 默认遵守，特殊情况可以说明理由后例外 |
| **参考** | 根据项目规模、硬件条件、队伍习惯选择采用 |

---

## 1. 总原则

### 1.1 强制：代码首先为人服务，其次才是给编译器看

电控代码不只是“能跑”，还要能被：

- 自己三个月后看懂
- 队友接手
- 联调时快速定位问题
- 比赛现场快速排查
- 新人训练时复用

禁止写“只有作者当时知道为什么”的代码。

---

### 1.2 强制：应用层不得直接操作底层硬件

电控代码推荐分为三层：

```text
Application 层：机器人应用逻辑
Module 层：设备、算法、通信模块
BSP 层：片上外设封装
```

依赖方向只能是：

```text
Application -> Module -> BSP -> HAL / LL / Register
```

禁止反向依赖。

**正确示例：**

```c
DJIMotorSetRef(yaw_motor, yaw_ref_rad);
ChassisSetVelocity(vx_mps, vy_mps, wz_radps);
PubPushMessage(gimbal_pub, &gimbal_feedback);
```

**错误示例：**

```c
// application/gimbal.c 中禁止直接出现这种代码
HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &mailbox);
HAL_UART_Transmit(&huart1, buffer, len, 10);
```

---

### 1.3 强制：所有物理量必须明确单位

控制代码最怕单位混乱。变量名或注释中必须说明单位。

**推荐：**

```c
float yaw_angle_rad;
float yaw_speed_radps;
float chassis_vx_mps;
float motor_speed_rpm;
float motor_current_amp;
float control_dt_s;
float pitch_limit_max_rad;
```

**禁止：**

```c
float yaw;
float speed;
float current;
float angle;
```

除非变量所在结构体、函数名或上下文已经明确单位。

---

### 1.4 强制：禁止魔法值

禁止在控制代码中直接出现无解释的数字。

**错误示例：**

```c
if (fabsf(yaw_error_rad) < 0.01f) {
    output = 0.0f;
}

pid.kp = 8.0f;
```

**正确示例：**

```c
#define YAW_ANGLE_DEADBAND_RAD 0.01f
#define GIMBAL_YAW_KP 8.0f

if (fabsf(yaw_error_rad) < YAW_ANGLE_DEADBAND_RAD) {
    output = 0.0f;
}

pid.kp = GIMBAL_YAW_KP;
```

---

## 2. 工程目录规范

### 2.1 推荐目录结构

```text
project/
├─ application/
│  ├─ robot.c
│  ├─ robot.h
│  ├─ robot_def.h
│  ├─ gimbal/
│  ├─ chassis/
│  ├─ shoot/
│  ├─ cmd/
│  └─ referee/
│
├─ modules/
│  ├─ motor/
│  ├─ imu/
│  ├─ remote/
│  ├─ referee/
│  ├─ vision/
│  ├─ super_cap/
│  ├─ message_center/
│  ├─ algorithm/
│  └─ daemon/
│
├─ bsp/
│  ├─ can/
│  ├─ usart/
│  ├─ spi/
│  ├─ i2c/
│  ├─ gpio/
│  ├─ pwm/
│  ├─ adc/
│  ├─ dwt/
│  └─ log/
│
├─ config/
│  ├─ robot_config.h
│  ├─ motor_config.h
│  ├─ pid_config.h
│  └─ board_config.h
│
├─ docs/
├─ tests/
├─ Drivers/
├─ Middlewares/
├─ Inc/
├─ Src/
├─ CMakeLists.txt
└─ README.md
```

---

### 2.2 强制：Application 层只写机器人逻辑

Application 层应该关心：

- 云台模式
- 底盘模式
- 发射逻辑
- 视觉自瞄开关
- 裁判系统功率限制
- 遥控器 / 键鼠 / 上位机指令解析
- 机器人状态机

Application 层不应该关心：

- CAN 邮箱
- UART DMA
- SPI 读写
- HAL 回调
- 电机协议细节
- IMU 寄存器细节

---

### 2.3 强制：Module 层封装设备和算法

Module 层负责：

```text
DJI 电机
达妙电机
IMU
遥控器
裁判系统
视觉通信
超级电容
PID
滤波器
观测器
消息中心
守护线程
```

Module 对 Application 提供硬件无关接口。

**示例：**

```c
DJIMotorInstance *DJIMotorRegister(const Motor_Init_Config_s *config);
void DJIMotorSetRef(DJIMotorInstance *motor, float ref);
void DJIMotorStop(DJIMotorInstance *motor);
void DJIMotorEnable(DJIMotorInstance *motor);
```

Application 不应该知道这个电机最终是通过 CAN1 还是 CAN2 发出去的。

---

### 2.4 强制：BSP 层是唯一允许直接调用 HAL 的业务封装层

BSP 层负责：

```text
bsp_can
bsp_usart
bsp_spi
bsp_i2c
bsp_pwm
bsp_gpio
bsp_dwt
bsp_log
```

只有 BSP 层可以直接出现：

```c
HAL_CAN_...
HAL_UART_...
HAL_SPI_...
HAL_I2C_...
HAL_TIM_...
HAL_GPIO_...
```

特殊情况必须在代码注释中说明原因。

---

## 3. 命名规范

### 3.1 强制：禁止拼音和英文混用

**错误示例：**

```c
float yaw_jiaodu;
float dianji_speed;
void SetMocaLunSpeed(void);
```

**正确示例：**

```c
float yaw_angle_rad;
float motor_speed_rpm;
void SetFrictionWheelSpeed(void);
```

允许已经约定俗成的专有名词，例如：

```c
DJIMotor
RoboMaster
DaMiao
Referee
```

---

### 3.2 文件和目录命名

**强制：文件名使用小写下划线。**

```text
gimbal_task.c
gimbal_task.h
dji_motor.c
dji_motor.h
message_center.c
message_center.h
robot_def.h
```

**禁止：**

```text
GimbalTask.c
DJIMotor.C
MessageCenter.h
```

---

### 3.3 变量命名

**强制：变量使用小写下划线。**

```c
float yaw_angle_rad;
float pitch_ref_rad;
uint8_t motor_online_flag;
uint32_t last_update_time_ms;
```

**推荐：变量名体现物理意义和单位。**

```c
float chassis_vx_mps;
float chassis_wz_radps;
float capacitor_voltage_v;
float motor_current_amp;
```

---

### 3.4 函数命名

C 工程中推荐使用：

```text
模块名 + 动作
```

采用大驼峰风格：

```c
void DJIMotorSetRef(DJIMotorInstance *motor, float ref);
void DJIMotorStop(DJIMotorInstance *motor);
void GimbalInit(void);
void GimbalTask(void);
void ChassisSetVelocity(float vx_mps, float vy_mps, float wz_radps);
```

C++ 工程中推荐：

```cpp
class DjiMotor {
public:
    void setRef(float ref);
    void stop();
    void enable();
};
```

如果项目已经统一使用 `snake_case` 函数名，也可以接受，但必须全项目一致。

---

### 3.5 类型命名

C 语言结构体：

简单数据类型使用 `_t` 后缀：

```c
typedef struct {
    float x;
    float y;
    float z;
} Vector3f_t;
```

复杂模块配置或状态结构体使用 `_s` 后缀：

```c
typedef struct {
    float kp;
    float ki;
    float kd;
    float max_out;
    float integral_limit;
} PID_Init_Config_s;
```

实例对象使用 `XXXInstance`：

```c
typedef struct {
    CANInstance *can_instance;
    Motor_Measure_s measure;
    PIDInstance speed_pid;
} DJIMotorInstance;
```

C++ 类名使用大驼峰：

```cpp
class DjiMotor;
class GimbalController;
class ChassisController;
class MessageCenter;
```

---

### 3.6 常量和宏命名

常量、宏使用全大写下划线：

```c
#define MAX_MOTOR_NUM 8
#define GIMBAL_YAW_LIMIT_RAD 1.57f
#define CHASSIS_CONTROL_PERIOD_MS 2
```

C++ 中优先使用：

```cpp
constexpr float kGimbalYawLimitRad = 1.57f;
constexpr uint32_t kChassisControlPeriodMs = 2;
```

但在纯 C 或需要条件编译时可以使用宏。

---

## 4. 代码格式规范

### 4.1 强制：使用 UTF-8 编码

所有源码、头文件、Markdown 文档统一使用 UTF-8。

---

### 4.2 强制：缩进使用 4 个空格

禁止混用 Tab 和空格。

---

### 4.3 推荐：单行长度不超过 120 字符

超过后换行。

```c
Motor_Init_Config_s yaw_config = {
    .can_init_config = {
        .can_handle = &hcan1,
        .tx_id = GIMBAL_YAW_MOTOR_CAN_ID,
    },
    .controller_param_init_config = {
        .angle_pid = {
            .kp = GIMBAL_YAW_ANGLE_KP,
            .ki = GIMBAL_YAW_ANGLE_KI,
            .kd = GIMBAL_YAW_ANGLE_KD,
        },
    },
};
```

---

### 4.4 强制：if / for / while 必须使用大括号

即使只有一行，也必须写大括号。

**错误示例：**

```c
if (motor == NULL)
    return;
```

**正确示例：**

```c
if (motor == NULL) {
    return;
}
```

---

### 4.5 强制：编译不允许 warning

工程应开启：

```text
-Wall
-Wextra
-Werror
```

至少比赛分支必须做到：

```text
0 warning
0 error
```

warning 不允许以“能跑就行”为理由保留。

---

## 5. 头文件规范

### 5.1 强制：头文件必须有保护

C 语言推荐：

```c
#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

// declarations

#ifdef __cplusplus
}
#endif

#endif
```

C++ 也可以使用：

```cpp
#pragma once
```

但同一项目内尽量统一。

---

### 5.2 强制：头文件只暴露必要接口

`.h` 文件中放：

- 类型定义
- 宏定义
- 外部接口声明
- 必要的 inline 函数

`.c/.cpp` 文件中放：

- 私有变量
- 私有函数
- 具体实现

私有函数必须使用 `static`。

---

### 5.3 强制：禁止在头文件中定义普通全局变量

**错误示例：**

```c
// motor.h
int motor_count;
```

**正确示例：**

```c
// motor.h
extern int motor_count;

// motor.c
int motor_count = 0;
```

但更推荐避免全局变量，通过接口访问。

---

## 6. C 语言模块设计规范

### 6.1 强制：模块统一使用 Instance 设计

对于可以存在多个实例的模块，统一设计为：

```c
XXXInstance *XXXRegister(const XXX_Init_Config_s *config);
void XXXDoSomething(XXXInstance *instance, ...);
```

示例：

```c
DJIMotorInstance *DJIMotorRegister(const Motor_Init_Config_s *config);
void DJIMotorSetRef(DJIMotorInstance *motor, float ref);
```

这相当于 C 语言手动模拟 C++ 的 `this` 指针。

---

### 6.2 强制：初始化配置使用结构体传入

禁止函数参数过长。

**错误示例：**

```c
DJIMotorInit(&hcan1, 1, GM6020, 8.0f, 0.0f, 0.0f, 50.0f, 200.0f, 0.0f);
```

**正确示例：**

```c
Motor_Init_Config_s yaw_config = {
    .can_config = {
        .can_handle = &hcan1,
        .tx_id = GIMBAL_YAW_MOTOR_CAN_ID,
    },
    .motor_type = GM6020,
    .angle_pid = {
        .kp = GIMBAL_YAW_ANGLE_KP,
        .ki = GIMBAL_YAW_ANGLE_KI,
        .kd = GIMBAL_YAW_ANGLE_KD,
    },
};

yaw_motor = DJIMotorRegister(&yaw_config);
```

---

### 6.3 强制：所有 public 接口必须做参数检查

```c
void DJIMotorSetRef(DJIMotorInstance *motor, float ref)
{
    if (motor == NULL) {
        return;
    }

    motor->ref = ref;
}
```

对于严重错误，可以使用断言或错误处理接口：

```c
ASSERT(motor != NULL);
```

---

### 6.4 推荐：回调函数使用 parent pointer / user_data

底层回调不应该直接 include 上层模块。

推荐设计：

```c
typedef struct CANInstance {
    CAN_HandleTypeDef *can_handle;
    uint32_t rx_id;
    void *parent;
    void (*rx_callback)(struct CANInstance *can);
} CANInstance;
```

Module 注册时：

```c
can_config.parent = motor;
can_config.rx_callback = DJIMotorDecode;
```

回调中：

```c
static void DJIMotorDecode(CANInstance *can)
{
    DJIMotorInstance *motor = (DJIMotorInstance *)can->parent;
    // decode motor feedback
}
```

这样可以避免 BSP 层包含 Module 层头文件。

---

## 7. C++ 嵌入式使用规范

如果项目使用 C++，必须遵守以下约束。

### 7.1 强制：默认禁用异常

嵌入式控制代码中禁止使用：

```cpp
throw
try
catch
```

编译选项建议：

```text
-fno-exceptions
```

错误处理使用：

```cpp
enum class ErrorCode;
bool;
std::optional;  // 如果项目支持且无动态分配风险
```

---

### 7.2 强制：默认禁用 RTTI

禁止依赖：

```cpp
dynamic_cast
typeid
```

编译选项建议：

```text
-fno-rtti
```

---

### 7.3 强制：控制周期内禁止动态内存分配

禁止在实时任务中使用：

```cpp
new
delete
malloc
free
std::vector::push_back
std::string 动态拼接
```

推荐使用：

```cpp
std::array
固定容量环形队列
静态对象
对象池
```

---

### 7.4 推荐：使用 `constexpr` 替代宏常量

```cpp
constexpr float kYawLimitRad = 1.57f;
constexpr uint32_t kControlPeriodMs = 2;
```

---

### 7.5 推荐：使用 `enum class`

**推荐：**

```cpp
enum class GimbalMode {
    ZeroForce,
    Gyro,
    Vision,
    Debug,
};
```

**不推荐：**

```cpp
enum GimbalMode {
    GIMBAL_ZERO_FORCE,
    GIMBAL_GYRO,
};
```

---

### 7.6 谨慎使用虚函数和模板

虚函数、模板不是禁止，但必须考虑：

- Flash 占用
- 调试复杂度
- 编译时间
- 是否真的需要运行时多态
- 是否会增加新人理解成本

控制主链路中不建议炫技。

---

## 8. 实时系统与并发规范

### 8.1 强制：中断回调必须短小

中断里只允许做：

- 读取寄存器 / DMA 状态
- 复制数据
- 设置标志位
- 释放信号量
- 调用非常短的解析函数

禁止在中断中：

```c
osDelay();
HAL_Delay();
printf();
复杂 PID 计算;
大量 memcpy;
等待锁;
阻塞式串口发送;
```

---

### 8.2 强制：临界区内禁止延时

禁止：

```c
taskENTER_CRITICAL();
osDelay(1);
taskEXIT_CRITICAL();
```

这可能导致系统时基停止，严重时卡死。

---

### 8.3 强制：跨任务共享数据必须说明所有权

以下数据必须考虑并发保护：

- CAN 接收反馈
- IMU 姿态数据
- 遥控器数据
- 裁判系统数据
- 视觉目标数据
- 电机反馈
- 控制指令
- 日志缓冲区

推荐方式：

1. 单写多读，使用快照复制
2. 消息队列
3. 环形缓冲区
4. 临界区保护
5. 互斥锁
6. 双缓冲

---

### 8.4 强制：`volatile` 不能替代锁

`volatile` 只能防止编译器优化访问，不能保证：

- 原子性
- 一致性
- 多字段同步
- 数据结构完整性

错误理解：

```c
volatile RemoteData remote_data; // 这样不代表线程安全
```

---

### 8.5 推荐：每个周期任务必须明确周期

```c
#define GIMBAL_TASK_PERIOD_MS 1
#define CHASSIS_TASK_PERIOD_MS 2
#define SHOOT_TASK_PERIOD_MS 2
#define REFEREE_TASK_PERIOD_MS 10
```

任务内推荐使用周期调度，而不是随手 `osDelay(1)`。

---

## 9. 控制算法规范

### 9.1 强制：控制输入、反馈、输出必须明确单位

```c
typedef struct {
    float ref_rad;
    float feedback_rad;
    float error_rad;
    float output_current_amp;
} Gimbal_Yaw_Control_s;
```

---

### 9.2 强制：PID 参数必须有来源

PID 参数不能只写：

```c
.kp = 8,
.ki = 0,
.kd = 0,
```

至少需要满足一种：

1. 注释说明调参对象和现象
2. 文档记录调参过程
3. 参数文件中记录日期和适用机构
4. 有阶跃响应、日志、曲线截图

推荐：

```c
// 2026-07-28, infantry gimbal v2, GM6020 yaw.
// Stable under 15 m/s small-buff tracking, no visible overshoot.
#define GIMBAL_YAW_ANGLE_KP 8.0f
#define GIMBAL_YAW_ANGLE_KI 0.0f
#define GIMBAL_YAW_ANGLE_KD 0.0f
```

---

### 9.3 强制：所有控制输出必须限幅

```c
output = LimitFloat(output, -max_output, max_output);
```

禁止 PID 或前馈输出无限制进入电机发送。

---

### 9.4 强制：积分项必须考虑抗饱和

至少使用一种：

- 积分限幅
- 积分分离
- 梯形积分
- 条件积分
- 输出饱和时停止积分

---

### 9.5 推荐：控制结构显式写出

比如串级 PID：

```text
角度环：
    输入：target_angle_rad
    反馈：imu_yaw_angle_rad
    输出：target_speed_radps

速度环：
    输入：target_speed_radps
    反馈：gyro_z_radps
    输出：motor_current_cmd
```

不要让读代码的人猜：

```c
pid_ref = PIDCalc(&angle_pid, target, feedback);
pid_ref = PIDCalc(&speed_pid, pid_ref, speed);
```

最好配合注释说明。

---

### 9.6 推荐：控制模式使用状态机

云台、底盘、发射等不要用大量散乱 `if`。

推荐：

```c
typedef enum {
    GIMBAL_ZERO_FORCE = 0,
    GIMBAL_GYRO_MODE,
    GIMBAL_VISION_MODE,
    GIMBAL_DEBUG_MODE,
} Gimbal_Mode_e;
```

任务中：

```c
switch (gimbal_mode) {
case GIMBAL_ZERO_FORCE:
    GimbalHandleZeroForce();
    break;

case GIMBAL_GYRO_MODE:
    GimbalHandleGyroMode();
    break;

case GIMBAL_VISION_MODE:
    GimbalHandleVisionMode();
    break;

default:
    GimbalHandleSafeMode();
    break;
}
```

---

## 10. 电机与 CAN 通信规范

### 10.1 强制：CAN ID 不得散落在业务代码中

**错误示例：**

```c
.tx_id = 1;
.rx_id = 0x201;
```

**正确示例：**

```c
#define GIMBAL_YAW_MOTOR_TX_ID 1
#define GIMBAL_YAW_MOTOR_RX_ID 0x205
```

或集中在配置结构体中。

---

### 10.2 强制：电机必须有离线检测

每个关键电机必须记录最后一次反馈时间。

```c
if (GetSystemTimeMs() - motor->last_update_time_ms > MOTOR_OFFLINE_TIMEOUT_MS) {
    motor->online = false;
    DJIMotorStop(motor);
}
```

---

### 10.3 强制：电机必须支持安全停止

至少提供：

```c
void DJIMotorStop(DJIMotorInstance *motor);
void DJIMotorEnable(DJIMotorInstance *motor);
```

停止时输出必须为 0 或安全值。

---

### 10.4 强制：发送电流前必须限幅

```c
motor_current = LimitFloat(motor_current, -MOTOR_MAX_CURRENT, MOTOR_MAX_CURRENT);
```

尤其是：

- GM6020
- M3508
- M2006
- 达妙电机
- 海泰电机
- 瓴控电机

---

### 10.5 推荐：统一 CAN 发送打包

DJI 电机不应该每个电机单独随便发送。

推荐由 motor task 统一处理：

```text
0x200 -> motor 1~4
0x1FF -> motor 5~8
```

避免同一组 CAN 电流被多个地方覆盖。

---

## 11. 消息通信规范

### 11.1 推荐：Application 之间通过消息中心通信

推荐数据流：

```text
cmd 发布 gimbal_cmd
cmd 发布 chassis_cmd
cmd 发布 shoot_cmd

gimbal 订阅 gimbal_cmd
chassis 订阅 chassis_cmd
shoot 订阅 shoot_cmd

gimbal 发布 gimbal_feedback
chassis 发布 chassis_feedback
shoot 发布 shoot_feedback
```

避免：

```c
#include "gimbal.h"
#include "chassis.h"

extern GimbalData gimbal_data;
extern ChassisData chassis_data;
```

---

### 11.2 强制：消息结构体必须稳定、清晰

```c
typedef struct {
    Gimbal_Mode_e mode;
    float yaw_ref_rad;
    float pitch_ref_rad;
    uint8_t vision_enable;
} Gimbal_Ctrl_Cmd_s;
```

禁止使用含义不明的字段：

```c
typedef struct {
    float a;
    float b;
    uint8_t flag;
} Cmd_s;
```

---

### 11.3 推荐：topic 名称集中定义

如果使用字符串 topic，禁止到处手写：

```c
PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
```

推荐：

```c
#define TOPIC_GIMBAL_CMD "gimbal_cmd"
#define TOPIC_GIMBAL_FEEDBACK "gimbal_feedback"
```

这样可以减少拼写错误。

---

## 12. 错误处理与安全规范

### 12.1 强制：关键模块必须有 fail-safe

以下情况必须进入安全模式：

- 遥控器离线
- 电机离线
- IMU 离线
- 视觉通信超时
- 裁判系统超时
- 超级电容异常
- 底盘功率异常
- 云台角度超过安全范围
- 发射机构卡弹

安全模式至少包括：

```text
电机输出归零
发射停止
云台限位
底盘停止或降功率
蜂鸣器 / LED / log 提示
```

---

### 12.2 强制：默认分支必须安全

```c
switch (mode) {
case NORMAL:
    ...
    break;

case DEBUG:
    ...
    break;

default:
    EnterSafeMode();
    break;
}
```

禁止：

```c
default:
    break;
```

除非明确说明不会出现其他值。

---

### 12.3 推荐：使用守护线程

守护线程负责检查：

- 模块是否在线
- 控制周期是否超时
- 任务是否卡死
- 电机反馈是否异常
- 遥控器是否失联

---

## 13. 日志与调试规范

### 13.1 强制：禁止用无节制 printf 调试实时控制

尤其禁止在：

- 高频控制任务
- 中断回调
- CAN 接收回调
- 1ms 任务

中大量打印。

---

### 13.2 推荐：日志分级

```c
LOG_DEBUG("yaw ref: %.2f", yaw_ref_rad);
LOG_INFO("gimbal initialized");
LOG_WARN("remote offline");
LOG_ERROR("imu init failed");
```

---

### 13.3 推荐：高频变量使用 Ozone / RTT / FreeMaster / VOFA 观察

控制调参应至少观察：

```text
target
feedback
error
output
motor_current
gyro
control_dt
```

云台调参建议同时看：

```text
yaw_ref_rad
yaw_feedback_rad
yaw_error_rad
yaw_speed_radps
yaw_output_current
pitch_ref_rad
pitch_feedback_rad
pitch_output_current
```

---

## 14. 注释规范

### 14.1 强制：注释解释“为什么”，不是翻译代码

**错误示例：**

```c
// 计算误差
error = target - feedback;
```

**正确示例：**

```c
// 云台 yaw 轴 IMU 正方向与电机编码器正方向相反，
// 因此这里使用 IMU 角度作为主反馈，编码器仅用于底盘跟随。
yaw_error_rad = target_yaw_rad - imu_yaw_rad;
```

---

### 14.2 强制：外部接口必须有注释

```c
/**
 * @brief Set motor reference value.
 *
 * @param motor Motor instance.
 * @param ref Reference value. Unit depends on selected outer loop:
 *            angle loop: rad
 *            speed loop: rad/s
 *            current loop: amp
 */
void DJIMotorSetRef(DJIMotorInstance *motor, float ref);
```

---

### 14.3 推荐：每个模块配套 Markdown 文档

每个 module 至少包含：

```text
1. 功能说明
2. 文件结构
3. 外部接口
4. 初始化示例
5. 典型使用流程
6. 注意事项
7. 常见错误
```

例如：

```text
modules/motor/dji_motor.md
modules/imu/imu.md
modules/message_center/message_center.md
```

---

## 15. 配置与参数规范

### 15.1 强制：机器人参数集中管理

推荐：

```text
application/robot_def.h
config/robot_config.h
config/motor_config.h
config/pid_config.h
```

不要把大量参数散落在：

```text
gimbal.c
chassis.c
shoot.c
cmd.c
```

---

### 15.2 强制：不同机器人 / 不同板子使用条件编译或配置切换

例如：

```c
#define ROBOT_INFANTRY 1
#define ROBOT_HERO 2
#define ROBOT_SENTRY 3

#define CURRENT_ROBOT ROBOT_INFANTRY
```

或者：

```c
#define BOARD_GIMBAL 1
#define BOARD_CHASSIS 2
#define CURRENT_BOARD BOARD_GIMBAL
```

---

### 15.3 推荐：参数命名包含机构和用途

```c
#define INFANTRY_GIMBAL_YAW_KP 8.0f
#define INFANTRY_CHASSIS_MAX_VX_MPS 4.0f
#define HERO_SHOOT_FRICTION_SPEED_RPM 5200.0f
```

---

## 16. 测试与验收规范

### 16.1 强制：新模块必须有最小测试用例

例如新增一个 `TFminiPlus` 测距模块，必须能单独测试：

```text
1. 初始化传感器
2. 读取距离
3. 打印或观察距离值
4. 断开传感器后能检测离线
```

---

### 16.2 强制：上车前必须完成模块级测试

不要直接整车联调。

推荐顺序：

```text
1. 单独测试 BSP 外设
2. 单独测试 Module
3. 测试单电机
4. 测试单机构
5. 测试单板
6. 测试双板通信
7. 整车低功率测试
8. 整车实战参数测试
```

---

### 16.3 强制：电机上电前检查

每次新代码上车前至少检查：

```text
CAN ID 是否正确
电机方向是否正确
输出限幅是否正确
遥控器急停是否有效
离线保护是否有效
云台 / 底盘 / 发射默认是否 zero force
电源线和 CAN 线是否可靠
```

---

## 17. Git 与协作规范

### 17.1 强制：禁止把不可编译代码合入主分支

合入前至少满足：

```text
编译通过
无 warning
基本功能验证通过
关键参数有说明
未破坏已有机器人配置
```

---

### 17.2 推荐：提交信息说明“改了什么”和“为什么”

**不推荐：**

```text
fix
update
修改了一下
能跑了
```

**推荐：**

```text
fix(gimbal): correct yaw gyro feedback direction
feat(motor): add offline detection for DJI motor
tune(chassis): update mecanum power limit parameters
```

---

### 17.3 推荐：调参提交附带现象说明

例如：

```text
tune(gimbal): reduce pitch speed Ki to suppress low-frequency oscillation

- robot: infantry v2
- motor: GM6020 pitch
- symptom: 3~5 Hz oscillation after vision lock
- change: speed Ki 350 -> 220
- result: overshoot reduced, no visible steady-state drift
```

---

## 18. 禁止事项清单

以下行为强制禁止：

```text
1. Application 层直接调用 HAL_CAN / HAL_UART / HAL_SPI 等函数
2. 控制任务中使用无解释魔法值
3. 单位不明的控制变量
4. 电机输出不做限幅
5. 无离线检测直接闭环控制
6. 中断回调中 osDelay / HAL_Delay / printf
7. 临界区中延时
8. 带 warning 合入比赛分支
9. 头文件中定义普通全局变量
10. 通过大量 extern 全局变量进行模块通信
11. 拼音英文混合命名
12. 没有急停 / zero force / safe mode 的电机控制模块
13. 没有测试用例的新 BSP / Module
14. 比赛前临时改参数但不记录
15. 上车前跳过单模块测试
```

---

## 19. 推荐实践清单

以下行为推荐采用：

```text
1. BSP / Module / Application 三层架构
2. Application 之间通过消息中心通信
3. 每个 Module 使用 Instance + Register 设计
4. 每个 Module 配套 .md 文档
5. 所有物理量使用标准单位
6. PID 参数集中配置
7. 控制任务周期显式定义
8. Ozone / RTT / FreeMaster / VOFA 辅助调参
9. DWT 测量任务耗时
10. 守护线程监控模块在线状态
11. 统一错误码和日志等级
12. 调参提交记录现象、参数、结果
13. 新机器人通过 robot_def.h / config 切换
14. 使用 clang-format 或 IDE 格式化
15. 使用 Git 分支管理不同功能开发
```

---



## 21. 建议阅读材料

- Google C++ Style Guide 中文版
- 《阿里巴巴 Java 开发手册》
- 湖南大学跃鹿 basic_framework
- FreeRTOS 官方文档
- STM32 HAL / LL 官方文档
- Segger Ozone / SystemView 文档
- RoboMaster 官方裁判系统串口协议

---

## 22. 版本记录

| 版本 | 日期 | 说明 |
|---|---|---|
| v0.1 | 2026-07-28 | 初版，综合 C/C++ 代码规范、RM 电控框架经验与嵌入式实时控制要求 |
