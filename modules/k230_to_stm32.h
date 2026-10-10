#ifndef K230_TO_STM32_H
#define K230_TO_STM32_H

#include <stdint.h>
#include "usart_k230.h" /* 需要 UART_HandleTypeDef */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file k230_to_stm32.h
 * @brief Module 层：K230 视觉帧接收、校验与解析（人体中心坐标）。
 *
 * 数据流：
 *   K230 --USART2 115200--> BSP(usart_k230) --中断回调--> 本模块状态机
 *        --> K230_Data_s 快照 --主循环读取--> Application（云台跟踪）
 *
 * 协议：13 字节定长帧，大端序（与 K230 侧 person_detection_gimbal.py 严格对应）
 *   字节 0-3   帧头        0xAA 0x55 0x01 0x08
 *   字节 4-5   x           int16_t  人体框中心像素 X，-1 表示这一帧没检测到人
 *   字节 6-7   y           int16_t  人体框中心像素 Y，-1 表示这一帧没检测到人
 *   字节 8-9   reserved_1  uint16_t 保留，固定 0
 *   字节 10-11 reserved_2  uint16_t 保留，固定 0
 *   字节 12    checksum    uint8_t  前 12 字节逐字节异或
 */

/* ============================ 协议常量 ============================ */

#define K230_FRAME_LEN              13u  /* 整帧长度(字节) */
#define K230_FRAME_HEADER_LEN        4u  /* 帧头长度(字节) */
#define K230_FRAME_CHECKSUM_INDEX   12u  /* 校验字节在帧内的下标 */

#define K230_FRAME_HEADER_0       0xAAu
#define K230_FRAME_HEADER_1       0x55u
#define K230_FRAME_HEADER_2       0x01u
#define K230_FRAME_HEADER_3       0x08u

/* 载荷字段在帧内的下标 */
#define K230_FRAME_X_INDEX           4u
#define K230_FRAME_Y_INDEX           6u

/* 目标丢失标志：协议规定 x == -1（y 同）表示这一帧没检测到人，
   此时**不得**用上一帧坐标继续闭环 */
#define K230_TARGET_LOST_PX        (-1)

/* ====================== 画面坐标 → 云台偏差 ====================== */
/*
 * K230 发来的是检测画面里的像素坐标，原点在左上角。云台要消掉的是
 * 「人体中心 - 画面中心」，所以本模块顺手把偏差算好：
 *     offset_x_px = x - K230_IMAGE_WIDTH_PX  / 2
 *     offset_y_px = y - K230_IMAGE_HEIGHT_PX / 2
 * 画面尺寸必须和 K230 侧 main 里的 rgb888p_size 一致，改了要两边一起改。
 */

/* 检测画面尺寸(像素)。K230 侧默认 rgb888p_size = [1280, 960] */
#define K230_IMAGE_WIDTH_PX      1280
#define K230_IMAGE_HEIGHT_PX      960

/* ============================ 运行参数 ============================ */

/* 视觉链路超时(ms)。K230 约 30Hz 发送，超时取 200ms（约 6 个发包周期），
   既不会误判短暂丢帧，又能在视觉断开后及时把坐标判为无效 */
#define K230_VISION_TIMEOUT_MS   200u

/* 接收缓冲(字节)。K230 单帧 13 字节、帧间有总线空闲，一次空闲中断通常
   只收到 1 帧，128 字节（约 9 帧）留足余量。
   Circular DMA 模式下这个值决定"消费端最多能被阻塞多久"
   （128B / 430B/s ≈ 0.3 秒），超过会被静默覆盖，所以主循环里别放长延时 */
#define K230_RX_BUF_SIZE         128u

/* 实例池大小。每个实例内含 128 字节接收缓冲，占 RAM 较多，
   所以按当前实际需要（仅一路 K230）分配；后续增加视觉路数时改这里即可 */
#define K230_TO_STM32_MAX_INSTANCE 1u

/* ============================== 类型 ============================== */

typedef struct K230ToStm32Instance K230ToStm32Instance;

/** @brief 注册参数。 */
typedef struct {
    UART_HandleTypeDef *uart_handle; /* 例如 &huart2 */
} K230ToStm32_Init_Config_s;

/**
 * @brief 一帧视觉数据解析结果（快照）。
 *
 * @note 本结构体由接收中断写入、主循环读取。读取请统一走
 *       K230ToStm32GetData()，它内部做了临界区保护；不要直接访问实例字段。
 */
typedef struct {
    int16_t x_px;                /* 人体框中心像素 X，丢失时为负值 */
    int16_t y_px;                /* 人体框中心像素 Y，丢失时为负值 */
    int16_t offset_x_px;         /* x_px - 画面中心 X，可直接当云台 yaw 误差 */
    int16_t offset_y_px;         /* y_px - 画面中心 Y，可直接当云台 pitch 误差 */
    uint8_t target_detected;     /* 1 = 本帧检测到人，0 = 目标丢失 */
    uint8_t data_valid;          /* 1 = 坐标可信且未超时，可用于闭环 */
    uint32_t last_frame_time_ms; /* 最近一帧合法数据的 HAL_GetTick() 时刻 */
} K230_Data_s;

/** @brief 模块统计，用于联调定位「协议不对」还是「链路不通」。 */
typedef struct {
    uint32_t frame_count;          /* 通过校验的合法帧总数 */
    uint32_t checksum_error_count; /* 校验失败丢弃的帧数 */
    uint32_t lost_frame_count;     /* 合法帧中 x 表示目标丢失的帧数 */
} K230ToStm32_Stats_s;

/* ============================ 外部接口 ============================ */

/**
 * @brief 一步完成初始化：注册 + 启动接收。
 *
 * @param uart_handle 承载 K230 的串口句柄，例如 &huart2；传 NULL 时返回 0。
 * @return 1 = 已在后台接收；0 = 注册或启动失败。
 *
 * @note 必须在 MX_USART2_UART_Init() 之后调用（注册时要读 CubeMX 配好的串口/DMA）。
 * @note 失败原因（实例池已满 / 串口状态异常）只体现在返回值上，
 *       本函数不做任何输出；只关心「能不能收到数据」时，参数用
 *       K230ToStm32Register() + K230ToStm32Start() 自行分步调用。
 */
uint8_t k230_to_stm32_Init(UART_HandleTypeDef *uart_handle);

/**
 * @brief 获取 k230_to_stm32_Init() 创建的默认 K230 实例。
 *
 * @return 默认实例指针；尚未初始化或初始化失败时返回 NULL。
 *
 * @note 该接口只适用于单实例便捷初始化路径。使用
 *       K230ToStm32Register() 分步注册时，请直接保存返回的实例指针。
 */
K230ToStm32Instance *K230ToStm32GetInstance(void);

/**
 * @brief 注册 K230 视觉接收模块（内部会一并注册 BSP 层串口实例）。
 *
 * @param config 注册参数，不可为 NULL。
 * @return 模块实例指针；参数非法或实例池已满时返回 NULL。
 *
 * @note 注册不启动接收，需要再调用 K230ToStm32Start()。
 */
K230ToStm32Instance *K230ToStm32Register(const K230ToStm32_Init_Config_s *config);

/**
 * @brief 启动视觉接收。
 *
 * @param instance 模块实例，可为 NULL（此时直接返回 0）。
 * @return 1 = 启动成功，0 = 失败。
 *
 * @note 上电初始化时调用一次即可，后续由驱动自动重挂接收。
 */
uint8_t K230ToStm32Start(K230ToStm32Instance *instance);

/**
 * @brief 读取最新一帧数据快照。
 *
 * @param instance 模块实例，不可为 NULL。
 * @param data_out 输出结构体，不可为 NULL。
 * @return 1 = 数据可用（坐标可信，可闭环）；0 = 不可用。
 *
 * @note 返回值恒等于 data_out->data_valid。即使返回 0，结构体中的
 *       x_px / y_px / offset_* 等调试字段仍然是有效的，方便区分
 *       「目标丢失」和「视觉离线」两种情况。
 *
 * @note 返回 0 时目标丢失或视觉离线，控制环必须停止跟踪、
 *       按安全策略处理，禁止沿用上一次的坐标（开发规范 §12.1）。
 */
uint8_t K230ToStm32GetData(K230ToStm32Instance *instance, K230_Data_s *data_out);

/**
 * @brief 查询视觉链路是否在线（超时判定）。
 *
 * @param instance 模块实例，可为 NULL。
 * @return 1 = 在线（K230_VISION_TIMEOUT_MS 内收到过合法帧），0 = 离线。
 */
uint8_t K230ToStm32IsOnline(K230ToStm32Instance *instance);

/**
 * @brief 读取模块统计。
 *
 * @param instance  模块实例，不可为 NULL。
 * @param stats_out 输出结构体，不可为 NULL。
 */
void K230ToStm32GetStats(K230ToStm32Instance *instance, K230ToStm32_Stats_s *stats_out);

#ifdef __cplusplus
}
#endif

#endif /* K230_TO_STM32_H */
