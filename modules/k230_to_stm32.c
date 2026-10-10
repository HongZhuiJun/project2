/**
 * @file k230_to_stm32.c
 * @brief Module 层：K230 视觉帧接收、校验与解析（人体中心坐标）。
 *
 * 解析器沿用逐字节状态机的写法：帧头匹配 → 收满载荷 → 校验 → 更新数据。
 * 状态机天然抗错位（失配后能重新同步），且一次回调收到多帧或半帧都能正确处理。
 *
 * 并发说明（开发规范 §8.3）：
 *   - 解析状态机（parse_index / frame_buf）只被接收中断访问，
 *     主循环不读，因此不需要保护。
 *   - 数据快照 data 由接收中断写、主循环读，属于单写多读，
 *     读取侧用临界区保护（见 K230ToStm32GetData）。
 */
#include "k230_to_stm32.h"
#include <string.h>

/* 帧头字节表，供状态机逐字节比对 */
static const uint8_t k230_frame_header[K230_FRAME_HEADER_LEN] = {
    K230_FRAME_HEADER_0,
    K230_FRAME_HEADER_1,
    K230_FRAME_HEADER_2,
    K230_FRAME_HEADER_3,
};

struct K230ToStm32Instance {
    K230UsartInstance *usart_instance;

    /* 解析状态机：仅接收中断访问 */
    uint8_t parse_index;                   /* 已匹配/已接收的字节数，0 表示等待帧头 */
    uint8_t frame_buf[K230_FRAME_LEN];     /* 整帧暂存 */
    uint8_t rx_buf[K230_RX_BUF_SIZE];      /* BSP 层接收缓冲，由本实例持有 */

    /* 数据快照：中断写，主循环读 */
    volatile K230_Data_s data;

    /* 统计量：仅接收中断写，主循环读单字是原子的 */
    volatile uint32_t frame_count;
    volatile uint32_t checksum_error_count;
    volatile uint32_t lost_frame_count;
};

/* 实例池：静态分配，控制周期内不做动态内存（开发规范 §7.3） */
static K230ToStm32Instance k230_to_stm32_pool[K230_TO_STM32_MAX_INSTANCE];

/* k230_to_stm32_Init() 成功注册的实例。调用方（main）不再持有这个指针，
   后续取数据 / 查在线状态的接口要拿它，所以在这里留一份 */
static K230ToStm32Instance *k230_to_stm32_instance = NULL;

/* ============================ 内部函数 ============================ */

/**
 * @brief 从字节流读取 int16（大端序）。
 */
static int16_t K230ToStm32ReadInt16BE(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

/**
 * @brief 判断像素坐标是否落在检测画面内。
 *
 * @param x_px 人体框中心像素 X。
 * @param y_px 人体框中心像素 Y。
 * @return 1 = 有效，0 = 无效（目标丢失或误检）。
 *
 * @note 任何负值都按「未检测到」处理：协议用 -1 表示丢失，
 *       即便上位机将来用别的负值表示其他异常，也不会被当成有效坐标
 *       送进控制环。超出画面范围的坐标直接丢弃而不是限幅 ——
 *       限幅会把误检变成一个「看起来合理」的坐标，控制环会去追它。
 */
static uint8_t K230ToStm32IsCoordValid(int16_t x_px, int16_t y_px)
{
    if ((x_px < 0) || (y_px < 0)) {
        return 0u;
    }

    if (((int32_t)x_px >= K230_IMAGE_WIDTH_PX) ||
        ((int32_t)y_px >= K230_IMAGE_HEIGHT_PX)) {
        return 0u;
    }

    return 1u;
}

/**
 * @brief 校验并处理一整帧。
 *
 * @note 在接收中断中执行，必须保持短小（开发规范 §8.1）。
 */
static void K230ToStm32HandleFrame(K230ToStm32Instance *instance)
{
    /* 校验：前 12 字节逐字节异或 */
    uint8_t checksum = 0u;
    for (uint8_t i = 0u; i < K230_FRAME_CHECKSUM_INDEX; i++) {
        checksum ^= instance->frame_buf[i];
    }

    if (checksum != instance->frame_buf[K230_FRAME_CHECKSUM_INDEX]) {
        /* 校验失败整帧丢弃。宁可少一帧数据，也不能把可疑数据交给控制环 */
        instance->checksum_error_count++;
        return;
    }

    int16_t x_px = K230ToStm32ReadInt16BE(&instance->frame_buf[K230_FRAME_X_INDEX]);
    int16_t y_px = K230ToStm32ReadInt16BE(&instance->frame_buf[K230_FRAME_Y_INDEX]);

    instance->frame_count++;
    instance->data.x_px               = x_px;
    instance->data.y_px               = y_px;
    instance->data.last_frame_time_ms = HAL_GetTick();

    if (K230ToStm32IsCoordValid(x_px, y_px) != 0u) {
        instance->data.offset_x_px     = (int16_t)(x_px - (K230_IMAGE_WIDTH_PX / 2));
        instance->data.offset_y_px     = (int16_t)(y_px - (K230_IMAGE_HEIGHT_PX / 2));
        instance->data.target_detected = 1u;
        instance->data.data_valid      = 1u;
    } else {
        /* 目标丢失：必须清掉 data_valid。保留 offset_* 旧值只是为了联调时
           能看到人是在哪里丢的，上层一律以 data_valid 为准 */
        instance->data.target_detected = 0u;
        instance->data.data_valid      = 0u;
        instance->lost_frame_count++;
    }
}

/**
 * @brief 逐字节喂入解析状态机。
 *
 * parse_index 表示「已经收了多少字节」：
 *   0 ~ 3  : 正在匹配 4 字节帧头。失配时回退重试当前字节，这样即使出现
 *            ... AA AA 55 01 08 ... 这种帧头被前缀污染的流，也不会漏掉
 *            后面那一帧的帧头。
 *   4 ~ 11 : 正在收集 8 字节载荷。
 *   12     : 收到校验字节，交给 K230ToStm32HandleFrame() 处理。
 *
 * @note 在接收中断中执行，必须保持短小（开发规范 §8.1）。
 */
static void K230ToStm32FeedByte(K230ToStm32Instance *instance, uint8_t byte)
{
    /* ---- 阶段一：匹配帧头 ---- */
    if (instance->parse_index < K230_FRAME_HEADER_LEN) {
        if (byte == k230_frame_header[instance->parse_index]) {
            instance->frame_buf[instance->parse_index] = byte;
            instance->parse_index++;
        } else {
            /* 失配，重新开始同步。当前字节本身可能是新帧头的第一个字节，
               所以要用 header[0] 再比对一次，不能直接丢弃 */
            instance->parse_index = 0u;
            if (byte == k230_frame_header[0]) {
                instance->frame_buf[0] = byte;
                instance->parse_index  = 1u;
            }
        }
        return;
    }

    /* ---- 阶段二：收集载荷与校验字节 ---- */
    instance->frame_buf[instance->parse_index] = byte;
    instance->parse_index++;

    if (instance->parse_index < K230_FRAME_LEN) {
        return;
    }

    /* 一帧收齐，先复位状态机再处理，保证下一字节重新从帧头开始匹配 */
    instance->parse_index = 0u;
    K230ToStm32HandleFrame(instance);
}

/**
 * @brief BSP 接收回调，**在串口中断上下文中执行**。
 *
 * @param usart_instance 触发的 BSP 实例。
 * @param data           本次收到的数据块。
 * @param len            字节数。
 */
static void K230ToStm32OnRxData(K230UsartInstance *usart_instance, const uint8_t *data, uint16_t len)
{
    if ((usart_instance == NULL) || (data == NULL) || (len == 0u)) {
        return;
    }

    /* BSP 层通过 parent 指针回传上层实例，BSP 不需要包含本模块头文件
       （开发规范 §6.4） */
    K230ToStm32Instance *instance = (K230ToStm32Instance *)usart_instance->parent;
    if (instance == NULL) {
        return;
    }

    for (uint16_t i = 0u; i < len; i++) {
        K230ToStm32FeedByte(instance, data[i]);
    }
}

/* ============================ 外部接口 ============================ */

uint8_t k230_to_stm32_Init(UART_HandleTypeDef *uart_handle)
{
    K230ToStm32_Init_Config_s config = {
        .uart_handle = uart_handle,
    };

    /* 传入 NULL 时 K230ToStm32Register() 会直接返回 NULL，统一走下面的失败分支 */
    k230_to_stm32_instance = K230ToStm32Register(&config);
    if (k230_to_stm32_instance == NULL) {
        return 0u;
    }

    return K230ToStm32Start(k230_to_stm32_instance);
}

/**
 * @brief 获取 k230_to_stm32_Init() 创建的默认 K230 实例。
 *
 * @return 默认实例指针；尚未初始化或初始化失败时返回 NULL。
 */
K230ToStm32Instance *K230ToStm32GetInstance(void)
{
    return k230_to_stm32_instance;
}

K230ToStm32Instance *K230ToStm32Register(const K230ToStm32_Init_Config_s *config)
{
    if ((config == NULL) || (config->uart_handle == NULL)) {
        return NULL;
    }

    K230ToStm32Instance *instance = NULL;
    for (uint8_t i = 0u; i < K230_TO_STM32_MAX_INSTANCE; i++) {
        if (k230_to_stm32_pool[i].usart_instance == NULL) {
            instance = &k230_to_stm32_pool[i];
            break;
        }
    }
    if (instance == NULL) {
        return NULL; /* 实例池已满 */
    }

    memset(instance, 0, sizeof(K230ToStm32Instance));

    /* 把本模块挂到 BSP 层：数据到达后由 BSP 回调 K230ToStm32OnRxData */
    K230Usart_Init_Config_s usart_config = {
        .uart_handle = config->uart_handle,
        .rx_buf      = instance->rx_buf,
        .rx_buf_size = K230_RX_BUF_SIZE,
        .parent      = instance,
        .rx_callback = K230ToStm32OnRxData,
    };

    instance->usart_instance = K230UsartRegister(&usart_config);
    if (instance->usart_instance == NULL) {
        return NULL;
    }

    return instance;
}

uint8_t K230ToStm32Start(K230ToStm32Instance *instance)
{
    if (instance == NULL) {
        return 0u;
    }

    return (K230UsartStart(instance->usart_instance) == HAL_OK) ? 1u : 0u;
}

/**
 * @brief 按给定的帧计数与时间戳判断链路是否在线。
 *
 * @note 抽成独立函数是为了让「读取快照」和「实时查询」两条路径共用同一套
 *       超时判据，避免两处阈值逻辑各写一遍后走偏。
 */
static uint8_t K230ToStm32IsOnlineAt(uint32_t frame_count, uint32_t last_frame_time_ms)
{
    /* 开机后一帧都没收到过，不能算在线 */
    if (frame_count == 0u) {
        return 0u;
    }

    /* 无符号相减，HAL_GetTick() 回绕（约 49.7 天）时结果依然正确 */
    uint32_t elapsed_ms = HAL_GetTick() - last_frame_time_ms;

    return (elapsed_ms <= K230_VISION_TIMEOUT_MS) ? 1u : 0u;
}

uint8_t K230ToStm32GetData(K230ToStm32Instance *instance, K230_Data_s *data_out)
{
    if ((instance == NULL) || (data_out == NULL)) {
        return 0u;
    }

    /* 单写(中断)多读(主循环)：读取期间关中断，避免拷到半新半旧的快照。
       临界区内只有一次结构体赋值，无延时（开发规范 §8.2） */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *data_out = instance->data;
    __set_PRIMASK(primask);

    /* 用快照自己的时间戳判在线，保证返回值与快照出自同一时刻 */
    if (K230ToStm32IsOnlineAt(instance->frame_count, data_out->last_frame_time_ms) == 0u) {
        /* 链路超时后坐标不再可信：即使最后一帧仍检测到人，也必须把
           data_valid 清掉，否则控制环会拿着过期坐标闭环（规范 §12.1） */
        data_out->data_valid      = 0u;
        data_out->target_detected = 0u;
    }

    return data_out->data_valid;
}

uint8_t K230ToStm32IsOnline(K230ToStm32Instance *instance)
{
    if (instance == NULL) {
        return 0u;
    }

    return K230ToStm32IsOnlineAt(instance->frame_count, instance->data.last_frame_time_ms);
}

void K230ToStm32GetStats(K230ToStm32Instance *instance, K230ToStm32_Stats_s *stats_out)
{
    if ((instance == NULL) || (stats_out == NULL)) {
        return;
    }

    stats_out->frame_count          = instance->frame_count;
    stats_out->checksum_error_count = instance->checksum_error_count;
    stats_out->lost_frame_count     = instance->lost_frame_count;
}
