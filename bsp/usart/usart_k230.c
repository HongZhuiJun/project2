/**
 * @file usart_k230.c
 * @brief BSP 层：K230 视觉串口「空闲中断不定长接收」驱动实现。
 *
 * 三种接收方式，差异都集中在本文件，上层无感：
 *   - 中断(IT)：没挂 RX DMA 时使用。HAL 用 RXNE 收字节、IDLE 收尾，
 *     回调里的 Size 就是本段收到的字节数，交付后立刻重挂。
 *   - DMA(Normal)：每次事件后 DMA 停止、缓冲归零，Size 就是本段字节数，
 *     交付后立刻重挂（见 K230UsartRearm）。
 *   - DMA(Circular)：DMA 永不停，缓冲循环覆盖。此时 HAL 回调传入的 Size
 *     是**写指针位置**（累计），不是本次增量，需要自己维护读指针算增量
 *     （见 K230UsartDeliverCircular）。
 */
#include "usart_k230.h"
#include <string.h>

/* 实例池：静态分配，控制周期内不做动态内存（开发规范 §7.3） */
static K230UsartInstance k230_usart_pool[K230_USART_MAX_INSTANCE];

/* ============================ 内部函数 ============================ */

/**
 * @brief 由 UART 句柄反查实例。
 *
 * @note HAL 的接收回调只传回 UART 句柄，不带用户数据，所以这里必须
 *       自己维护句柄到实例的映射（开发规范 §6.4 的 parent 指针思路）。
 */
static K230UsartInstance *K230UsartFindInstance(UART_HandleTypeDef *uart_handle)
{
    if (uart_handle == NULL) {
        return NULL;
    }

    for (uint8_t i = 0u; i < K230_USART_MAX_INSTANCE; i++) {
        if (k230_usart_pool[i].uart_handle == uart_handle) {
            return &k230_usart_pool[i];
        }
    }

    return NULL;
}

/**
 * @brief 把一段数据交给上层，并记统计。
 *
 * @note 在中断上下文执行。上层回调可能被本函数连续调用多次
 *       （Circular 模式回绕时一次事件拆成两段），所以上层回调必须是
 *       可重入的字节流消费者。
 */
static void K230UsartDeliver(K230UsartInstance *instance, const uint8_t *data, uint16_t len)
{
    if (len == 0u) {
        return;
    }

    instance->rx_chunk_count++;
    instance->rx_byte_count += len;

    if (instance->rx_callback != NULL) {
        instance->rx_callback(instance, data, len);
    }
}

/**
 * @brief 重新挂载接收（中断 / DMA-Normal 模式使用）。
 *
 * @return HAL_OK 表示重挂成功；失败会累加到 error_count，供上层发现。
 *
 * @note HAL 在调用 HAL_UARTEx_RxEventCallback() / HAL_UART_ErrorCallback()
 *       之前已经把 RxState 复位成 READY 并结束了上一轮接收，
 *       所以在这两个回调里可以直接重挂，不需要先 Abort。
 * @note Circular 模式 DMA 不会停，不需要（也不应该）调用本函数。
 */
static HAL_StatusTypeDef K230UsartRearm(K230UsartInstance *instance)
{
    HAL_StatusTypeDef status;

    if (instance->rx_mode == K230_USART_RX_MODE_IT) {
        status = HAL_UARTEx_ReceiveToIdle_IT(instance->uart_handle,
                                             instance->rx_buf,
                                             instance->rx_buf_size);
    } else {
        status = HAL_UARTEx_ReceiveToIdle_DMA(instance->uart_handle,
                                              instance->rx_buf,
                                              instance->rx_buf_size);
    }

    if (status != HAL_OK) {
        /* 重挂失败说明串口状态异常，计数后由上层通过 error_count 发现 */
        instance->error_count++;
    }

    return status;
}

/**
 * @brief Circular 模式下的增量交付。
 *
 * @param instance  实例。
 * @param write_pos 写指针位置（HAL 传入的 Size，即已写入字节数对缓冲取模）。
 *
 * @note Circular 模式下 DMA 一直在跑，HAL 给的 Size 是写指针位置而不是
 *       本次增量，因此必须自己记住上次消费到哪里（rx_read_pos）。
 * @note 回绕时分两段交付：先缓冲尾部，再缓冲头部。
 */
static void K230UsartDeliverCircular(K230UsartInstance *instance, uint16_t write_pos)
{
    if (write_pos > instance->rx_buf_size) {
        /* 正常情况下不会发生。真发生说明 HAL 的语义变了，
           这里宁可丢一次数据也不能越界读 */
        instance->error_count++;
        return;
    }

    uint16_t read_pos = instance->rx_read_pos;
    if (write_pos == read_pos) {
        /* 没有新数据。半传输/传输完成事件与空闲事件落在同一位置时会走到这里 */
        return;
    }

    if (write_pos > read_pos) {
        K230UsartDeliver(instance, &instance->rx_buf[read_pos],
                         (uint16_t)(write_pos - read_pos));
    } else {
        /* 已回绕 */
        K230UsartDeliver(instance, &instance->rx_buf[read_pos],
                         (uint16_t)(instance->rx_buf_size - read_pos));
        K230UsartDeliver(instance, instance->rx_buf, write_pos);
    }

    /* 传输完成事件时 HAL 传的 Size 等于缓冲全长，这里归一化到 0，
       让读指针始终落在 [0, buf_size) 内。语义不变：位置 buf_size 与 0
       指向同一处，下一次事件照常算出增量。 */
    instance->rx_read_pos = (write_pos == instance->rx_buf_size) ? 0u : write_pos;
}

/* ============================ 外部接口 ============================ */

K230UsartInstance *K230UsartRegister(const K230Usart_Init_Config_s *config)
{
    if ((config == NULL) ||
        (config->uart_handle == NULL) ||
        (config->rx_buf == NULL) ||
        (config->rx_buf_size == 0u)) {
        return NULL;
    }

    K230UsartInstance *instance = NULL;
    for (uint8_t i = 0u; i < K230_USART_MAX_INSTANCE; i++) {
        if (k230_usart_pool[i].uart_handle == NULL) {
            instance = &k230_usart_pool[i];
            break;
        }
    }
    if (instance == NULL) {
        return NULL; /* 实例池已满 */
    }

    memset(instance, 0, sizeof(K230UsartInstance));

    instance->uart_handle = config->uart_handle;
    instance->dma_handle  = config->uart_handle->hdmarx;
    instance->rx_buf      = config->rx_buf;
    instance->rx_buf_size = config->rx_buf_size;
    instance->parent      = config->parent;
    instance->rx_callback = config->rx_callback;

    /* 接收方式在这里判定并缓存，中断里就不必反复读结构体 */
    if (config->uart_handle->hdmarx == NULL) {
        instance->rx_mode = K230_USART_RX_MODE_IT;
    } else if (config->uart_handle->hdmarx->Init.Mode == DMA_CIRCULAR) {
        instance->rx_mode = K230_USART_RX_MODE_DMA_CIRCULAR;
    } else {
        instance->rx_mode = K230_USART_RX_MODE_DMA_NORMAL;
    }

    return instance;
}

HAL_StatusTypeDef K230UsartStart(K230UsartInstance *instance)
{
    if (instance == NULL) {
        return HAL_ERROR;
    }

    /* 清掉可能残留的空闲/过载标志。若启动瞬间 SR 里带着 ORE，
       HAL 会认为接收已被中止并直接返回 HAL_ERROR */
    __HAL_UART_CLEAR_IDLEFLAG(instance->uart_handle);
    __HAL_UART_CLEAR_OREFLAG(instance->uart_handle);

    /* 从缓冲起点重新开始计数，读指针必须一起归零 */
    instance->rx_read_pos = 0u;

    return K230UsartRearm(instance);
}

void K230UsartGetStats(const K230UsartInstance *instance, K230Usart_Stats_s *stats_out)
{
    if ((instance == NULL) || (stats_out == NULL)) {
        return;
    }

    stats_out->rx_chunk_count = instance->rx_chunk_count;
    stats_out->rx_byte_count  = instance->rx_byte_count;
    stats_out->error_count    = instance->error_count;
}

/* ==========================================================================
 * HAL 回调：本文件是 BSP 层，允许直接接触 HAL 中断回调（开发规范 §2.4）
 * ========================================================================== */

/**
 * @brief 接收事件回调：空闲中断（IDLE）、半传输（HT）或传输完成（TC）时触发。
 *
 * @param huart 触发事件的串口句柄。
 * @param Size  含义随接收方式而变：
 *              Circular DMA 下是「写指针位置」（累计，非增量）；
 *              中断 / Normal DMA 下是「本段收到的字节数」。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    K230UsartInstance *instance = K230UsartFindInstance(huart);
    if (instance == NULL) {
        return;
    }

    if (instance->rx_mode == K230_USART_RX_MODE_DMA_CIRCULAR) {
        /* DMA 不会停，不需要重挂；按写指针位置算增量 */
        K230UsartDeliverCircular(instance, Size);
    } else {
        /* 中断 / Normal DMA：本次事件后接收已结束、缓冲即将被下次接收覆盖，
           所以必须先交付、再重挂 */
        if (Size > 0u) {
            K230UsartDeliver(instance, instance->rx_buf, Size);
        }
        (void)K230UsartRearm(instance);
    }
}

/**
 * @brief 串口错误回调，在 DMA 中止完成后被调用。
 *
 * @note 过载(ORE)/帧错误/噪声错误都会让 HAL 结束本轮接收 —— 这一点
 *       两种 DMA 模式都一样，所以这里**必须无条件重挂**，否则一次线路
 *       干扰就足以让视觉数据永久停更（规范 §12.1 的 fail-safe）。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    K230UsartInstance *instance = K230UsartFindInstance(huart);
    if (instance == NULL) {
        return;
    }

    instance->error_count++;

    /* 清掉过载标志，否则 HAL 会把接下来的启动直接判为失败 */
    __HAL_UART_CLEAR_OREFLAG(huart);

    /* 重新从缓冲起点接收，读指针一起归零，避免把旧内容当成新数据 */
    instance->rx_read_pos = 0u;
    (void)K230UsartRearm(instance);
}
