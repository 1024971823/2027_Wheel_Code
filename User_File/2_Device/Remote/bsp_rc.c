/**
 * @file    bsp_rc.c
 * @brief   遥控器 BSP 适配层
 * @note    本文件不负责配置 UART / DMA / IRQ，只负责：
 *          1. 记录底层缓冲区信息
 *          2. 管理启停状态
 *          3. 将底层收到的数据转发给上层回调
 */

#include "bsp_rc.h"

#include <stddef.h>

/* 底层接收双缓冲区指针，仅做记录，不直接驱动 DMA */
static uint8_t *rc_rx_buf0 = NULL;
static uint8_t *rc_rx_buf1 = NULL;

/* 单个接收缓冲区长度 */
static uint16_t rc_buffer_length = 0u;

/* 上层帧处理回调 */
static RC_Frame_Handler rc_frame_handler = NULL;

/* 软件使能标志 */
static bool rc_enabled = false;

/**
 * @brief 检查当前缓冲区配置是否有效
 * @return true 有效
 * @return false 无效
 */
static bool RC_Buffer_Config_Is_Valid(void)
{
    return (rc_rx_buf0 != NULL && rc_rx_buf1 != NULL && rc_buffer_length != 0u);
}

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    if (rx1_buf == NULL || rx2_buf == NULL || dma_buf_num == 0u)
    {
        rc_rx_buf0 = NULL;
        rc_rx_buf1 = NULL;
        rc_buffer_length = 0u;
        rc_enabled = false;
        return;
    }

    rc_rx_buf0 = rx1_buf;
    rc_rx_buf1 = rx2_buf;
    rc_buffer_length = dma_buf_num;
    rc_enabled = true;
}

void RC_Disable(void)
{
    rc_enabled = false;
}

void RC_unable(void)
{
    RC_Disable();
}

void RC_Restart(uint16_t dma_buf_num)
{
    if (dma_buf_num != 0u)
    {
        rc_buffer_length = dma_buf_num;
    }

    if (!RC_Buffer_Config_Is_Valid())
    {
        rc_enabled = false;
        return;
    }

    rc_enabled = true;
}

void RC_restart(uint16_t dma_buf_num)
{
    RC_Restart(dma_buf_num);
}

void RC_Register_Frame_Handler(RC_Frame_Handler handler)
{
    rc_frame_handler = handler;
}

void RC_IRQHandler(void)
{
    /* 保留旧接口，仅用于兼容旧工程调用。 */
}

void RC_Process_Received_Data(uint8_t *buffer, uint16_t length)
{
    if (!rc_enabled)
    {
        return;
    }

    if (!RC_Buffer_Config_Is_Valid())
    {
        return;
    }

    if (buffer == NULL || length == 0u)
    {
        return;
    }

    if (length > rc_buffer_length)
    {
        return;
    }

    /* 仅允许底层登记过的双缓冲区进入 BSP 处理链，避免误喂其他来源的数据 */
    if (buffer != rc_rx_buf0 && buffer != rc_rx_buf1)
    {
        return;
    }

    if (rc_frame_handler != NULL)
    {
        rc_frame_handler(buffer, length);
    }
}

bool RC_Is_Enabled(void)
{
    return rc_enabled;
}
