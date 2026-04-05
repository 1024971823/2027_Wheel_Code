#ifndef BSP_RC_H
#define BSP_RC_H

#include "struct_typedef.h"
#include <stdbool.h>

/*
 * bsp_rc only keeps remote-control BSP-side buffer / callback management.
 * It does not configure UART, DMA or interrupts.
 *
 * The lower layer should pass raw receiver bytes into RC_Process_Received_Data().
 *
 * DJI DBUS on STM32H723:
 *   protocol side: 100k, 8E1, inverted
 *   HAL config side: 9-bit word length + even parity
 *
 * Reason:
 *   On STM32H7 HAL, parity occupies one hardware bit in the word length field.
 *   If configured as 8-bit + even parity, effective payload becomes 7-bit and
 *   the 18-byte DBUS frame will decode incorrectly.
 */

#define BSP_RC_DJI_DBUS_FRAME_LENGTH 18u

typedef void (*RC_Frame_Handler)(uint8_t *buffer, uint16_t length);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化遥控 BSP 缓冲区信息
 * @param rx1_buf 底层接收缓冲区 0
 * @param rx2_buf 底层接收缓冲区 1
 * @param dma_buf_num 单个缓冲区长度
 */
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

/**
 * @brief 禁用遥控 BSP
 */
void RC_Disable(void);

/**
 * @brief 兼容旧接口，等价于 RC_Disable()
 */
void RC_unable(void);

/**
 * @brief 重新启用遥控 BSP，可选择更新缓冲区长度
 * @param dma_buf_num 新长度，传 0 表示保持原长度
 */
void RC_Restart(uint16_t dma_buf_num);

/**
 * @brief 兼容旧接口，等价于 RC_Restart()
 */
void RC_restart(uint16_t dma_buf_num);

/**
 * @brief 注册接收完成后的帧处理回调
 * @param handler 回调函数，可为 NULL
 */
void RC_Register_Frame_Handler(RC_Frame_Handler handler);

/**
 * @brief 兼容旧接口，当前不处理底层中断
 * @note  底层 IRQ 应由 UART/DMA 驱动自行处理
 */
void RC_IRQHandler(void);

/**
 * @brief 将底层收到的一帧数据送入遥控 BSP
 * @param buffer 原始数据缓冲区
 * @param length 有效长度
 */
void RC_Process_Received_Data(uint8_t *buffer, uint16_t length);

/**
 * @brief 获取当前是否处于启用状态
 * @return true 已启用
 * @return false 未启用
 */
bool RC_Is_Enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_RC_H */
