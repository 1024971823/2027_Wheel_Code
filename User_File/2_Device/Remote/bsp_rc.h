/******************************************************************************
 * @file    bsp_rc.h
 * @brief   遥控器板级支持包 —— UART5 + DMA（适配达妙MC02开发板 STM32H723）
 * @note    根据 robot_param.h 中 __RC_TYPE 自动适配：
 *          - RC_DT7  : DJI DBUS  100000-8E1, 18字节帧, 信号非反相
 *          - RC_SBUS : 标准SBUS  100000-8E2, 25字节帧, 信号反相(板载硬件反相)
 *          UART5 的 DMA 接收由 drv_uart 中间件管理，本模块负责：
 *          1. 针对不同遥控器协议修正 UART5 硬件参数(停止位/RX反相)
 *          2. 通过 drv_uart 注册接收回调并启动 DMA
 *          3. 提供关闭/重启接口用于错误恢复和热插拔
 ******************************************************************************/
#ifndef BSP_RC_H
#define BSP_RC_H

#include "struct_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 帧长度常量 ---- */
#define RC_DT7_FRAME_LENGTH     18u   /* DJI DT7 DBUS 帧长 */
#define RC_SBUS_FRAME_LENGTH    25u   /* 标准 SBUS 帧长   */

/* ---- 回调函数类型 ---- */
typedef void (*RC_RxCallback_t)(uint8_t *buffer, uint16_t length);

/**
 * @brief  初始化遥控器接收（UART5 + DMA）
 * @param  callback  每收到一帧调用的回调函数（在中断上下文执行）
 * @note   内部会根据 __RC_TYPE 自动调整 UART5 的停止位与 RX 反相配置，
 *         随后通过 drv_uart 中间件注册回调并启动 DMA 接收。
 */
void RC_Init(RC_RxCallback_t callback);

/**
 * @brief  关闭遥控器的 UART5 DMA 接收
 */
void RC_Disable(void);

/**
 * @brief  重启遥控器的 UART5 DMA 接收（用于错误恢复 / 热插拔）
 */
void RC_Restart(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_RC_H */
