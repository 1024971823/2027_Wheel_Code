/******************************************************************************
 * @file    bsp_rc.cpp
 * @brief   遥控器板级支持包 —— UART5 + DMA（适配达妙MC02开发板 STM32H723）
 * @note    使用 C++ 编译以便调用 drv_uart 中间件（C++ 实现），
 *          对外接口通过 bsp_rc.h 的 extern "C" 导出 C 链接符号。
 *
 *          【DJI DT7 与 达妙MC02 校验差异说明】
 *          ┌──────────┬───────────┬──────────┬───────┬──────────┬───────────┐
 *          │ 遥控器   │ 波特率    │ 数据位   │ 校验  │ 停止位   │ 信号极性  │
 *          ├──────────┼───────────┼──────────┼───────┼──────────┼───────────┤
 *          │ DJI DT7  │ 100000    │ 8        │ EVEN  │ 1        │ 正逻辑    │
 *          │ SBUS类   │ 100000    │ 8        │ EVEN  │ 2        │ 反相      │
 *          └──────────┴───────────┴──────────┴───────┴──────────┴───────────┘
 *          CubeMX 默认将 UART5 配置为 SBUS 参数(100000-8E2)：
 *          - 若使用 DT7，本驱动会在运行时将停止位改为 1 并关闭 RX 反相
 *          - 若使用 SBUS 类遥控器，保持默认；达妙MC02 板载硬件反相电路，
 *            无需软件开启 RX 反相
 ******************************************************************************/

#include "bsp_rc.h"
#include "drv_uart.h"
#include "usart.h"
#include "robot_param.h"

/* ---- 私有变量 ---- */

/* 用户注册的接收回调 */
static RC_RxCallback_t s_user_callback = nullptr;

/* ---- 私有函数 ---- */

/**
 * @brief  drv_uart 层回调包装器（C++ 链接）
 *         将 UART_Callback 签名转发给用户注册的 C 回调
 */
static void BSP_RC_UART5_Wrapper(uint8_t *Buffer, uint16_t Length)
{
    if (s_user_callback != nullptr)
    {
        s_user_callback(Buffer, Length);
    }
}

/* ---- 导出函数 (extern "C") ---- */

/**
 * @brief  初始化遥控器接收
 */
void RC_Init(RC_RxCallback_t callback)
{
    s_user_callback = callback;

#if (__RC_TYPE == RC_DT7)
    /*
     * DJI DT7 DBUS 协议: 100000 baud, 8 数据位, 偶校验, 1 停止位
     * CubeMX 默认为 SBUS(2 停止位), 此处修正为 1 停止位。
     * DT7 信号为正逻辑(非反相), 显式关闭 RX 引脚反相以确保兼容。
     */
    huart5.Init.StopBits = UART_STOPBITS_1;
    huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINV_INIT;
    huart5.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_DISABLE;
    HAL_UART_Init(&huart5);
#else
    /*
     * SBUS 类遥控器 (AT9S PRO / HT8A / ET08A / i6x):
     * 100000 baud, 8 数据位, 偶校验, 2 停止位 → 保持 CubeMX 默认配置
     *
     * SBUS 物理层信号为反相逻辑。达妙MC02开发板 SBUS 接口已包含
     * 硬件反相电路(74HC14 等), 因此无需软件开启 RX 反相。
     *
     * ⚠ 如果你的板子没有硬件反相器, 取消下面的注释启用 RX 软件反相:
     */
    // huart5.AdvancedInit.AdvFeatureInit  = UART_ADVFEATURE_RXINV_INIT;
    // huart5.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
    // HAL_UART_Init(&huart5);
#endif

    /* 通过 drv_uart 中间件注册回调并启动 DMA 空闲中断接收 */
    UART_Init(&huart5, BSP_RC_UART5_Wrapper);
}

/**
 * @brief  关闭遥控器 UART5 DMA 接收
 */
void RC_Disable(void)
{
    HAL_UART_DMAStop(&huart5);
}

/**
 * @brief  重启遥控器 UART5 DMA 接收
 */
void RC_Restart(void)
{
    UART_Reinit(&huart5);
}
