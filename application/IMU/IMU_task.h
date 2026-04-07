/**
  ****************************(C) COPYRIGHT 2025 PolarBear****************************
  * @file       IMU_task.cpp/h
  * @brief      IMU任务，适配达妙MC02开发板(STM32H723)
  *             使用BSP_BMI088驱动(SPI2)与板载EKF姿态解算
  * @note       达妙板无磁力计IST8310，已移除相关代码
  *             BMI088的初始化与EKF解算已由BSP_BMI088在定时器回调中完成
  *             本任务仅负责将BSP数据转发至Publish/Subscribe系统
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088
  *  V3.0.0     Apr-05-2025     Penguin         1. 采用EKF解算
  *  V4.0.0     Apr-06-2026     Copilot         1. 适配达妙MC02开发板(H7)
  *                                             2. 移除IST8310磁力计依赖
  *                                             3. 使用BSP_BMI088 C++驱动(SPI2)
  *                                             4. 移除F407 SPI1 DMA手动管理
  *                                             5. 移除旧版kalman_filter依赖
  *
  ****************************(C) COPYRIGHT 2025 PolarBear****************************
  */

#ifndef INS_Task_H
#define INS_Task_H

#include "struct_typedef.h"
#include "data_exchange.h"

#ifdef __cplusplus
extern "C" {
#endif

#define INS_TASK_INIT_TIME 7  // 任务开始初期 delay 一段时间(ms)

extern void IMU_task(void const *pvParameters);

extern const fp32 *get_INS_angle_point(void);
extern const fp32 *get_gyro_data_point(void);
extern const fp32 *get_accel_data_point(void);

#ifdef __cplusplus
}
#endif

#endif
