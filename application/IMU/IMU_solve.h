/**
 ******************************************************************************
 * @file    IMU_solve.h
 * @brief   IMU姿态解算接口 - 适配达妙MC02开发板
 *          通过BSP_BMI088的EKF实现姿态解算，无需独立的kalman_filter库
 * @note    BSP_BMI088内部已集成四元数EKF与卡方检验
 *          本模块仅作为兼容接口层，供其他模块调用
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     2025-04-05      Penguin         1. done
 *  V2.0.0     2026-04-06      Copilot         1. 适配达妙MC02
 *                                             2. 移除kalman_filter.h依赖
 *                                             3. 委托BSP_BMI088进行EKF解算
 ******************************************************************************
 @verbatim
 ==============================================================================
 详细原理参考王工的文章：
    四元数EKF姿态更新算法 https://zhuanlan.zhihu.com/p/454155643

 ==============================================================================
 @endverbatim
*/

#ifndef __IMU_SOLVE_H
#define __IMU_SOLVE_H

#include "struct_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

extern float GetEkfAngle(int i);
extern float GetEkfAccel(int i);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_SOLVE_H */
/*------------------------------ End of File ------------------------------*/
