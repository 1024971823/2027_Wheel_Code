/**
 ******************************************************************************
 * @file    IMU_solve.cpp
 * @brief   IMU姿态解算兼容接口 - 适配达妙MC02开发板
 *          BSP_BMI088内部已集成四元数EKF与卡方检验
 *          本模块仅提供兼容接口，供老代码调用
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     2025-04-05      Penguin         1. done
 *  V2.0.0     2026-04-06      Copilot         1. 适配达妙MC02
 *                                             2. 移除kalman_filter.h依赖
 *                                             3. 委托BSP_BMI088进行EKF解算
 ******************************************************************************
 */

#include "IMU_solve.h"
#include "bsp_bmi088.h"

/**
 * @brief  获取EKF解算后的欧拉角
 * @param  i: 0=Yaw, 1=Pitch, 2=Roll
 * @retval (rad) 欧拉角
 */
float GetEkfAngle(int i)
{
    auto euler = BSP_BMI088.Get_Euler_Angle();
    if (i >= 0 && i < 3) return euler[i][0];
    return 0.0f;
}

/**
 * @brief  获取大地坐标系下的加速度(已去除重力)
 * @param  i: 0=X, 1=Y, 2=Z
 * @retval (m/s^2) 加速度
 */
float GetEkfAccel(int i)
{
    auto accel = BSP_BMI088.Get_Accel();
    if (i >= 0 && i < 3) return accel[i][0];
    return 0.0f;
}

/*------------------------------ End of File ------------------------------*/
