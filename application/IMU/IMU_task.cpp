/**
  ****************************(C) COPYRIGHT 2025 PolarBear****************************
  * @file       IMU_task.cpp
  * @brief      IMU任务，适配达妙MC02开发板(STM32H723)
  *             使用BSP_BMI088驱动(SPI2)与板载EKF姿态解算
  * @note       达妙板无磁力计IST8310，已移除相关代码
  *             BMI088的初始化与EKF解算由BSP_BMI088在定时器回调中完成
  *             (TIM4 10us / TIM8 125us / TIM7 128ms)
  *             本任务仅负责将BSP数据转发至Publish/Subscribe系统
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088
  *  V3.0.0     Apr-05-2025     Penguin         1. 采用EKF解算
  *  V4.0.0     Apr-06-2026     Copilot         1. 适配达妙MC02(H7)
  *                                             2. 移除IST8310/SPI1 DMA
  *                                             3. 委托BSP_BMI088完成解算
  *
  ****************************(C) COPYRIGHT 2025 PolarBear****************************
  */

#include "IMU_task.h"

#include "IMU.h"
#include "bsp_bmi088.h"
#include "cmsis_os.h"
#include "data_exchange.h"
#include "macro_typedef.h"
#include "robot_param.h"

// IMU 数据更新周期 (ms)
#define IMU_TASK_PERIOD 1

// 内部数据
static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};

static Imu_t IMU_DATA = {};

/**
 * @brief 从BSP_BMI088中提取数据，更新到Imu_t结构体
 */
static void UpdateImuData(void)
{
    // 获取欧拉角 (Yaw-Pitch-Roll)
    auto euler = BSP_BMI088.Get_Euler_Angle();
    INS_angle[AX_X] = euler[2][0];  // Roll
    INS_angle[AX_Y] = euler[1][0];  // Pitch
    INS_angle[AX_Z] = euler[0][0];  // Yaw

    // 获取机体坐标系角速度
    auto gyro_body = BSP_BMI088.Get_Gyro_Body();
    INS_gyro[AX_X] = gyro_body[0][0];
    INS_gyro[AX_Y] = gyro_body[1][0];
    INS_gyro[AX_Z] = gyro_body[2][0];

    // 获取大地坐标系加速度(已去除重力)
    auto accel = BSP_BMI088.Get_Accel();
    INS_accel[AX_X] = accel[0][0];
    INS_accel[AX_Y] = accel[1][0];
    INS_accel[AX_Z] = accel[2][0];

    // 填充 Imu_t 结构体
    IMU_DATA.angle[AX_X] = INS_angle[AX_X];
    IMU_DATA.angle[AX_Y] = INS_angle[AX_Y];
    IMU_DATA.angle[AX_Z] = INS_angle[AX_Z];

    IMU_DATA.gyro[AX_X] = INS_gyro[AX_X];
    IMU_DATA.gyro[AX_Y] = INS_gyro[AX_Y];
    IMU_DATA.gyro[AX_Z] = INS_gyro[AX_Z];

    IMU_DATA.accel[AX_X] = INS_accel[AX_X];
    IMU_DATA.accel[AX_Y] = INS_accel[AX_Y];
    IMU_DATA.accel[AX_Z] = INS_accel[AX_Z];
}

/**
 * @brief IMU任务主函数
 * @note  BSP_BMI088的初始化和EKF解算由tsk_config_and_callback.cpp中的
 *        定时器回调驱动(TIM4/TIM7/TIM8)，本任务仅负责数据转发
 * @param pvParameters: NULL
 */
void IMU_task(void const *pvParameters)
{
    (void)pvParameters;

    // 发布IMU数据到Publish/Subscribe系统
    Publish(&IMU_DATA, const_cast<char *>(IMU_NAME));

    // 等待BSP_BMI088初始化完成
    osDelay(INS_TASK_INIT_TIME);

    while (1) {
        // 从BSP_BMI088读取最新姿态数据
        UpdateImuData();

        osDelay(IMU_TASK_PERIOD);
    }
}

/******************************************************************/
/* API                                                            */
/******************************************************************/

/**
 * @brief  获取欧拉角
 * @param  axis: 轴id (AX_X=roll, AX_Y=pitch, AX_Z=yaw)
 * @retval (rad) 对应轴的角度值
 */
float GetImuAngle(uint8_t axis) { return IMU_DATA.angle[axis]; }

/**
 * @brief  获取角速度
 * @param  axis: 轴id
 * @retval (rad/s) 对应轴的角速度
 */
float GetImuVelocity(uint8_t axis) { return IMU_DATA.gyro[axis]; }

/**
 * @brief  获取加速度
 * @param  axis: 轴id
 * @retval (m/s^2) 对应轴上的加速度
 */
float GetImuAccel(uint8_t axis) { return IMU_DATA.accel[axis]; }

/**
 * @brief  获取yaw轴角速度(可用于零飘补偿等)
 * @retval (rad/s)
 */
float GetYawBias(void) { return IMU_DATA.gyro[AX_Z]; }

/**
 * @brief  获取原始加速度计数据
 * @param  axis: 轴id
 * @retval (m/s^2) 原始加速度计数据
 */
float get_raw_accel(uint8_t axis)
{
    auto raw = BSP_BMI088.Get_Original_Accel();
    if (axis < 3) return raw[axis][0];
    return 0.0f;
}

/**
 * @brief  获取原始陀螺仪数据
 * @param  axis: 轴id
 * @retval (rad/s) 原始陀螺仪数据
 */
float get_raw_gyro(uint8_t axis)
{
    auto raw = BSP_BMI088.Get_Original_Gyro();
    if (axis < 3) return raw[axis][0];
    return 0.0f;
}

/**
 * @brief  获取欧拉角数组指针
 * @retval INS_angle的指针, 0:roll, 1:pitch, 2:yaw 单位 rad
 */
const fp32 *get_INS_angle_point(void) { return INS_angle; }

/**
 * @brief  获取角速度数组指针
 * @retval INS_gyro的指针, 0:x, 1:y, 2:z 单位 rad/s
 */
const fp32 *get_gyro_data_point(void) { return INS_gyro; }

/**
 * @brief  获取加速度数组指针
 * @retval INS_accel的指针, 0:x, 1:y, 2:z 单位 m/s^2
 */
const fp32 *get_accel_data_point(void) { return INS_accel; }

/*------------------------------ End of File ------------------------------*/
