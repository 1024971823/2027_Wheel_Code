/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_balance.c/h
  * @brief      平衡底盘控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *  V1.0.2     Sep-16-2024     Penguin         1. 添加速度观测器并测试效果
  *  V1.0.3     Nov-20-2024     Penguin         1. 完善离地检测
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  
  @todo:
    2.添加状态清零，当运行过程中出现异常时可以手动将底盘状态清零
    3.在浮空时通过动量守恒维持底盘的平衡，并调整合适的触地姿态

  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/
#include "chassis_balance.h"
#if (CHASSIS_TYPE == CHASSIS_BALANCE)
#include "CAN_communication.h"
#include "bsp_delay.h"
#include "chassis.h"
#include "chassis_balance_extras.h"
#include "cmsis_os.h"
#include "data_exchange.h"
#include "detect_task.h"
#include "kalman_filter.h"
#include "macro_typedef.h"
#include "signal_generator.h"
#include "stdbool.h"
#include "string.h"
#include "usb_debug.h"
#include "user_lib.h"
#include "gimbal.h"
#include "IMU.h"

// 一些内部的配置
#define TAKE_OFF_DETECT 0  // 启用离地检测
#define CLOSE_LEG_LEFT 0   // 关闭左腿输出
#define CLOSE_LEG_RIGHT 0  // 关闭右腿输出
#define LIFTED_UP 0        // 被架起

// Parameters on ---------------------
#define MS_TO_S 0.001f

#define CALIBRATE_STOP_VELOCITY 0.05f  // rad/s
#define CALIBRATE_STOP_TIME 200        // ms
#define CALIBRATE_VELOCITY 2.0f        // rad/s

#define VEL_PROCESS_NOISE 25   // 速度过程噪声
#define VEL_MEASURE_NOISE 800  // 速度测量噪声
// 同时估计加速度和速度时对加速度的噪声
// 更好的方法是设置为动态,当有冲击时/加加速度大时更相信轮速
#define ACC_PROCESS_NOISE 2000  // 加速度过程噪声
#define ACC_MEASURE_NOISE 0.01  // 加速度测量噪声

#define RC_OFF_HOOK_VALUE_HOLE 650

// 支持力阈值，当支持力小于这个值时认为离地
#define TAKE_OFF_FN_THRESHOLD (3.0f)
// 触地状态切换时间阈值，当时间接触或离地时间超过这个值时切换触地状态
#define TOUCH_TOGGLE_THRESHOLD (100)
// Parameters off ---------------------

// Step definitions on ---------------------
// clang-format off
#define NORMAL_STEP        0  // 正常状态
#define JUMP_STEP_SQUST    1  // 跳跃状态——蹲下
#define JUMP_STEP_JUMP     2  // 跳跃状态——跳跃
#define JUMP_STEP_RECOVERY 3  // 跳跃状态——收腿
// clang-format on
// Step definitions on ---------------------

// Step time definitions on ---------------------
// clang-format off
#define MAX_STEP_TIME           5000  // 最大步骤时间

#define NORMAL_STEP_TIME        0  // 正常状态
#define JUMP_STEP_TIME_SQUST    50  // 跳跃状态——蹲下
#define JUMP_STEP_TIME_JUMP     50  // 跳跃状态——跳跃
#define JUMP_STEP_TIME_RECOVERY 200  // 跳跃状态——收腿
// clang-format on
// Step time definitions on ---------------------

#define rc_deadband_limit(input, output, dealine)          \
    {                                                      \
        if ((input) > (dealine) || (input) < -(dealine)) { \
            (output) = (input);                            \
        } else {                                           \
            (output) = 0;                                  \
        }                                                  \
    }

static Calibrate_s CALIBRATE = {
    .cali_cnt = 0,
    .velocity = {0.0f, 0.0f, 0.0f, 0.0f},
    .stpo_time = {0, 0, 0, 0},
    .reached = {false, false, false, false},
    .calibrated = false,
};

static Observer_t OBSERVER;

Chassis_s CHASSIS = {
    .mode = CHASSIS_OFF,
    .error_code = 0,
    .yaw_mid = 0,
    .dyaw = 0.0f,
};

int8_t TRANSITION_MATRIX[10] = {0};

/*-------------------- Publish --------------------*/

void ChassisPublish(void) { Publish(&CHASSIS.fdb.speed_vector, CHASSIS_FDB_SPEED_NAME); }

/******************************************************************/
/* Init                                                           */
/*----------------------------------------------------------------*/
/* main function:      ChassisInit                                */
/* auxiliary function: None                                       */
/******************************************************************/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void ChassisInit(void)
{
    CHASSIS.rc = get_remote_control_point();  // 获取遥控器指针
    CHASSIS.imu = Subscribe(IMU_NAME);        // 获取IMU数据指针
    /*-------------------- 初始化状态转移矩阵 --------------------*/
    TRANSITION_MATRIX[NORMAL_STEP] = NORMAL_STEP;
    TRANSITION_MATRIX[JUMP_STEP_SQUST] = JUMP_STEP_JUMP;
    TRANSITION_MATRIX[JUMP_STEP_JUMP] = JUMP_STEP_RECOVERY;
    TRANSITION_MATRIX[JUMP_STEP_RECOVERY] = NORMAL_STEP;
    /*-------------------- 初始化底盘电机 --------------------*/
    MotorInit(&CHASSIS.joint_motor[0], 1, JOINT_CAN, DM_8009, J0_DIRECTION, 1, DM_MODE_MIT);
    MotorInit(&CHASSIS.joint_motor[1], 2, JOINT_CAN, DM_8009, J1_DIRECTION, 1, DM_MODE_MIT);
    MotorInit(&CHASSIS.joint_motor[2], 3, JOINT_CAN, DM_8009, J2_DIRECTION, 1, DM_MODE_MIT);
    MotorInit(&CHASSIS.joint_motor[3], 4, JOINT_CAN, DM_8009, J3_DIRECTION, 1, DM_MODE_MIT);

    MotorInit(&CHASSIS.wheel_motor[0], 1, WHEEL_CAN, MF_9025, W0_DIRECTION, 1, 0);
    MotorInit(&CHASSIS.wheel_motor[1], 2, WHEEL_CAN, MF_9025, W1_DIRECTION, 1, 0);

    /*-------------------- 值归零 --------------------*/
    memset(&CHASSIS.fdb, 0, sizeof(CHASSIS.fdb));
    memset(&CHASSIS.ref, 0, sizeof(CHASSIS.ref));

    CHASSIS.fdb.leg[0].is_take_off = false;
    CHASSIS.fdb.leg[1].is_take_off = false;

    /*-------------------- 初始化底盘PID --------------------*/
    // yaw轴跟踪pid
    float yaw_angle_pid[3] = {KP_CHASSIS_YAW_ANGLE, KI_CHASSIS_YAW_ANGLE, KD_CHASSIS_YAW_ANGLE};
    float yaw_velocity_pid[3] = {
        KP_CHASSIS_YAW_VELOCITY, KI_CHASSIS_YAW_VELOCITY, KD_CHASSIS_YAW_VELOCITY};
    PID_init(
        &CHASSIS.pid.yaw_angle, PID_POSITION, yaw_angle_pid, MAX_OUT_CHASSIS_YAW_ANGLE,
        MAX_IOUT_CHASSIS_YAW_ANGLE);
    PID_init(
        &CHASSIS.pid.yaw_velocity, PID_POSITION, yaw_velocity_pid, MAX_OUT_CHASSIS_YAW_VELOCITY,
        MAX_IOUT_CHASSIS_YAW_VELOCITY);

    // 速度增量pid
    float vel_add_pid[3] = {KP_CHASSIS_VEL_ADD, KI_CHASSIS_VEL_ADD, KD_CHASSIS_VEL_ADD};
    PID_init(
        &CHASSIS.pid.vel_add, PID_POSITION, vel_add_pid, MAX_OUT_CHASSIS_VEL_ADD,
        MAX_IOUT_CHASSIS_VEL_ADD);

    /*========== Start of locomotion control pid ==========*/

    float roll_angle_pid[3] = {KP_CHASSIS_ROLL_ANGLE, KI_CHASSIS_ROLL_ANGLE, KD_CHASSIS_ROLL_ANGLE};
    // float roll_velocity_pid[3] = {
    //     KP_CHASSIS_ROLL_VELOCITY, KI_CHASSIS_ROLL_VELOCITY, KD_CHASSIS_ROLL_VELOCITY};

    float leg_length_length_pid[3] = {
        KP_CHASSIS_LEG_LENGTH_LENGTH, KI_CHASSIS_LEG_LENGTH_LENGTH, KD_CHASSIS_LEG_LENGTH_LENGTH};

    PID_init(
        &CHASSIS.pid.roll_angle, PID_POSITION, roll_angle_pid, MAX_OUT_CHASSIS_ROLL_ANGLE,
        MAX_IOUT_CHASSIS_ROLL_ANGLE);
    CHASSIS.pid.roll_angle.N = N_CHASSIS_ROLL_ANGLE;

    // PID_init(
    //     &CHASSIS.pid.roll_velocity, PID_POSITION, roll_velocity_pid, MAX_OUT_CHASSIS_ROLL_VELOCITY,
    //     MAX_IOUT_CHASSIS_ROLL_VELOCITY);

    PID_init(
        &CHASSIS.pid.leg_length_length[0], PID_POSITION, leg_length_length_pid,
        MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH, MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH);
    CHASSIS.pid.leg_length_length[0].N = N_LEG_LENGTH_LENGTH;

    PID_init(
        &CHASSIS.pid.leg_length_length[1], PID_POSITION, leg_length_length_pid,
        MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH, MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH);
    CHASSIS.pid.leg_length_length[1].N = N_LEG_LENGTH_LENGTH;

    /*========== End of locomotion control pid ==========*/

    float stand_up_pid[3] = {KP_CHASSIS_STAND_UP, KI_CHASSIS_STAND_UP, KD_CHASSIS_STAND_UP};
    PID_init(
        &CHASSIS.pid.stand_up, PID_POSITION, stand_up_pid, MAX_OUT_CHASSIS_STAND_UP,
        MAX_IOUT_CHASSIS_STAND_UP);

    float wheel_stop_pid[3] = {KP_CHASSIS_WHEEL_STOP, KI_CHASSIS_WHEEL_STOP, KD_CHASSIS_WHEEL_STOP};
    PID_init(
        &CHASSIS.pid.wheel_stop[0], PID_POSITION, wheel_stop_pid, MAX_OUT_CHASSIS_WHEEL_STOP,
        MAX_IOUT_CHASSIS_WHEEL_STOP);
    PID_init(
        &CHASSIS.pid.wheel_stop[1], PID_POSITION, wheel_stop_pid, MAX_OUT_CHASSIS_WHEEL_STOP,
        MAX_IOUT_CHASSIS_WHEEL_STOP);
    
    float chassis_follow_gimbal_pid[3] = {
        KP_CHASSIS_FOLLOW_GIMBAL, KI_CHASSIS_FOLLOW_GIMBAL, KD_CHASSIS_FOLLOW_GIMBAL};
    PID_init(
        &CHASSIS.pid.chassis_follow_gimbal, PID_POSITION, chassis_follow_gimbal_pid,
        MAX_OUT_CHASSIS_FOLLOW_GIMBAL, MAX_IOUT_CHASSIS_FOLLOW_GIMBAL);

    // 初始化低通滤波器
    LowPassFilterInit(&CHASSIS.lpf.leg_l0_accel_filter[0], LEG_DDL0_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.leg_l0_accel_filter[1], LEG_DDL0_LPF_ALPHA);

    LowPassFilterInit(&CHASSIS.lpf.leg_phi0_accel_filter[0], LEG_DDPHI0_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.leg_phi0_accel_filter[1], LEG_DDPHI0_LPF_ALPHA);

    LowPassFilterInit(&CHASSIS.lpf.leg_theta_accel_filter[0], LEG_DDTHETA_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.leg_theta_accel_filter[1], LEG_DDTHETA_LPF_ALPHA);

    LowPassFilterInit(&CHASSIS.lpf.support_force_filter[0], LEG_SUPPORT_FORCE_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.support_force_filter[1], LEG_SUPPORT_FORCE_LPF_ALPHA);

    LowPassFilterInit(&CHASSIS.lpf.roll, CHASSIS_ROLL_ALPHA);

    // 初始化机体速度观测器
    // 使用kf同时估计速度和加速度
    Kalman_Filter_Init(&OBSERVER.body.v_kf, 2, 0, 2);
    float F[4] = {1, 0.005, 0, 1};
    float Q[4] = {VEL_PROCESS_NOISE, 0, 0, ACC_PROCESS_NOISE};
    float R[4] = {VEL_MEASURE_NOISE, 0, 0, ACC_MEASURE_NOISE};
    float P[4] = {100000, 0, 0, 100000};
    float H[4] = {1, 0, 0, 1};
    memcpy(OBSERVER.body.v_kf.F_data, F, sizeof(F));
    memcpy(OBSERVER.body.v_kf.P_data, P, sizeof(P));
    memcpy(OBSERVER.body.v_kf.Q_data, Q, sizeof(Q));
    memcpy(OBSERVER.body.v_kf.R_data, R, sizeof(R));
    memcpy(OBSERVER.body.v_kf.H_data, H, sizeof(H));
}

/******************************************************************/
/* Handle exception                                               */
/*----------------------------------------------------------------*/
/* main function:      ChassisHandleException                     */
/* auxiliary function:                                            */
/******************************************************************/

/**
 * @brief          异常处理
 * @param[in]      none
 * @retval         none
 */
void ChassisHandleException(void)
{
    if (GetRcOffline()) {
        CHASSIS.error_code |= DBUS_ERROR_OFFSET;
    } else {
        CHASSIS.error_code &= ~DBUS_ERROR_OFFSET;
    }

    if (CHASSIS.imu == NULL) {
        CHASSIS.error_code |= IMU_ERROR_OFFSET;
    } else {
        CHASSIS.error_code &= ~IMU_ERROR_OFFSET;
    }

    for (uint8_t i = 0; i < 4; i++) {
        if (fabs(CHASSIS.joint_motor[i].fdb.tor) > MAX_TORQUE_PROTECT) {
            CHASSIS.error_code |= JOINT_ERROR_OFFSET;
            break;
        }
    }

    if ((CHASSIS.mode == CHASSIS_OFF || CHASSIS.mode == CHASSIS_SAFE) &&
        fabs(CHASSIS.pid.stand_up.max_out) != 0.0f) {
        PID_clear(&CHASSIS.pid.stand_up);
    }
}

/******************************************************************/
/* Set mode                                                       */
/*----------------------------------------------------------------*/
/* main function:      ChassisSetMode                             */
/* auxiliary function: None                                       */
/******************************************************************/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void ChassisSetMode(void)
{
    if (CHASSIS.error_code & DBUS_ERROR_OFFSET) {  // 遥控器出错时的状态处理
        CHASSIS.mode = CHASSIS_SAFE;
        return;
    }

    if (CHASSIS.error_code & IMU_ERROR_OFFSET) {  // IMU出错时的状态处理
        CHASSIS.mode = CHASSIS_SAFE;
        return;
    }

    if (CHASSIS.error_code & JOINT_ERROR_OFFSET) {  // 关节电机出错时的状态处理
        CHASSIS.mode = CHASSIS_SAFE;
        return;
    }

    if (CHASSIS.mode == CHASSIS_CALIBRATE && (!CALIBRATE.calibrated)) {  //校准完成后才退出校准
        return;
    }

    if (CALIBRATE.toggle) {  // 切入底盘校准
        CALIBRATE.toggle = false;
        CHASSIS.mode = CHASSIS_CALIBRATE;
        CALIBRATE.calibrated = false;

        uint32_t now = HAL_GetTick();
        for (uint8_t i = 0; i < 4; i++) {
            CALIBRATE.reached[i] = false;
            CALIBRATE.stpo_time[i] = now;
        }

        return;
    }

#if TAKE_OFF_DETECT
    // 离地状态切换
    for (uint8_t i = 0; i < 2; i++) {
        if (CHASSIS.fdb.leg[i].is_take_off &&
            CHASSIS.fdb.leg[i].touch_time > TOUCH_TOGGLE_THRESHOLD) {
            CHASSIS.fdb.leg[i].is_take_off = false;
        } else if (
            !CHASSIS.fdb.leg[i].is_take_off &&
            CHASSIS.fdb.leg[i].take_off_time > TOUCH_TOGGLE_THRESHOLD) {
            CHASSIS.fdb.leg[i].is_take_off = true;
        }
    }
#endif

    if (switch_is_up(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        // CHASSIS.mode = CHASSIS_FREE;
        CHASSIS.mode = CHASSIS_SAFE;
    } else if (switch_is_mid(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        CHASSIS.mode = CHASSIS_FOLLOW_GIMBAL_YAW;;
    } else if (switch_is_down(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        // 在安全模式时，遥控器摇杆打成左下，右上进入脱困模式
        if (CHASSIS.rc->rc.ch[0] > RC_OFF_HOOK_VALUE_HOLE &&
            CHASSIS.rc->rc.ch[1] > RC_OFF_HOOK_VALUE_HOLE &&
            CHASSIS.rc->rc.ch[2] < -RC_OFF_HOOK_VALUE_HOLE &&
            CHASSIS.rc->rc.ch[3] < -RC_OFF_HOOK_VALUE_HOLE) {
            CHASSIS.mode = CHASSIS_OFF_HOOK;
        } else {
            CHASSIS.mode = CHASSIS_SAFE;
        }
    }
}

/******************************************************************/
/* Observe                                                        */
/*----------------------------------------------------------------*/
/* main function:      ChassisObserver                            */
/* auxiliary function: UpdateBodyStatus                           */
/*                     UpdateLegStatus                            */
/*                     UpdateMotorStatus                          */
/*                     UpdateCalibrateStatus                      */
/*                     BodyMotionObserve                          */
/******************************************************************/

#define ZERO_POS_THRESHOLD 0.001f

static void UpdateBodyStatus(void);
static void UpdateLegStatus(void);
static void UpdateMotorStatus(void);
static void UpdateCalibrateStatus(void);
static void UpdateStepStatus(void);

static void BodyMotionObserve(void);

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void ChassisObserver(void)
{
    CHASSIS.duration = xTaskGetTickCount() - CHASSIS.last_time;
    CHASSIS.last_time = xTaskGetTickCount();

    UpdateMotorStatus();
    UpdateLegStatus();
    UpdateBodyStatus();
    UpdateCalibrateStatus();
    UpdateStepStatus();

    BodyMotionObserve();

    // 传输数据
    float F0_Tp[2];
    GetLegForce(
        CHASSIS.fdb.leg[0].J, CHASSIS.fdb.leg[0].joint.T1, CHASSIS.fdb.leg[0].joint.T2, F0_Tp);
}

/**
 * @brief  更新底盘电机数据
 * @param  none
 */
static void UpdateMotorStatus(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        GetMotorMeasure(&CHASSIS.joint_motor[i]);
    }

    for (uint8_t i = 0; i < 2; i++) {
        GetMotorMeasure(&CHASSIS.wheel_motor[i]);
    }
}

static void UpdateBodyStatus(void)
{
    // 更新陀螺仪反馈数据
    CHASSIS.fdb.body.roll = GetImuAngle(AX_ROLL);
    CHASSIS.fdb.body.roll_dot = GetImuVelocity(AX_ROLL);

    CHASSIS.fdb.body.pitch = GetImuAngle(AX_PITCH);
    CHASSIS.fdb.body.pitch_dot = GetImuVelocity(AX_PITCH);

    CHASSIS.fdb.body.yaw = GetImuAngle(AX_YAW);
    CHASSIS.fdb.body.yaw_dot = GetImuVelocity(AX_YAW);

    LowPassFilterCalc(&CHASSIS.lpf.roll, CHASSIS.fdb.body.roll);

    // 更新加速度反馈数据，记录下来方便使用
    float ax = GetImuAccel(AX_X);
    float ay = GetImuAccel(AX_Y);
    float az = GetImuAccel(AX_Z);
    // 计算几个常用的三角函数值，减少重复计算
    float cos_roll = cosf(CHASSIS.fdb.body.roll);
    float sin_roll = sinf(CHASSIS.fdb.body.roll);
    float cos_pitch = cosf(CHASSIS.fdb.body.pitch);
    float sin_pitch = sinf(CHASSIS.fdb.body.pitch);
    float cos_yaw = cosf(CHASSIS.fdb.body.yaw);
    float sin_yaw = sinf(CHASSIS.fdb.body.yaw);

    // 计算重力加速度在各个轴上的分量相反值
    CHASSIS.fdb.body.gx = GRAVITY * sin_pitch;
    CHASSIS.fdb.body.gy = -GRAVITY * sin_roll * cos_pitch;
    CHASSIS.fdb.body.gz = -GRAVITY * cos_roll * cos_pitch;

    // 消除重力加速度的影响，获取机体坐标系下的加速度
    CHASSIS.fdb.body.x_accel = ax + CHASSIS.fdb.body.gx;
    CHASSIS.fdb.body.y_accel = ay + CHASSIS.fdb.body.gy;
    CHASSIS.fdb.body.z_accel = az + CHASSIS.fdb.body.gz;

    // 计算从机体坐标系到大地坐标系的旋转矩阵
    // clang-format off
    float R[3][3] = {
        {cos_pitch * cos_yaw, sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw, cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw},
        {cos_pitch * sin_yaw, sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw, cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw},
        {-sin_pitch         , sin_roll * cos_pitch                               , cos_roll * cos_pitch                               }
    };
    // clang-format on

    // 更新大地坐标系下的加速度
    CHASSIS.fdb.world.x_accel = R[0][0] * ax + R[0][1] * ay + R[0][2] * az;
    CHASSIS.fdb.world.y_accel = R[1][0] * ax + R[1][1] * ay + R[1][2] * az;
    CHASSIS.fdb.world.z_accel = R[2][0] * ax + R[2][1] * ay + R[2][2] * az - GRAVITY;

    // 更新...
    CHASSIS.fdb.body.phi = -CHASSIS.fdb.body.pitch;
    CHASSIS.fdb.body.phi_dot = -CHASSIS.fdb.body.pitch_dot;

    CHASSIS.fdb.body.x_acc = CHASSIS.fdb.body.x_accel;
}

/**
 * @brief  更新腿部状态
 * @param  none
 */
static void UpdateLegStatus(void)
{
    uint8_t i = 0;
    // =====更新关节姿态=====
    CHASSIS.fdb.leg[0].joint.Phi1 =
        theta_transform(CHASSIS.joint_motor[0].fdb.pos, J0_ANGLE_OFFSET, J0_DIRECTION, 1);
    CHASSIS.fdb.leg[0].joint.Phi4 =
        theta_transform(CHASSIS.joint_motor[1].fdb.pos, J1_ANGLE_OFFSET, J1_DIRECTION, 1);
    CHASSIS.fdb.leg[1].joint.Phi1 =
        theta_transform(CHASSIS.joint_motor[2].fdb.pos, J2_ANGLE_OFFSET, J2_DIRECTION, 1);
    CHASSIS.fdb.leg[1].joint.Phi4 =
        theta_transform(CHASSIS.joint_motor[3].fdb.pos, J3_ANGLE_OFFSET, J3_DIRECTION, 1);

    CHASSIS.fdb.leg[0].joint.dPhi1 = CHASSIS.joint_motor[0].fdb.vel * (J0_DIRECTION);
    CHASSIS.fdb.leg[0].joint.dPhi4 = CHASSIS.joint_motor[1].fdb.vel * (J1_DIRECTION);
    CHASSIS.fdb.leg[1].joint.dPhi1 = CHASSIS.joint_motor[2].fdb.vel * (J2_DIRECTION);
    CHASSIS.fdb.leg[1].joint.dPhi4 = CHASSIS.joint_motor[3].fdb.vel * (J3_DIRECTION);

    CHASSIS.fdb.leg[0].joint.T1 = CHASSIS.joint_motor[0].fdb.tor * (J0_DIRECTION);
    CHASSIS.fdb.leg[0].joint.T2 = CHASSIS.joint_motor[1].fdb.tor * (J1_DIRECTION);
    CHASSIS.fdb.leg[1].joint.T1 = CHASSIS.joint_motor[2].fdb.tor * (J2_DIRECTION);
    CHASSIS.fdb.leg[1].joint.T2 = CHASSIS.joint_motor[3].fdb.tor * (J3_DIRECTION);

    // =====更新驱动轮姿态=====
    CHASSIS.fdb.leg[0].wheel.Velocity = CHASSIS.wheel_motor[0].fdb.vel * (W0_DIRECTION);
    CHASSIS.fdb.leg[1].wheel.Velocity = CHASSIS.wheel_motor[1].fdb.vel * (W1_DIRECTION);

    // =====更新摆杆姿态=====
    float L0_Phi0[2];
    float dL0_dPhi0[2];
    for (i = 0; i < 2; i++) {
        float last_dL0 = CHASSIS.fdb.leg[i].rod.dL0;
        float last_dPhi0 = CHASSIS.fdb.leg[i].rod.dPhi0;
        float last_dTheta = CHASSIS.fdb.leg[i].rod.dTheta;

        // 更新位置信息
        GetL0AndPhi0(CHASSIS.fdb.leg[i].joint.Phi1, CHASSIS.fdb.leg[i].joint.Phi4, L0_Phi0);
        CHASSIS.fdb.leg[i].rod.L0 = L0_Phi0[0];
        CHASSIS.fdb.leg[i].rod.Phi0 = L0_Phi0[1];
        CHASSIS.fdb.leg[i].rod.Theta = M_PI_2 - CHASSIS.fdb.leg[i].rod.Phi0 - CHASSIS.fdb.body.phi;

        // 计算雅可比矩阵
        CalcJacobian(
            CHASSIS.fdb.leg[i].joint.Phi1, CHASSIS.fdb.leg[i].joint.Phi4, CHASSIS.fdb.leg[i].J);

        // 更新速度信息
        GetdL0AnddPhi0(
            CHASSIS.fdb.leg[i].J, CHASSIS.fdb.leg[i].joint.dPhi1, CHASSIS.fdb.leg[i].joint.dPhi4,
            dL0_dPhi0);
        CHASSIS.fdb.leg[i].rod.dL0 = dL0_dPhi0[0];
        CHASSIS.fdb.leg[i].rod.dPhi0 = dL0_dPhi0[1];
        CHASSIS.fdb.leg[i].rod.dTheta = -CHASSIS.fdb.leg[i].rod.dPhi0 - CHASSIS.fdb.body.phi_dot;

        // 更新加速度信息
        float accel = (CHASSIS.fdb.leg[i].rod.dL0 - last_dL0) / (CHASSIS.duration * MS_TO_S);
        CHASSIS.fdb.leg[i].rod.ddL0 = accel;

        accel = (CHASSIS.fdb.leg[i].rod.dPhi0 - last_dPhi0) / (CHASSIS.duration * MS_TO_S);
        CHASSIS.fdb.leg[i].rod.ddPhi0 = accel;

        accel = (CHASSIS.fdb.leg[i].rod.dTheta - last_dTheta) / (CHASSIS.duration * MS_TO_S);
        CHASSIS.fdb.leg[i].rod.ddTheta = accel;

        // 差分计算腿长变化率和腿角速度
        float ddot_z_M = CHASSIS.fdb.world.z_accel;
        float l0 = CHASSIS.fdb.leg[i].rod.L0;
        float v_l0 = CHASSIS.fdb.leg[i].rod.dL0;
        float theta = CHASSIS.fdb.leg[i].rod.Theta;
        float w_theta = CHASSIS.fdb.leg[i].rod.dTheta;

        float dot_v_l0 = CHASSIS.fdb.leg[i].rod.ddL0;
        float dot_w_theta = CHASSIS.fdb.leg[i].rod.ddTheta;
        // clang-format off
        float ddot_z_w = ddot_z_M 
                    - dot_v_l0 * cosf(theta) 
                    + 2.0f * v_l0 * w_theta * sinf(theta) 
                    + l0 * dot_w_theta * sinf(theta) 
                    + l0 * w_theta * w_theta * cosf(theta);
        // clang-format on

        // 计算支撑力
        float F[2];
        GetLegForce(
            CHASSIS.fdb.leg[i].J, CHASSIS.fdb.leg[i].joint.T1, CHASSIS.fdb.leg[i].joint.T2, F);
        float F0 = F[0];
        float Tp = F[1];

        float P = F0 * cosf(theta) + Tp * sinf(theta) / l0;
        CHASSIS.fdb.leg[i].Fn = P + WHEEL_MASS * (9.8f + ddot_z_w);
        if (CHASSIS.fdb.leg[i].Fn < TAKE_OFF_FN_THRESHOLD) {
            CHASSIS.fdb.leg[i].touch_time = 0;
            CHASSIS.fdb.leg[i].take_off_time += CHASSIS.duration;
        } else {
            CHASSIS.fdb.leg[i].touch_time += CHASSIS.duration;
            CHASSIS.fdb.leg[i].take_off_time = 0;
        }
    }
}

static void UpdateCalibrateStatus(void)
{
    if ((CHASSIS.mode == CHASSIS_CALIBRATE) &&
        fabs(CHASSIS.joint_motor[0].fdb.pos) < ZERO_POS_THRESHOLD &&
        fabs(CHASSIS.joint_motor[1].fdb.pos) < ZERO_POS_THRESHOLD &&
        fabs(CHASSIS.joint_motor[2].fdb.pos) < ZERO_POS_THRESHOLD &&
        fabs(CHASSIS.joint_motor[3].fdb.pos) < ZERO_POS_THRESHOLD) {
        CALIBRATE.calibrated = true;
    }

    // 校准模式的相关反馈数据
    uint32_t now = HAL_GetTick();
    if (CHASSIS.mode == CHASSIS_CALIBRATE) {
        for (uint8_t i = 0; i < 4; i++) {
            CALIBRATE.velocity[i] = CHASSIS.joint_motor[i].fdb.vel;
            if (CALIBRATE.velocity[i] > CALIBRATE_STOP_VELOCITY) {  // 速度大于阈值时重置计时
                CALIBRATE.reached[i] = false;
                CALIBRATE.stpo_time[i] = now;
            } else {
                if (now - CALIBRATE.stpo_time[i] > CALIBRATE_STOP_TIME) {
                    CALIBRATE.reached[i] = true;
                }
            }
        }
    }
}

#define StateTransfer()    \
    CHASSIS.step_time = 0; \
    CHASSIS.step = TRANSITION_MATRIX[CHASSIS.step];

static void UpdateStepStatus(void)
{
    CHASSIS.step_time += CHASSIS.duration;

    if (CHASSIS.mode == CHASSIS_CUSTOM) {
        if (0 && (GetDt7RcCh(DT7_CH_RH) < -0.9f)) {  // 遥控器左侧水平摇杆打到左边切换至跳跃状态
            CHASSIS.step_time = 0;
            CHASSIS.step = JUMP_STEP_SQUST;
        } else if (CHASSIS.step == JUMP_STEP_SQUST) {  // 跳跃——蹲下蓄力状态
            if (CHASSIS.fdb.leg[0].rod.L0 < MIN_LEG_LENGTH + 0.02f &&
                CHASSIS.fdb.leg[1].rod.L0 < MIN_LEG_LENGTH + 0.02f) {
                StateTransfer();
            }
        } else if (CHASSIS.step == JUMP_STEP_JUMP) {  // 跳跃——起跳状态
            if (CHASSIS.fdb.leg[0].rod.L0 > MAX_LEG_LENGTH - 0.03f &&
                CHASSIS.fdb.leg[1].rod.L0 > MAX_LEG_LENGTH - 0.03f) {
                StateTransfer();
            }
        } else if (CHASSIS.step == JUMP_STEP_RECOVERY) {  // 跳跃——收腿状态
            if (CHASSIS.step_time > 1000) {               // 1000ms后切换状态
                StateTransfer();
            }
        } else if (CHASSIS.step != NORMAL_STEP && CHASSIS.step_time > MAX_STEP_TIME) {
            // 状态持续时间超过 MAX_STEP_TIME ，自动切换到NORMAL状态
            CHASSIS.step_time = 0;
            CHASSIS.step = NORMAL_STEP;
        }
    } else {
        CHASSIS.step_time = 0;
        CHASSIS.step = NORMAL_STEP;
    }
}
#undef StateTransfer

/**
 * @brief  机体运动状态观测器
 * @param  none
 */
static void BodyMotionObserve(void)
{
    // clang-format off
    float speed = WHEEL_RADIUS * (CHASSIS.fdb.leg[0].wheel.Velocity + CHASSIS.fdb.leg[1].wheel.Velocity) / 2;
    // clang-format on

    // 使用kf同时估计加速度和速度,滤波更新
    OBSERVER.body.v_kf.MeasuredVector[0] = speed;                   // 输入轮速
    OBSERVER.body.v_kf.MeasuredVector[1] = CHASSIS.fdb.body.x_acc;  // 输入加速度
    OBSERVER.body.v_kf.F_data[1] = CHASSIS.duration * MS_TO_S;      // 更新采样时间

    Kalman_Filter_Update(&OBSERVER.body.v_kf);
    CHASSIS.fdb.body.x_dot_obv = OBSERVER.body.v_kf.xhat_data[0];
    CHASSIS.fdb.body.x_acc_obv = OBSERVER.body.v_kf.xhat_data[1];

    // 更新行驶距离
    if (fabs(CHASSIS.ref.speed_vector.vx) < WHEEL_DEADZONE &&
        fabs(CHASSIS.fdb.body.x_dot_obv) < 0.8f) {
        // 当目标速度为0，且速度小于阈值时，计算反馈距离
        CHASSIS.fdb.body.x += CHASSIS.fdb.body.x_dot_obv * CHASSIS.duration * MS_TO_S;
    } else {
        //CHASSIS.fdb.body.x = 0;
    }

    // 更新2条腿的状态向量
    uint8_t i = 0;
    for (i = 0; i < 2; i++) {
        // clang-format off
        CHASSIS.fdb.leg_state[i].theta     =  M_PI_2 - CHASSIS.fdb.leg[i].rod.Phi0 - CHASSIS.fdb.body.phi;
        CHASSIS.fdb.leg_state[i].theta_dot = -CHASSIS.fdb.leg[i].rod.dPhi0 - CHASSIS.fdb.body.phi_dot;
        CHASSIS.fdb.leg_state[i].x         =  CHASSIS.fdb.body.x;
        CHASSIS.fdb.leg_state[i].x_dot     =  CHASSIS.fdb.body.x_dot_obv;
        CHASSIS.fdb.leg_state[i].phi       =  CHASSIS.fdb.body.phi;
        CHASSIS.fdb.leg_state[i].phi_dot   =  CHASSIS.fdb.body.phi_dot;
        // clang-format on
    }
}

/******************************************************************/
/* Reference                                                      */
/*----------------------------------------------------------------*/
/* main function:      ChassisReference                           */
/* auxiliary function: None                                       */
/******************************************************************/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void ChassisReference(void)
{
    int16_t rc_x = 0, rc_wz = 0;
    int16_t rc_length = 0, rc_angle = 0;
    int16_t rc_roll = 0;
    rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_X_CHANNEL], rc_x, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_WZ_CHANNEL], rc_wz, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_LENGTH_CHANNEL], rc_length, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_ANGLE_CHANNEL], rc_angle, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_ROLL_CHANNEL], rc_roll, CHASSIS_RC_DEADLINE);

    // 计算速度向量
    ChassisSpeedVector_t v_set = {0.0f, 0.0f, 0.0f};
    v_set.vx = rc_x * RC_TO_ONE * MAX_SPEED_VECTOR_VX;
    v_set.vy = 0;
    v_set.wz = -rc_wz * RC_TO_ONE * MAX_SPEED_VECTOR_WZ;
    switch (CHASSIS.mode) {
        case CHASSIS_FREE: {  // 底盘自由模式下，控制量为底盘坐标系下的速度
            CHASSIS.ref.speed_vector.vx = v_set.vx;
            CHASSIS.ref.speed_vector.vy = 0;
            CHASSIS.ref.speed_vector.wz = v_set.wz;
            break;
        }
        case CHASSIS_CUSTOM: {
            CHASSIS.ref.speed_vector.vx = v_set.vx;
            CHASSIS.ref.speed_vector.vy = 0;
            CHASSIS.ref.speed_vector.wz = v_set.wz;
            break;
        }
        case CHASSIS_FOLLOW_GIMBAL_YAW: {  // 云台跟随模式下，控制量为云台坐标系下的速度，需要进行坐标转换
            float delta_yaw = GetGimbalDeltaYawMid();
            CHASSIS.ref.speed_vector.vx = v_set.vx * cosf(delta_yaw);
            CHASSIS.ref.speed_vector.vy = 0;
            if (GetGimbalInitJudgeReturn()) {
                CHASSIS.ref.speed_vector.wz = 0;
            } else {
                CHASSIS.ref.speed_vector.wz =
                    PID_calc(&CHASSIS.pid.chassis_follow_gimbal, -delta_yaw, 0);
            }
        } break;
        case CHASSIS_AUTO: {  // 底盘自动模式，控制量为云台坐标系下的速度，需要进行坐标转换
            CHASSIS.ref.speed_vector.vx = v_set.vx;
            CHASSIS.ref.speed_vector.vy = 0;
            CHASSIS.ref.speed_vector.wz = v_set.wz;
            break;
        }
        case CHASSIS_STAND_UP: {
            CHASSIS.ref.speed_vector.vx = 0;
            CHASSIS.ref.speed_vector.vy = 0;
            CHASSIS.ref.speed_vector.wz = 0;
        } break;
        default:
            CHASSIS.ref.speed_vector.vx = 0;
            CHASSIS.ref.speed_vector.vy = 0;
            CHASSIS.ref.speed_vector.wz = 0;
            break;
    }

    // 计算期望状态
    // clang-format off
    for (uint8_t i = 0; i < 2; i++) {
        CHASSIS.ref.leg_state[i].theta     =  0;
        CHASSIS.ref.leg_state[i].theta_dot =  0;
        CHASSIS.ref.leg_state[i].x         =  0;//CHASSIS.ref.speed_vector.vx * CHASSIS_CONTROL_TIME_S * X_ADD_RATIO;
        CHASSIS.ref.leg_state[i].x_dot     =  CHASSIS.ref.speed_vector.vx;
        CHASSIS.ref.leg_state[i].phi       =  0;
        CHASSIS.ref.leg_state[i].phi_dot   =  0;
    }
    // clang-format on

    // 腿部控制
    static float angle = M_PI_2;
    static float length = 0.12f;
    switch (CHASSIS.mode) {
        case CHASSIS_STAND_UP: {
            length = 0.12f;
            angle = M_PI_2;
        } break;
        case CHASSIS_DEBUG: {
            length = 0.12f;
            CHASSIS.ref.leg_state[0].theta = rc_angle * RC_TO_ONE * 0.3f;
            CHASSIS.ref.leg_state[1].theta = rc_angle * RC_TO_ONE * 0.3f;
        }
        case CHASSIS_FOLLOW_GIMBAL_YAW:
        case CHASSIS_CUSTOM:
        case CHASSIS_POS_DEBUG: {
            angle = M_PI_2 + rc_angle * RC_TO_ONE * 0.3f;
            length = 0.24f + rc_length * 0.00000001f;

            if (CHASSIS.step == JUMP_STEP_SQUST) {
                length = MIN_LEG_LENGTH;
            } else if (CHASSIS.step == JUMP_STEP_JUMP) {
                length = MAX_LEG_LENGTH;
            } else if (CHASSIS.step == JUMP_STEP_RECOVERY) {
                length = MIN_LEG_LENGTH + 0.05f;
            }
        } break;
        case CHASSIS_FREE: {
        } break;
        default: {
            angle = M_PI_2;
            length = 0.12f;
        }
    }
    length = fp32_constrain(length, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
    angle = fp32_constrain(angle, MIN_LEG_ANGLE, MAX_LEG_ANGLE);

    CHASSIS.ref.rod_L0[0] = length;
    CHASSIS.ref.rod_L0[1] = length;
    CHASSIS.ref.rod_Angle[0] = angle;
    CHASSIS.ref.rod_Angle[1] = angle;

    CHASSIS.ref.body.roll = fp32_constrain(-rc_roll * RC_TO_ONE * MAX_ROLL, MIN_ROLL, MAX_ROLL);
}

/******************************************************************/
/* Console                                                        */
/*----------------------------------------------------------------*/
/* main function:      ChassisConsole                             */
/* auxiliary function: LocomotionController                       */
/*                     LegTorqueController                        */
/*                     LegFeedForward                             */
/*                     CalcLQR                                    */
/*                     ConsoleZeroForce                           */
/*                     ConsoleCalibrate                           */
/*                     ConsoleOffHook                             */
/*                     ConsoleNormal                              */
/*                     ConsoleDebug                               */
/*                     ConsolePosDebug                            */
/*                     ConsoleStandUp                             */
/******************************************************************/

static void LocomotionController(void);
// static void LegPositionController(void);
static void LegTorqueController(void);
static float LegFeedForward(float theta);
static void CalcLQR(float k[2][6], float x[6], float t[2]);

static void ConsoleZeroForce(void);
static void ConsoleCalibrate(void);
static void ConsoleOffHook(void);
static void ConsoleNormal(void);
static void ConsoleDebug(void);
static void ConsolePosDebug(void);
static void ConsoleStandUp(void);

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisConsole(void)
{
    switch (CHASSIS.mode) {
        case CHASSIS_CALIBRATE: {
            ConsoleCalibrate();
        } break;
        case CHASSIS_OFF_HOOK: {
            ConsoleOffHook();
        } break;
        case CHASSIS_FOLLOW_GIMBAL_YAW:
        case CHASSIS_CUSTOM:
        case CHASSIS_FREE: {
            ConsoleNormal();
        } break;
        case CHASSIS_DEBUG: {
            ConsoleDebug();
        } break;
        case CHASSIS_POS_DEBUG: {
            ConsolePosDebug();
        } break;
        case CHASSIS_STAND_UP: {
            ConsoleStandUp();
        } break;
        case CHASSIS_OFF:
        case CHASSIS_SAFE:
        default: {
            ConsoleZeroForce();
        }
    }

#if CLOSE_LEG_LEFT
    memset(&CHASSIS.joint_motor[0].set, 0, sizeof(CHASSIS.joint_motor[0].set));
    memset(&CHASSIS.joint_motor[1].set, 0, sizeof(CHASSIS.joint_motor[1].set));
    memset(&CHASSIS.wheel_motor[0].set, 0, sizeof(CHASSIS.wheel_motor[0].set));
#endif

#if CLOSE_LEG_RIGHT
    memset(&CHASSIS.joint_motor[2].set, 0, sizeof(CHASSIS.joint_motor[2].set));
    memset(&CHASSIS.joint_motor[3].set, 0, sizeof(CHASSIS.joint_motor[3].set));
    memset(&CHASSIS.wheel_motor[1].set, 0, sizeof(CHASSIS.wheel_motor[1].set));
#endif

#if LIFTED_UP
    CHASSIS.wheel_motor[0].set.tor = 0;
    CHASSIS.wheel_motor[1].set.tor = 0;
    CHASSIS.wheel_motor[0].set.value = 0;
    CHASSIS.wheel_motor[1].set.value = 0;
#endif
}

/**
 * @brief      运动控制器
 */
static void LocomotionController(void)
{
    // 计算LQR增益=============================================
    float k[2][6];
    float x[6];
    float T_Tp[2];
    bool is_take_off = CHASSIS.fdb.leg[0].is_take_off || CHASSIS.fdb.leg[1].is_take_off;
#if LIFTED_UP
    is_take_off = true;
#endif
    // if (CHASSIS.step == JUMP_STEP_RECOVERY) {
    //     is_take_off = true;
    // }

    for (uint8_t i = 0; i < 2; i++) {
        GetK(CHASSIS.fdb.leg[i].rod.L0, k, is_take_off);

        // clang-format off
        x[0] = X0_OFFSET + (CHASSIS.fdb.leg_state[i].theta     - CHASSIS.ref.leg_state[i].theta);
        x[1] = X1_OFFSET + (CHASSIS.fdb.leg_state[i].theta_dot - CHASSIS.ref.leg_state[i].theta_dot);
        x[2] = X2_OFFSET + (CHASSIS.fdb.leg_state[i].x         - CHASSIS.ref.leg_state[i].x);
        x[3] = X3_OFFSET + (CHASSIS.fdb.leg_state[i].x_dot     - CHASSIS.ref.leg_state[i].x_dot);
        x[4] = X4_OFFSET + (CHASSIS.fdb.leg_state[i].phi       - CHASSIS.ref.leg_state[i].phi);
        x[5] = X5_OFFSET + (CHASSIS.fdb.leg_state[i].phi_dot   - CHASSIS.ref.leg_state[i].phi_dot);
        // clang-format on
        CalcLQR(k, x, T_Tp);

        CHASSIS.cmd.leg[i].wheel.T = T_Tp[0];
        CHASSIS.cmd.leg[i].rod.Tp = T_Tp[1];
    }

    // ROLL角控制=============================================
    // 计算腿长差值
    float Ld0 = CHASSIS.fdb.leg[0].rod.L0 - CHASSIS.fdb.leg[1].rod.L0;
    float L_diff = CalcLegLengthDiff(Ld0, CHASSIS.fdb.body.roll, CHASSIS.ref.body.roll);

    // PID补偿稳态误差
    float delta_L0 = 0.0f;

    // 维持腿长在范围内
    CoordinateLegLength(&CHASSIS.ref.rod_L0[0], &CHASSIS.ref.rod_L0[1], L_diff, delta_L0);

    // 转向控制================================================
    if (!is_take_off) {
        PID_calc(&CHASSIS.pid.yaw_velocity, CHASSIS.fdb.body.yaw_dot, CHASSIS.ref.speed_vector.wz);
        CHASSIS.cmd.leg[0].wheel.T += CHASSIS.pid.yaw_velocity.out;
        CHASSIS.cmd.leg[1].wheel.T -= CHASSIS.pid.yaw_velocity.out;
    }
}

/**
 * @brief 腿部力矩控制
 */
static void LegTorqueController(void)
{
    // 腿长控制
    float F_ff, F_compensate;

    float roll_vel_limit_f =
        fp32_constrain(CHASSIS.fdb.body.roll_dot * ROLL_VEL_LIMIT_FACTOR, -0.2, 0.2);

    for (uint8_t i = 0; i < 2; i++) {
        if (CHASSIS.step == JUMP_STEP_JUMP) {
            // 直接给一个超大力F起飞
            CHASSIS.cmd.leg[i].rod.F = 40;
        } else {
            // 计算前馈力
            F_ff = LegFeedForward(CHASSIS.fdb.leg_state[i].theta) * FF_RATIO;
            // PID补偿
            F_compensate = PID_calc(
                &CHASSIS.pid.leg_length_length[i], CHASSIS.fdb.leg[i].rod.L0,
                CHASSIS.ref.rod_L0[i]);
            // 计算总力
            CHASSIS.cmd.leg[i].rod.F = F_ff + F_compensate;
        }
        // CHASSIS.cmd.leg[i].rod.F = F_ff + F_compensate - F_ff;
    }

    CHASSIS.cmd.leg[0].rod.F -= roll_vel_limit_f;
    CHASSIS.cmd.leg[1].rod.F += roll_vel_limit_f;

    // 转换为关节力矩
    CalcVmc(
        CHASSIS.cmd.leg[0].rod.F, CHASSIS.cmd.leg[0].rod.Tp, CHASSIS.fdb.leg[0].J,
        CHASSIS.cmd.leg[0].joint.T);
    CalcVmc(
        CHASSIS.cmd.leg[1].rod.F, CHASSIS.cmd.leg[1].rod.Tp, CHASSIS.fdb.leg[1].J,
        CHASSIS.cmd.leg[1].joint.T);
}

/**
 * @brief        前馈控制
 * @param[in]    theta 当前腿与竖直方向夹角
 * @return       前馈量
 */
static float LegFeedForward(float theta) { return BODY_MASS * GRAVITY * cosf(theta) / 2; }

/**
 * @brief         矩阵相乘，计算LQR输出
 * @param[in]     k   LQR反馈矩阵K
 * @param[in]     x   状态变量向量
 * @param[out]    T_Tp 反馈数据T和Tp
 * @return        none
 */
static void CalcLQR(float k[2][6], float x[6], float T_Tp[2])
{
    T_Tp[0] = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] + k[0][4] * x[4] +
              k[0][5] * x[5];
    T_Tp[1] = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3] + k[1][4] * x[4] +
              k[1][5] * x[5];
}

//* 各个模式下的控制

static void ConsoleZeroForce(void)
{
    CHASSIS.joint_motor[0].set.tor = 0;
    CHASSIS.joint_motor[1].set.tor = 0;
    CHASSIS.joint_motor[2].set.tor = 0;
    CHASSIS.joint_motor[3].set.tor = 0;

    CHASSIS.joint_motor[0].set.vel = 0;
    CHASSIS.joint_motor[1].set.vel = 0;
    CHASSIS.joint_motor[2].set.vel = 0;
    CHASSIS.joint_motor[3].set.vel = 0;

    CHASSIS.wheel_motor[0].set.vel = 0;
    CHASSIS.wheel_motor[1].set.vel = 0;

    CHASSIS.wheel_motor[0].set.value =
        PID_calc(&CHASSIS.pid.wheel_stop[0], CHASSIS.wheel_motor[0].fdb.vel, 0);
    CHASSIS.wheel_motor[1].set.value =
        PID_calc(&CHASSIS.pid.wheel_stop[1], CHASSIS.wheel_motor[1].fdb.vel, 0);
}

static void ConsoleCalibrate(void)
{
    CHASSIS.joint_motor[0].set.vel = -CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[1].set.vel = CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[2].set.vel = CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[3].set.vel = -CALIBRATE_VELOCITY;

    CHASSIS.wheel_motor[0].set.tor = 0;
    CHASSIS.wheel_motor[1].set.tor = 0;
}

static void ConsoleOffHook(void)
{
    CHASSIS.joint_motor[0].set.vel = -CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[1].set.vel = CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[2].set.vel = CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[3].set.vel = -CALIBRATE_VELOCITY;

    CHASSIS.wheel_motor[0].set.tor = 0;
    CHASSIS.wheel_motor[1].set.tor = 0;
}

static void ConsoleNormal(void)
{
    LocomotionController();
    LegTorqueController();

    // 给关节电机赋值
    CHASSIS.joint_motor[0].set.tor = CHASSIS.cmd.leg[0].joint.T[0] * (J0_DIRECTION);
    CHASSIS.joint_motor[1].set.tor = CHASSIS.cmd.leg[0].joint.T[1] * (J1_DIRECTION);
    CHASSIS.joint_motor[2].set.tor = CHASSIS.cmd.leg[1].joint.T[0] * (J2_DIRECTION);
    CHASSIS.joint_motor[3].set.tor = CHASSIS.cmd.leg[1].joint.T[1] * (J3_DIRECTION);

    for (uint8_t i = 0; i < 4; i++) {
        if (CHASSIS.step == JUMP_STEP_JUMP) {
            CHASSIS.joint_motor[i].set.tor = fp32_constrain(
                CHASSIS.joint_motor[i].set.tor, MIN_JOINT_TORQUE_JUMP, MAX_JOINT_TORQUE_JUMP);
        } else {
            CHASSIS.joint_motor[i].set.tor =
                fp32_constrain(CHASSIS.joint_motor[i].set.tor, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
        }
    }

    // 给驱动轮电机赋值
    // QUESTION: 排查电机发送的力矩要反向的问题，这种情况下控制正常
    //不知道为什么要反向，待后续研究
    CHASSIS.wheel_motor[0].set.tor = -(CHASSIS.cmd.leg[0].wheel.T * (W0_DIRECTION));
    CHASSIS.wheel_motor[1].set.tor = -(CHASSIS.cmd.leg[1].wheel.T * (W1_DIRECTION));
}

static void ConsoleDebug(void)
{
    LocomotionController();
    LegTorqueController();

    // 给关节电机赋值
    CHASSIS.joint_motor[0].set.tor = CHASSIS.cmd.leg[0].joint.T[0] * (J0_DIRECTION);
    CHASSIS.joint_motor[1].set.tor = CHASSIS.cmd.leg[0].joint.T[1] * (J1_DIRECTION);
    CHASSIS.joint_motor[2].set.tor = CHASSIS.cmd.leg[1].joint.T[0] * (J2_DIRECTION);
    CHASSIS.joint_motor[3].set.tor = CHASSIS.cmd.leg[1].joint.T[1] * (J3_DIRECTION);

    for (uint8_t i = 0; i < 4; i++) {
        CHASSIS.joint_motor[i].set.tor =
            fp32_constrain(CHASSIS.joint_motor[i].set.tor, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    }
}

static void ConsolePosDebug(void)
{
    CHASSIS.joint_motor[0].set.tor = 0;
    CHASSIS.joint_motor[1].set.tor = 0;
    CHASSIS.joint_motor[2].set.tor = 0;
    CHASSIS.joint_motor[3].set.tor = 0;

    CHASSIS.joint_motor[0].set.vel = 0;
    CHASSIS.joint_motor[1].set.vel = 0;
    CHASSIS.joint_motor[2].set.vel = 0;
    CHASSIS.joint_motor[3].set.vel = 0;

    LocomotionController();

    float phi1_phi4_l[2], phi1_phi4_r[2];
    CalcPhi1AndPhi4(CHASSIS.ref.rod_Angle[0], CHASSIS.ref.rod_L0[0], phi1_phi4_l);
    CalcPhi1AndPhi4(CHASSIS.ref.rod_Angle[1], CHASSIS.ref.rod_L0[1], phi1_phi4_r);

    // 当解算出的角度正常时，设置目标角度
    if (!(isnan(phi1_phi4_l[0]) || isnan(phi1_phi4_l[1]) || isnan(phi1_phi4_r[0]) ||
          isnan(phi1_phi4_r[1]))) {
        CHASSIS.joint_motor[0].set.pos =
            theta_transform(phi1_phi4_l[0], -J0_ANGLE_OFFSET, J0_DIRECTION, 1);
        CHASSIS.joint_motor[1].set.pos =
            theta_transform(phi1_phi4_l[1], -J1_ANGLE_OFFSET, J1_DIRECTION, 1);
        CHASSIS.joint_motor[2].set.pos =
            theta_transform(phi1_phi4_r[0], -J2_ANGLE_OFFSET, J2_DIRECTION, 1);
        CHASSIS.joint_motor[3].set.pos =
            theta_transform(phi1_phi4_r[1], -J3_ANGLE_OFFSET, J3_DIRECTION, 1);
    }
    // 检测设定角度是否超过电机角度限制
    CHASSIS.joint_motor[0].set.pos =
        fp32_constrain(CHASSIS.joint_motor[0].set.pos, MIN_J0_ANGLE, MAX_J0_ANGLE);
    CHASSIS.joint_motor[1].set.pos =
        fp32_constrain(CHASSIS.joint_motor[1].set.pos, MIN_J1_ANGLE, MAX_J1_ANGLE);
    CHASSIS.joint_motor[2].set.pos =
        fp32_constrain(CHASSIS.joint_motor[2].set.pos, MIN_J2_ANGLE, MAX_J2_ANGLE);
    CHASSIS.joint_motor[3].set.pos =
        fp32_constrain(CHASSIS.joint_motor[3].set.pos, MIN_J3_ANGLE, MAX_J3_ANGLE);

    // ===驱动轮控制===
    CHASSIS.wheel_motor[0].set.tor = -(CHASSIS.cmd.leg[0].wheel.T * (W0_DIRECTION));
    CHASSIS.wheel_motor[1].set.tor = -(CHASSIS.cmd.leg[1].wheel.T * (W1_DIRECTION));

    // DEBUG:架空调试用
    // CHASSIS.wheel_motor[0].set.tor = 0;
    // CHASSIS.wheel_motor[1].set.tor = 0;
}

static void ConsoleStandUp(void)
{
    // ===腿部位置控制===

    double joint_pos_l[2], joint_pos_r[2];
    // LegController(joint_pos_l, joint_pos_r);

    // 当解算出的角度正常时，设置目标角度
    if (!(isnan(joint_pos_l[0]) || isnan(joint_pos_l[1]) || isnan(joint_pos_r[0]) ||
          isnan(joint_pos_r[1]))) {
        CHASSIS.joint_motor[0].set.pos =
            theta_transform(joint_pos_l[1], -J0_ANGLE_OFFSET, J0_DIRECTION, 1);
        CHASSIS.joint_motor[1].set.pos =
            theta_transform(joint_pos_l[0], -J1_ANGLE_OFFSET, J1_DIRECTION, 1);
        CHASSIS.joint_motor[2].set.pos =
            theta_transform(joint_pos_r[1], -J2_ANGLE_OFFSET, J2_DIRECTION, 1);
        CHASSIS.joint_motor[3].set.pos =
            theta_transform(joint_pos_r[0], -J3_ANGLE_OFFSET, J3_DIRECTION, 1);
    }
    // 检测设定角度是否超过电机角度限制
    CHASSIS.joint_motor[0].set.pos =
        fp32_constrain(CHASSIS.joint_motor[0].set.pos, MIN_J0_ANGLE, MAX_J0_ANGLE);
    CHASSIS.joint_motor[1].set.pos =
        fp32_constrain(CHASSIS.joint_motor[1].set.pos, MIN_J1_ANGLE, MAX_J1_ANGLE);
    CHASSIS.joint_motor[2].set.pos =
        fp32_constrain(CHASSIS.joint_motor[2].set.pos, MIN_J2_ANGLE, MAX_J2_ANGLE);
    CHASSIS.joint_motor[3].set.pos =
        fp32_constrain(CHASSIS.joint_motor[3].set.pos, MIN_J3_ANGLE, MAX_J3_ANGLE);

    // ===驱动轮pid控制===
    float feedforward = -220;
    PID_calc(&CHASSIS.pid.stand_up, CHASSIS.fdb.body.phi, 0);
    CHASSIS.wheel_motor[0].set.value = (feedforward + CHASSIS.pid.stand_up.out) * W0_DIRECTION;
    CHASSIS.wheel_motor[1].set.value = (feedforward + CHASSIS.pid.stand_up.out) * W1_DIRECTION;
}

/******************************************************************/
/* Cmd                                                            */
/*----------------------------------------------------------------*/
/* main function:      ChassisSendCmd                             */
/* auxiliary function: SendJointMotorCmd                          */
/*                     SendWheelMotorCmd                          */
/******************************************************************/

#define DM_DELAY 250

#define DEBUG_KP 17
#define DEBUG_KD 2

static void SendJointMotorCmd(void);
static void SendWheelMotorCmd(void);

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisSendCmd(void)
{
    SendJointMotorCmd();
    SendWheelMotorCmd();
}

/**
 * @brief 发送关节电机控制指令
 * @param[in] chassis
 */
static void SendJointMotorCmd(void)
{
    uint8_t cnt;
    if (CHASSIS.mode == CHASSIS_OFF) {
        DmMitStop(&CHASSIS.joint_motor[0]);
        DmMitStop(&CHASSIS.joint_motor[1]);
        delay_us(DM_DELAY);
        DmMitStop(&CHASSIS.joint_motor[2]);
        DmMitStop(&CHASSIS.joint_motor[3]);
    } else {
        // bool flag = false;
        for (uint8_t i = 0; i < 4; i++) {
            if (cnt % 2 == 0) {
                delay_us(DM_DELAY);
            }
            if (CHASSIS.joint_motor[i].fdb.state == DM_STATE_DISABLE) {
                DmEnable(&CHASSIS.joint_motor[i]);
                // flag = true;
                cnt++;
            }
        }

        delay_us(DM_DELAY);

        switch (CHASSIS.mode) {
            case CHASSIS_FOLLOW_GIMBAL_YAW:
            case CHASSIS_DEBUG:
            case CHASSIS_CUSTOM:
            case CHASSIS_FREE: {
                DmMitCtrlTorque(&CHASSIS.joint_motor[0]);
                DmMitCtrlTorque(&CHASSIS.joint_motor[1]);
                delay_us(DM_DELAY);
                DmMitCtrlTorque(&CHASSIS.joint_motor[2]);
                DmMitCtrlTorque(&CHASSIS.joint_motor[3]);
            } break;
            case CHASSIS_STAND_UP: {
                DmMitCtrlPosition(&CHASSIS.joint_motor[0], NORMAL_POS_KP, NORMAL_POS_KD);
                DmMitCtrlPosition(&CHASSIS.joint_motor[1], NORMAL_POS_KP, NORMAL_POS_KD);
                delay_us(DM_DELAY);
                DmMitCtrlPosition(&CHASSIS.joint_motor[2], NORMAL_POS_KP, NORMAL_POS_KD);
                DmMitCtrlPosition(&CHASSIS.joint_motor[3], NORMAL_POS_KP, NORMAL_POS_KD);
            } break;
            case CHASSIS_CALIBRATE: {
                DmMitCtrlVelocity(&CHASSIS.joint_motor[0], CALIBRATE_VEL_KP);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[1], CALIBRATE_VEL_KP);
                delay_us(DM_DELAY);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[2], CALIBRATE_VEL_KP);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[3], CALIBRATE_VEL_KP);

                if (CALIBRATE.reached[0] && CALIBRATE.reached[1] && CALIBRATE.reached[2] &&
                    CALIBRATE.reached[3]) {
                    delay_us(DM_DELAY);
                    DmSavePosZero(&CHASSIS.joint_motor[0]);
                    DmSavePosZero(&CHASSIS.joint_motor[1]);
                    delay_us(DM_DELAY);
                    DmSavePosZero(&CHASSIS.joint_motor[2]);
                    DmSavePosZero(&CHASSIS.joint_motor[3]);
                }
            } break;
            case CHASSIS_OFF_HOOK: {
                DmMitCtrlVelocity(&CHASSIS.joint_motor[0], CALIBRATE_VEL_KP);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[1], CALIBRATE_VEL_KP);
                delay_us(DM_DELAY);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[2], CALIBRATE_VEL_KP);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[3], CALIBRATE_VEL_KP);
            } break;
            case CHASSIS_POS_DEBUG: {
                DmMitCtrlPosition(&CHASSIS.joint_motor[0], DEBUG_KP, DEBUG_KD);
                DmMitCtrlPosition(&CHASSIS.joint_motor[1], DEBUG_KP, DEBUG_KD);
                delay_us(DM_DELAY);
                DmMitCtrlPosition(&CHASSIS.joint_motor[2], DEBUG_KP, DEBUG_KD);
                DmMitCtrlPosition(&CHASSIS.joint_motor[3], DEBUG_KP, DEBUG_KD);

                // DmMitCtrlVelocity(&CHASSIS.joint_motor[0], CALIBRATE_VEL_KP);
                // DmMitCtrlVelocity(&CHASSIS.joint_motor[1], CALIBRATE_VEL_KP);
                // delay_us(DM_DELAY);
                // DmMitCtrlVelocity(&CHASSIS.joint_motor[2], CALIBRATE_VEL_KP);
                // DmMitCtrlVelocity(&CHASSIS.joint_motor[3], CALIBRATE_VEL_KP);

                // DmMitCtrlTorque(&CHASSIS.joint_motor[0]);
                // DmMitCtrlTorque(&CHASSIS.joint_motor[1]);
                // delay_us(DM_DELAY);
                // DmMitCtrlTorque(&CHASSIS.joint_motor[2]);
                // DmMitCtrlTorque(&CHASSIS.joint_motor[3]);
            } break;
            case CHASSIS_SAFE:
            default: {
                DmMitCtrlVelocity(&CHASSIS.joint_motor[0], ZERO_FORCE_VEL_KP);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[1], ZERO_FORCE_VEL_KP);
                delay_us(DM_DELAY);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[2], ZERO_FORCE_VEL_KP);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[3], ZERO_FORCE_VEL_KP);
            }
        }
    }
}

/**
 * @brief 发送驱动轮电机控制指令
 * @param chassis
 */
static void SendWheelMotorCmd(void)
{
    switch (CHASSIS.mode) {
        case CHASSIS_FOLLOW_GIMBAL_YAW:
        case CHASSIS_CUSTOM:
        case CHASSIS_FREE: {
            LkMultipleTorqueControl(
                WHEEL_CAN, CHASSIS.wheel_motor[0].set.tor, CHASSIS.wheel_motor[1].set.tor, 0, 0);
        } break;
        case CHASSIS_STAND_UP: {
            LkMultipleIqControl(
                WHEEL_CAN, CHASSIS.wheel_motor[0].set.value, CHASSIS.wheel_motor[1].set.value, 0,
                0);
        } break;
        case CHASSIS_CALIBRATE: {
            LkMultipleTorqueControl(WHEEL_CAN, 0, 0, 0, 0);
        } break;
        case CHASSIS_OFF: {
            LkMultipleTorqueControl(WHEEL_CAN, 0, 0, 0, 0);
        } break;
        case CHASSIS_POS_DEBUG: {
            LkMultipleTorqueControl(
                WHEEL_CAN, CHASSIS.wheel_motor[0].set.tor, CHASSIS.wheel_motor[1].set.tor, 0, 0);
        } break;
        case CHASSIS_SAFE:
        default: {
            LkMultipleTorqueControl(
                WHEEL_CAN, CHASSIS.wheel_motor[0].set.value, CHASSIS.wheel_motor[1].set.value, 0,
                0);
        }
    }
}

/******************************************************************/
/* Public                                                         */
/*----------------------------------------------------------------*/
/* main function:      None                                       */
/* auxiliary function: SetCali                                    */
/*                     CmdCali                                    */
/*                     ChassisSetCaliData                         */
/*                     ChassisCmdCali                             */
/*                     ChassisGetStatus                           */
/*                     ChassisGetDuration                         */
/*                     ChassisGetSpeedVx                          */
/*                     ChassisGetSpeedVy                          */
/*                     ChassisGetSpeedWz                          */
/******************************************************************/

void SetCali(const fp32 motor_middle[4]) {}

bool_t CmdCali(fp32 motor_middle[4])
{
    // if (CALIBRATE.calibrated) {  // 校准完成
    //     CHASSIS.mode = CHASSIS_SAFE;
    //     CALIBRATE.calibrated = false;
    //     return true;
    // }

    // if (CHASSIS.mode != CHASSIS_CALIBRATE) {
    //     CALIBRATE.toggle = true;
    // }

    // return false;

    return true;
}

void ChassisSetCaliData(const fp32 motor_middle[4]) { SetCali(motor_middle); }

bool_t ChassisCmdCali(fp32 motor_middle[4]) { return CmdCali(motor_middle); }

inline uint8_t ChassisGetStatus(void) { return 0; }
inline uint32_t ChassisGetDuration(void) { return CHASSIS.duration; }
inline float ChassisGetSpeedVx(void) { return CHASSIS.fdb.speed_vector.vx; }
inline float ChassisGetSpeedVy(void) { return CHASSIS.fdb.speed_vector.vy; }
inline float ChassisGetSpeedWz(void) { return CHASSIS.fdb.speed_vector.wz; }
#endif /* CHASSIS_BALANCE */
/*------------------------------ End of File ------------------------------*/
