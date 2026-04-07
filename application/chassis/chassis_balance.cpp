/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_balance.cpp
  * @brief      平衡底盘控制器 (重构适配 DaMiao MC02 H7 平台)
  * @history
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *  V1.0.2     Sep-16-2024     Penguin         1. 添加速度观测器并测试效果
  *  V1.0.3     Nov-20-2024     Penguin         1. 完善离地检测
  *  V2.0.0     2025            Refactor        适配H7新中间件(C++)
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/
#include "chassis_balance.h"
#if (CHASSIS_TYPE == CHASSIS_BALANCE)

#include "chassis.h"
#include "chassis_balance_extras.h"
#include "cmsis_os.h"
#include "data_exchange.h"
#include "drv_can.h"
#include "fdcan.h"
#include "macro_typedef.h"
#include "usb_debug.h"
#include "gimbal.h"
#include "IMU.h"
#include "alg_basic.h"
#include <cmath>
#include <cstring>

// Gimbal functions may not be available if GIMBAL_TYPE == GIMBAL_NONE
// Provide weak stubs so chassis can compile independently
#if GIMBAL_TYPE == GIMBAL_NONE
static float GetGimbalDeltaYawMid(void) { return 0.0f; }
static bool GetGimbalInitJudgeReturn(void) { return false; }
#endif

// 一些内部的配置
#define TAKE_OFF_DETECT 0
#define CLOSE_LEG_LEFT 0
#define CLOSE_LEG_RIGHT 0
#define LIFTED_UP 0

#define MS_TO_S 0.001f

#define CALIBRATE_STOP_VELOCITY 0.05f
#define CALIBRATE_STOP_TIME 200
#define CALIBRATE_VELOCITY 2.0f

#define VEL_PROCESS_NOISE 25
#define VEL_MEASURE_NOISE 800
#define ACC_PROCESS_NOISE 2000
#define ACC_MEASURE_NOISE 0.01

#define RC_OFF_HOOK_VALUE_HOLE 650

#define TAKE_OFF_FN_THRESHOLD (3.0f)
#define TOUCH_TOGGLE_THRESHOLD (100)

// clang-format off
#define NORMAL_STEP        0
#define JUMP_STEP_SQUST    1
#define JUMP_STEP_JUMP     2
#define JUMP_STEP_RECOVERY 3
// clang-format on

#define MAX_STEP_TIME           5000

#define rc_deadband_limit(input, output, dealine)          \
    {                                                      \
        if ((input) > (dealine) || (input) < -(dealine)) { \
            (output) = (input);                            \
        } else {                                           \
            (output) = 0;                                  \
        }                                                  \
    }

/*-------------------- LK (MF9025) wheel motor CAN protocol --------------------*/
// 瓴控电机CAN协议: 多电机力矩控制 (0x280+id偏移)
// Data[0..1] = motor1 torque (int16_t), Data[2..3] = motor2, ...
// 单位: 原始值映射到力矩

static void LkMultipleTorqueControl(uint8_t can_bus, float t1, float t2, float t3, float t4)
{
    FDCAN_HandleTypeDef *hcan = (can_bus == 1) ? &hfdcan1 : (can_bus == 2) ? &hfdcan2 : &hfdcan3;
    uint8_t data[8];
    int16_t v1 = (int16_t)(t1 * 2000.0f / 33.0f);   // MF9025 torque scale
    int16_t v2 = (int16_t)(t2 * 2000.0f / 33.0f);
    int16_t v3 = (int16_t)(t3 * 2000.0f / 33.0f);
    int16_t v4 = (int16_t)(t4 * 2000.0f / 33.0f);
    data[0] = (uint8_t)(v1 & 0xFF);
    data[1] = (uint8_t)((v1 >> 8) & 0xFF);
    data[2] = (uint8_t)(v2 & 0xFF);
    data[3] = (uint8_t)((v2 >> 8) & 0xFF);
    data[4] = (uint8_t)(v3 & 0xFF);
    data[5] = (uint8_t)((v3 >> 8) & 0xFF);
    data[6] = (uint8_t)(v4 & 0xFF);
    data[7] = (uint8_t)((v4 >> 8) & 0xFF);
    CAN_Transmit_Data(hcan, 0x280, data, 8);
}

static void LkMultipleIqControl(uint8_t can_bus, float iq1, float iq2, float iq3, float iq4)
{
    FDCAN_HandleTypeDef *hcan = (can_bus == 1) ? &hfdcan1 : (can_bus == 2) ? &hfdcan2 : &hfdcan3;
    uint8_t data[8];
    int16_t v1 = (int16_t)(iq1);
    int16_t v2 = (int16_t)(iq2);
    int16_t v3 = (int16_t)(iq3);
    int16_t v4 = (int16_t)(iq4);
    data[0] = (uint8_t)(v1 & 0xFF);
    data[1] = (uint8_t)((v1 >> 8) & 0xFF);
    data[2] = (uint8_t)(v2 & 0xFF);
    data[3] = (uint8_t)((v2 >> 8) & 0xFF);
    data[4] = (uint8_t)(v3 & 0xFF);
    data[5] = (uint8_t)((v3 >> 8) & 0xFF);
    data[6] = (uint8_t)(v4 & 0xFF);
    data[7] = (uint8_t)((v4 >> 8) & 0xFF);
    CAN_Transmit_Data(hcan, 0x280, data, 8);
}

/*-------------------- Microsecond delay (DWT-based) --------------------*/

static inline void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) {}
}

/*-------------------- PID helper --------------------*/

static inline void PID_Init_From_Params(Class_PID *pid, float kp, float ki, float kd,
                                         float max_out, float max_iout, float dt = 0.002f)
{
    pid->Init(kp, ki, kd, 0.0f, max_iout, max_out, dt);
}

static inline float PID_Calc(Class_PID *pid, float now, float target)
{
    pid->Set_Now(now);
    pid->Set_Target(target);
    pid->TIM_Calculate_PeriodElapsedCallback();
    return pid->Get_Out();
}

static inline void PID_Clear(Class_PID *pid)
{
    pid->Set_Integral_Error(0.0f);
}

/*-------------------- Static variables --------------------*/

static Calibrate_s CALIBRATE = {
    .cali_cnt = 0,
    .velocity = {0.0f, 0.0f, 0.0f, 0.0f},
    .stpo_time = {0, 0, 0, 0},
    .reached = {false, false, false, false},
    .calibrated = false,
    .toggle = false,
};

static Observer_t OBSERVER;

Chassis_s CHASSIS;

int8_t TRANSITION_MATRIX[10] = {0};

/*-------------------- Publish --------------------*/

void ChassisPublish(void) { Publish(&CHASSIS.fdb.speed_vector, (char *)CHASSIS_FDB_SPEED_NAME); }

/******************************************************************/
/* Init                                                           */
/******************************************************************/

void ChassisInit(void)
{
    // 清零关键字段
    CHASSIS.mode = CHASSIS_OFF;
    CHASSIS.error_code = 0;
    CHASSIS.yaw_mid = 0;
    CHASSIS.dyaw = 0.0f;

    CHASSIS.rc = get_remote_control_point();
    CHASSIS.imu = (const Imu_t *)Subscribe((char *)IMU_NAME);

    /*-------------------- 初始化状态转移矩阵 --------------------*/
    TRANSITION_MATRIX[NORMAL_STEP] = NORMAL_STEP;
    TRANSITION_MATRIX[JUMP_STEP_SQUST] = JUMP_STEP_JUMP;
    TRANSITION_MATRIX[JUMP_STEP_JUMP] = JUMP_STEP_RECOVERY;
    TRANSITION_MATRIX[JUMP_STEP_RECOVERY] = NORMAL_STEP;

    /*-------------------- 初始化关节电机 (DM8009 MIT mode) --------------------*/
    // DM8009: Angle_Max=12.5, Omega_Max=25.0, Torque_Max=10.0
    FDCAN_HandleTypeDef *joint_hcan = (JOINT_CAN == 1) ? &hfdcan1 : (JOINT_CAN == 2) ? &hfdcan2 : &hfdcan3;
    // Motor ID: CAN_Rx_ID = 0x00+id, CAN_Tx_ID = id (达妙MIT模式)
    CHASSIS.joint_motor[0].Init(joint_hcan, 0x01, 0x01, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 25.0f, 10.0f);
    CHASSIS.joint_motor[1].Init(joint_hcan, 0x02, 0x02, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 25.0f, 10.0f);
    CHASSIS.joint_motor[2].Init(joint_hcan, 0x03, 0x03, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 25.0f, 10.0f);
    CHASSIS.joint_motor[3].Init(joint_hcan, 0x04, 0x04, Motor_DM_Control_Method_NORMAL_MIT, 12.5f, 25.0f, 10.0f);

    /*-------------------- 值归零 --------------------*/
    memset(&CHASSIS.fdb, 0, sizeof(CHASSIS.fdb));
    memset(&CHASSIS.ref, 0, sizeof(CHASSIS.ref));
    CHASSIS.fdb.leg[0].is_take_off = false;
    CHASSIS.fdb.leg[1].is_take_off = false;

    /*-------------------- 初始化底盘PID --------------------*/
    PID_Init_From_Params(&CHASSIS.pid.yaw_angle,
        KP_CHASSIS_YAW_ANGLE, KI_CHASSIS_YAW_ANGLE, KD_CHASSIS_YAW_ANGLE,
        MAX_OUT_CHASSIS_YAW_ANGLE, MAX_IOUT_CHASSIS_YAW_ANGLE);
    PID_Init_From_Params(&CHASSIS.pid.yaw_velocity,
        KP_CHASSIS_YAW_VELOCITY, KI_CHASSIS_YAW_VELOCITY, KD_CHASSIS_YAW_VELOCITY,
        MAX_OUT_CHASSIS_YAW_VELOCITY, MAX_IOUT_CHASSIS_YAW_VELOCITY);
    PID_Init_From_Params(&CHASSIS.pid.vel_add,
        KP_CHASSIS_VEL_ADD, KI_CHASSIS_VEL_ADD, KD_CHASSIS_VEL_ADD,
        MAX_OUT_CHASSIS_VEL_ADD, MAX_IOUT_CHASSIS_VEL_ADD);
    PID_Init_From_Params(&CHASSIS.pid.roll_angle,
        KP_CHASSIS_ROLL_ANGLE, KI_CHASSIS_ROLL_ANGLE, KD_CHASSIS_ROLL_ANGLE,
        MAX_OUT_CHASSIS_ROLL_ANGLE, MAX_IOUT_CHASSIS_ROLL_ANGLE);
    PID_Init_From_Params(&CHASSIS.pid.leg_length_length[0],
        KP_CHASSIS_LEG_LENGTH_LENGTH, KI_CHASSIS_LEG_LENGTH_LENGTH, KD_CHASSIS_LEG_LENGTH_LENGTH,
        MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH, MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH);
    PID_Init_From_Params(&CHASSIS.pid.leg_length_length[1],
        KP_CHASSIS_LEG_LENGTH_LENGTH, KI_CHASSIS_LEG_LENGTH_LENGTH, KD_CHASSIS_LEG_LENGTH_LENGTH,
        MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH, MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH);
    PID_Init_From_Params(&CHASSIS.pid.stand_up,
        KP_CHASSIS_STAND_UP, KI_CHASSIS_STAND_UP, KD_CHASSIS_STAND_UP,
        MAX_OUT_CHASSIS_STAND_UP, MAX_IOUT_CHASSIS_STAND_UP);
    PID_Init_From_Params(&CHASSIS.pid.wheel_stop[0],
        KP_CHASSIS_WHEEL_STOP, KI_CHASSIS_WHEEL_STOP, KD_CHASSIS_WHEEL_STOP,
        MAX_OUT_CHASSIS_WHEEL_STOP, MAX_IOUT_CHASSIS_WHEEL_STOP);
    PID_Init_From_Params(&CHASSIS.pid.wheel_stop[1],
        KP_CHASSIS_WHEEL_STOP, KI_CHASSIS_WHEEL_STOP, KD_CHASSIS_WHEEL_STOP,
        MAX_OUT_CHASSIS_WHEEL_STOP, MAX_IOUT_CHASSIS_WHEEL_STOP);
    PID_Init_From_Params(&CHASSIS.pid.chassis_follow_gimbal,
        KP_CHASSIS_FOLLOW_GIMBAL, KI_CHASSIS_FOLLOW_GIMBAL, KD_CHASSIS_FOLLOW_GIMBAL,
        MAX_OUT_CHASSIS_FOLLOW_GIMBAL, MAX_IOUT_CHASSIS_FOLLOW_GIMBAL);

    /*-------------------- 初始化低通滤波器 --------------------*/
    LowPassFilterInit(&CHASSIS.lpf.leg_l0_accel_filter[0], LEG_DDL0_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.leg_l0_accel_filter[1], LEG_DDL0_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.leg_phi0_accel_filter[0], LEG_DDPHI0_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.leg_phi0_accel_filter[1], LEG_DDPHI0_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.leg_theta_accel_filter[0], LEG_DDTHETA_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.leg_theta_accel_filter[1], LEG_DDTHETA_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.support_force_filter[0], LEG_SUPPORT_FORCE_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.support_force_filter[1], LEG_SUPPORT_FORCE_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.roll, CHASSIS_ROLL_ALPHA);

    /*-------------------- 初始化机体速度观测器 --------------------*/
    float dt = CHASSIS_CONTROL_TIME_S;
    // 状态方程: x_k = A*x_{k-1} + B*u_k
    // 状态: [velocity, acceleration]
    // A = [1 dt; 0 1], B = 0, H = [1 0; 0 1]
    float A_data[4] = {1.0f, dt, 0.0f, 1.0f};
    float B_data[2] = {0.0f, 0.0f};
    float H_data[4] = {1.0f, 0.0f, 0.0f, 1.0f};
    float Q_data[4] = {(float)VEL_PROCESS_NOISE, 0.0f, 0.0f, (float)ACC_PROCESS_NOISE};
    float R_data[4] = {(float)VEL_MEASURE_NOISE, 0.0f, 0.0f, (float)ACC_MEASURE_NOISE};
    float P_data[4] = {100000.0f, 0.0f, 0.0f, 100000.0f};

    Class_Matrix_f32<2, 2> mat_A(A_data);
    Class_Matrix_f32<2, 1> mat_B(B_data);
    Class_Matrix_f32<2, 2> mat_H(H_data);
    Class_Matrix_f32<2, 2> mat_Q(Q_data);
    Class_Matrix_f32<2, 2> mat_R(R_data);
    Class_Matrix_f32<2, 2> mat_P(P_data);

    OBSERVER.body.v_kf.Init(mat_A, mat_B, mat_H, mat_Q, mat_R, mat_P);
}

/******************************************************************/
/* Handle exception                                               */
/******************************************************************/

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
        if (fabsf(CHASSIS.joint_motor[i].Get_Now_Torque()) > MAX_TORQUE_PROTECT) {
            CHASSIS.error_code |= JOINT_ERROR_OFFSET;
            break;
        }
    }

    if ((CHASSIS.mode == CHASSIS_OFF || CHASSIS.mode == CHASSIS_SAFE)) {
        PID_Clear(&CHASSIS.pid.stand_up);
    }
}

/******************************************************************/
/* Set mode                                                       */
/******************************************************************/

void ChassisSetMode(void)
{
    if (CHASSIS.error_code & DBUS_ERROR_OFFSET) {
        CHASSIS.mode = CHASSIS_SAFE;
        return;
    }
    if (CHASSIS.error_code & IMU_ERROR_OFFSET) {
        CHASSIS.mode = CHASSIS_SAFE;
        return;
    }
    if (CHASSIS.error_code & JOINT_ERROR_OFFSET) {
        CHASSIS.mode = CHASSIS_SAFE;
        return;
    }
    if (CHASSIS.mode == CHASSIS_CALIBRATE && (!CALIBRATE.calibrated)) {
        return;
    }
    if (CALIBRATE.toggle) {
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
    for (uint8_t i = 0; i < 2; i++) {
        if (CHASSIS.fdb.leg[i].is_take_off &&
            CHASSIS.fdb.leg[i].touch_time > TOUCH_TOGGLE_THRESHOLD) {
            CHASSIS.fdb.leg[i].is_take_off = false;
        } else if (!CHASSIS.fdb.leg[i].is_take_off &&
                   CHASSIS.fdb.leg[i].take_off_time > TOUCH_TOGGLE_THRESHOLD) {
            CHASSIS.fdb.leg[i].is_take_off = true;
        }
    }
#endif

    if (switch_is_up(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        CHASSIS.mode = CHASSIS_SAFE;
    } else if (switch_is_mid(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        CHASSIS.mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    } else if (switch_is_down(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
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
/******************************************************************/

#define ZERO_POS_THRESHOLD 0.001f

static void UpdateBodyStatus(void);
static void UpdateLegStatus(void);
static void UpdateMotorStatus(void);
static void UpdateCalibrateStatus(void);
static void UpdateStepStatus(void);
static void BodyMotionObserve(void);

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

    float F0_Tp[2];
    GetLegForce(CHASSIS.fdb.leg[0].J, CHASSIS.fdb.leg[0].joint.T1, CHASSIS.fdb.leg[0].joint.T2, F0_Tp);
}

static void UpdateMotorStatus(void)
{
    // DM关节电机: 从 Class_Motor_DM_Normal 获取反馈
    // 反馈数据通过CAN回调自动更新

    // 轮子电机: LK MF9025 反馈数据通过CAN回调更新
    // 此处不需要额外操作
}

static void UpdateBodyStatus(void)
{
    CHASSIS.fdb.body.roll = GetImuAngle(AX_ROLL);
    CHASSIS.fdb.body.roll_dot = GetImuVelocity(AX_ROLL);
    CHASSIS.fdb.body.pitch = GetImuAngle(AX_PITCH);
    CHASSIS.fdb.body.pitch_dot = GetImuVelocity(AX_PITCH);
    CHASSIS.fdb.body.yaw = GetImuAngle(AX_YAW);
    CHASSIS.fdb.body.yaw_dot = GetImuVelocity(AX_YAW);

    LowPassFilterCalc(&CHASSIS.lpf.roll, CHASSIS.fdb.body.roll);

    float ax = GetImuAccel(AX_X);
    float ay = GetImuAccel(AX_Y);
    float az = GetImuAccel(AX_Z);

    float cos_roll = cosf(CHASSIS.fdb.body.roll);
    float sin_roll = sinf(CHASSIS.fdb.body.roll);
    float cos_pitch = cosf(CHASSIS.fdb.body.pitch);
    float sin_pitch = sinf(CHASSIS.fdb.body.pitch);
    float cos_yaw = cosf(CHASSIS.fdb.body.yaw);
    float sin_yaw = sinf(CHASSIS.fdb.body.yaw);

    CHASSIS.fdb.body.gx = GRAVITY * sin_pitch;
    CHASSIS.fdb.body.gy = -GRAVITY * sin_roll * cos_pitch;
    CHASSIS.fdb.body.gz = -GRAVITY * cos_roll * cos_pitch;

    CHASSIS.fdb.body.x_accel = ax + CHASSIS.fdb.body.gx;
    CHASSIS.fdb.body.y_accel = ay + CHASSIS.fdb.body.gy;
    CHASSIS.fdb.body.z_accel = az + CHASSIS.fdb.body.gz;

    // clang-format off
    float R[3][3] = {
        {cos_pitch * cos_yaw, sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw, cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw},
        {cos_pitch * sin_yaw, sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw, cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw},
        {-sin_pitch         , sin_roll * cos_pitch                               , cos_roll * cos_pitch                               }
    };
    // clang-format on

    CHASSIS.fdb.world.x_accel = R[0][0] * ax + R[0][1] * ay + R[0][2] * az;
    CHASSIS.fdb.world.y_accel = R[1][0] * ax + R[1][1] * ay + R[1][2] * az;
    CHASSIS.fdb.world.z_accel = R[2][0] * ax + R[2][1] * ay + R[2][2] * az - GRAVITY;

    CHASSIS.fdb.body.phi = -CHASSIS.fdb.body.pitch;
    CHASSIS.fdb.body.phi_dot = -CHASSIS.fdb.body.pitch_dot;
    CHASSIS.fdb.body.x_acc = CHASSIS.fdb.body.x_accel;
}

static void UpdateLegStatus(void)
{
    uint8_t i = 0;
    // =====更新关节姿态=====
    CHASSIS.fdb.leg[0].joint.Phi1 =
        theta_transform(CHASSIS.joint_motor[0].Get_Now_Angle(), J0_ANGLE_OFFSET, J0_DIRECTION, 1);
    CHASSIS.fdb.leg[0].joint.Phi4 =
        theta_transform(CHASSIS.joint_motor[1].Get_Now_Angle(), J1_ANGLE_OFFSET, J1_DIRECTION, 1);
    CHASSIS.fdb.leg[1].joint.Phi1 =
        theta_transform(CHASSIS.joint_motor[2].Get_Now_Angle(), J2_ANGLE_OFFSET, J2_DIRECTION, 1);
    CHASSIS.fdb.leg[1].joint.Phi4 =
        theta_transform(CHASSIS.joint_motor[3].Get_Now_Angle(), J3_ANGLE_OFFSET, J3_DIRECTION, 1);

    CHASSIS.fdb.leg[0].joint.dPhi1 = CHASSIS.joint_motor[0].Get_Now_Omega() * (J0_DIRECTION);
    CHASSIS.fdb.leg[0].joint.dPhi4 = CHASSIS.joint_motor[1].Get_Now_Omega() * (J1_DIRECTION);
    CHASSIS.fdb.leg[1].joint.dPhi1 = CHASSIS.joint_motor[2].Get_Now_Omega() * (J2_DIRECTION);
    CHASSIS.fdb.leg[1].joint.dPhi4 = CHASSIS.joint_motor[3].Get_Now_Omega() * (J3_DIRECTION);

    CHASSIS.fdb.leg[0].joint.T1 = CHASSIS.joint_motor[0].Get_Now_Torque() * (J0_DIRECTION);
    CHASSIS.fdb.leg[0].joint.T2 = CHASSIS.joint_motor[1].Get_Now_Torque() * (J1_DIRECTION);
    CHASSIS.fdb.leg[1].joint.T1 = CHASSIS.joint_motor[2].Get_Now_Torque() * (J2_DIRECTION);
    CHASSIS.fdb.leg[1].joint.T2 = CHASSIS.joint_motor[3].Get_Now_Torque() * (J3_DIRECTION);

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

        GetL0AndPhi0(CHASSIS.fdb.leg[i].joint.Phi1, CHASSIS.fdb.leg[i].joint.Phi4, L0_Phi0);
        CHASSIS.fdb.leg[i].rod.L0 = L0_Phi0[0];
        CHASSIS.fdb.leg[i].rod.Phi0 = L0_Phi0[1];
        CHASSIS.fdb.leg[i].rod.Theta = (float)M_PI_2 - CHASSIS.fdb.leg[i].rod.Phi0 - CHASSIS.fdb.body.phi;

        CalcJacobian(CHASSIS.fdb.leg[i].joint.Phi1, CHASSIS.fdb.leg[i].joint.Phi4, CHASSIS.fdb.leg[i].J);

        GetdL0AnddPhi0(CHASSIS.fdb.leg[i].J, CHASSIS.fdb.leg[i].joint.dPhi1, CHASSIS.fdb.leg[i].joint.dPhi4, dL0_dPhi0);
        CHASSIS.fdb.leg[i].rod.dL0 = dL0_dPhi0[0];
        CHASSIS.fdb.leg[i].rod.dPhi0 = dL0_dPhi0[1];
        CHASSIS.fdb.leg[i].rod.dTheta = -CHASSIS.fdb.leg[i].rod.dPhi0 - CHASSIS.fdb.body.phi_dot;

        float accel = (CHASSIS.fdb.leg[i].rod.dL0 - last_dL0) / (CHASSIS.duration * MS_TO_S);
        CHASSIS.fdb.leg[i].rod.ddL0 = accel;
        accel = (CHASSIS.fdb.leg[i].rod.dPhi0 - last_dPhi0) / (CHASSIS.duration * MS_TO_S);
        CHASSIS.fdb.leg[i].rod.ddPhi0 = accel;
        accel = (CHASSIS.fdb.leg[i].rod.dTheta - last_dTheta) / (CHASSIS.duration * MS_TO_S);
        CHASSIS.fdb.leg[i].rod.ddTheta = accel;

        float ddot_z_M = CHASSIS.fdb.world.z_accel;
        float l0 = CHASSIS.fdb.leg[i].rod.L0;
        float v_l0 = CHASSIS.fdb.leg[i].rod.dL0;
        float leg_theta = CHASSIS.fdb.leg[i].rod.Theta;
        float w_theta = CHASSIS.fdb.leg[i].rod.dTheta;
        float dot_v_l0 = CHASSIS.fdb.leg[i].rod.ddL0;
        float dot_w_theta = CHASSIS.fdb.leg[i].rod.ddTheta;

        // clang-format off
        float ddot_z_w = ddot_z_M
                    - dot_v_l0 * cosf(leg_theta)
                    + 2.0f * v_l0 * w_theta * sinf(leg_theta)
                    + l0 * dot_w_theta * sinf(leg_theta)
                    + l0 * w_theta * w_theta * cosf(leg_theta);
        // clang-format on

        float F[2];
        GetLegForce(CHASSIS.fdb.leg[i].J, CHASSIS.fdb.leg[i].joint.T1, CHASSIS.fdb.leg[i].joint.T2, F);
        float F0 = F[0];
        float Tp = F[1];

        float P = F0 * cosf(leg_theta) + Tp * sinf(leg_theta) / l0;
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
        fabsf(CHASSIS.joint_motor[0].Get_Now_Angle()) < ZERO_POS_THRESHOLD &&
        fabsf(CHASSIS.joint_motor[1].Get_Now_Angle()) < ZERO_POS_THRESHOLD &&
        fabsf(CHASSIS.joint_motor[2].Get_Now_Angle()) < ZERO_POS_THRESHOLD &&
        fabsf(CHASSIS.joint_motor[3].Get_Now_Angle()) < ZERO_POS_THRESHOLD) {
        CALIBRATE.calibrated = true;
    }

    uint32_t now = HAL_GetTick();
    if (CHASSIS.mode == CHASSIS_CALIBRATE) {
        for (uint8_t i = 0; i < 4; i++) {
            CALIBRATE.velocity[i] = CHASSIS.joint_motor[i].Get_Now_Omega();
            if (CALIBRATE.velocity[i] > CALIBRATE_STOP_VELOCITY) {
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
        if (0 && (GetDt7RcCh(DT7_CH_RH) < -0.9f)) {
            CHASSIS.step_time = 0;
            CHASSIS.step = JUMP_STEP_SQUST;
        } else if (CHASSIS.step == JUMP_STEP_SQUST) {
            if (CHASSIS.fdb.leg[0].rod.L0 < MIN_LEG_LENGTH + 0.02f &&
                CHASSIS.fdb.leg[1].rod.L0 < MIN_LEG_LENGTH + 0.02f) {
                StateTransfer();
            }
        } else if (CHASSIS.step == JUMP_STEP_JUMP) {
            if (CHASSIS.fdb.leg[0].rod.L0 > MAX_LEG_LENGTH - 0.03f &&
                CHASSIS.fdb.leg[1].rod.L0 > MAX_LEG_LENGTH - 0.03f) {
                StateTransfer();
            }
        } else if (CHASSIS.step == JUMP_STEP_RECOVERY) {
            if (CHASSIS.step_time > 1000) {
                StateTransfer();
            }
        } else if (CHASSIS.step != NORMAL_STEP && CHASSIS.step_time > MAX_STEP_TIME) {
            CHASSIS.step_time = 0;
            CHASSIS.step = NORMAL_STEP;
        }
    } else {
        CHASSIS.step_time = 0;
        CHASSIS.step = NORMAL_STEP;
    }
}
#undef StateTransfer

static void BodyMotionObserve(void)
{
    float speed = WHEEL_RADIUS * (CHASSIS.fdb.leg[0].wheel.Velocity + CHASSIS.fdb.leg[1].wheel.Velocity) / 2;

    // 更新测量向量
    OBSERVER.body.v_kf.Vector_Z.Data[0] = speed;
    OBSERVER.body.v_kf.Vector_Z.Data[1] = CHASSIS.fdb.body.x_acc;
    // 更新状态转移矩阵中的dt
    OBSERVER.body.v_kf.Matrix_A.Data[1] = CHASSIS.duration * MS_TO_S;

    OBSERVER.body.v_kf.TIM_Predict_PeriodElapsedCallback();
    OBSERVER.body.v_kf.TIM_Update_PeriodElapsedCallback();

    CHASSIS.fdb.body.x_dot_obv = OBSERVER.body.v_kf.Vector_X.Data[0];
    CHASSIS.fdb.body.x_acc_obv = OBSERVER.body.v_kf.Vector_X.Data[1];

    if (fabsf(CHASSIS.ref.speed_vector.vx) < WHEEL_DEADZONE &&
        fabsf(CHASSIS.fdb.body.x_dot_obv) < 0.8f) {
        CHASSIS.fdb.body.x += CHASSIS.fdb.body.x_dot_obv * CHASSIS.duration * MS_TO_S;
    }

    for (uint8_t i = 0; i < 2; i++) {
        // clang-format off
        CHASSIS.fdb.leg_state[i].theta     =  (float)M_PI_2 - CHASSIS.fdb.leg[i].rod.Phi0 - CHASSIS.fdb.body.phi;
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
/******************************************************************/

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

    ChassisSpeedVector_t v_set = {0.0f, 0.0f, 0.0f};
    v_set.vx = rc_x * RC_TO_ONE * MAX_SPEED_VECTOR_VX;
    v_set.vy = 0;
    v_set.wz = -rc_wz * RC_TO_ONE * MAX_SPEED_VECTOR_WZ;

    switch (CHASSIS.mode) {
        case CHASSIS_FREE: {
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
        case CHASSIS_FOLLOW_GIMBAL_YAW: {
            float delta_yaw = GetGimbalDeltaYawMid();
            CHASSIS.ref.speed_vector.vx = v_set.vx * cosf(delta_yaw);
            CHASSIS.ref.speed_vector.vy = 0;
            if (GetGimbalInitJudgeReturn()) {
                CHASSIS.ref.speed_vector.wz = 0;
            } else {
                CHASSIS.ref.speed_vector.wz = PID_Calc(&CHASSIS.pid.chassis_follow_gimbal, -delta_yaw, 0);
            }
        } break;
        case CHASSIS_AUTO: {
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

    // clang-format off
    for (uint8_t i = 0; i < 2; i++) {
        CHASSIS.ref.leg_state[i].theta     = 0;
        CHASSIS.ref.leg_state[i].theta_dot = 0;
        CHASSIS.ref.leg_state[i].x         = 0;
        CHASSIS.ref.leg_state[i].x_dot     = CHASSIS.ref.speed_vector.vx;
        CHASSIS.ref.leg_state[i].phi       = 0;
        CHASSIS.ref.leg_state[i].phi_dot   = 0;
    }
    // clang-format on

    static float angle = (float)M_PI_2;
    static float length = 0.12f;
    switch (CHASSIS.mode) {
        case CHASSIS_STAND_UP: {
            length = 0.12f;
            angle = (float)M_PI_2;
        } break;
        case CHASSIS_DEBUG: {
            length = 0.12f;
            CHASSIS.ref.leg_state[0].theta = rc_angle * RC_TO_ONE * 0.3f;
            CHASSIS.ref.leg_state[1].theta = rc_angle * RC_TO_ONE * 0.3f;
        }
        /* fall through */
        case CHASSIS_FOLLOW_GIMBAL_YAW:
        case CHASSIS_CUSTOM:
        case CHASSIS_POS_DEBUG: {
            angle = (float)M_PI_2 + rc_angle * RC_TO_ONE * 0.3f;
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
            angle = (float)M_PI_2;
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
/******************************************************************/

static void LocomotionController(void);
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

void ChassisConsole(void)
{
    switch (CHASSIS.mode) {
        case CHASSIS_CALIBRATE: ConsoleCalibrate(); break;
        case CHASSIS_OFF_HOOK: ConsoleOffHook(); break;
        case CHASSIS_FOLLOW_GIMBAL_YAW:
        case CHASSIS_CUSTOM:
        case CHASSIS_FREE: ConsoleNormal(); break;
        case CHASSIS_DEBUG: ConsoleDebug(); break;
        case CHASSIS_POS_DEBUG: ConsolePosDebug(); break;
        case CHASSIS_STAND_UP: ConsoleStandUp(); break;
        case CHASSIS_OFF:
        case CHASSIS_SAFE:
        default: ConsoleZeroForce(); break;
    }

#if CLOSE_LEG_LEFT
    CHASSIS.joint_motor[0].Set_Control_Torque(0);
    CHASSIS.joint_motor[1].Set_Control_Torque(0);
    CHASSIS.wheel_motor[0].set.tor = 0;
    CHASSIS.wheel_motor[0].set.value = 0;
#endif

#if CLOSE_LEG_RIGHT
    CHASSIS.joint_motor[2].Set_Control_Torque(0);
    CHASSIS.joint_motor[3].Set_Control_Torque(0);
    CHASSIS.wheel_motor[1].set.tor = 0;
    CHASSIS.wheel_motor[1].set.value = 0;
#endif

#if LIFTED_UP
    CHASSIS.wheel_motor[0].set.tor = 0;
    CHASSIS.wheel_motor[1].set.tor = 0;
    CHASSIS.wheel_motor[0].set.value = 0;
    CHASSIS.wheel_motor[1].set.value = 0;
#endif
}

static void LocomotionController(void)
{
    float k[2][6];
    float x[6];
    float T_Tp[2];
    bool is_take_off = CHASSIS.fdb.leg[0].is_take_off || CHASSIS.fdb.leg[1].is_take_off;
#if LIFTED_UP
    is_take_off = true;
#endif

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

    // ROLL角控制
    float Ld0 = CHASSIS.fdb.leg[0].rod.L0 - CHASSIS.fdb.leg[1].rod.L0;
    float L_diff = CalcLegLengthDiff(Ld0, CHASSIS.fdb.body.roll, CHASSIS.ref.body.roll);
    float delta_L0 = 0.0f;
    CoordinateLegLength(&CHASSIS.ref.rod_L0[0], &CHASSIS.ref.rod_L0[1], L_diff, delta_L0);

    // 转向控制
    if (!is_take_off) {
        PID_Calc(&CHASSIS.pid.yaw_velocity, CHASSIS.fdb.body.yaw_dot, CHASSIS.ref.speed_vector.wz);
        CHASSIS.cmd.leg[0].wheel.T += CHASSIS.pid.yaw_velocity.Get_Out();
        CHASSIS.cmd.leg[1].wheel.T -= CHASSIS.pid.yaw_velocity.Get_Out();
    }
}

static void LegTorqueController(void)
{
    float F_ff, F_compensate;
    float roll_vel_limit_f = fp32_constrain(CHASSIS.fdb.body.roll_dot * ROLL_VEL_LIMIT_FACTOR, -0.2f, 0.2f);

    for (uint8_t i = 0; i < 2; i++) {
        if (CHASSIS.step == JUMP_STEP_JUMP) {
            CHASSIS.cmd.leg[i].rod.F = 40;
        } else {
            F_ff = LegFeedForward(CHASSIS.fdb.leg_state[i].theta) * FF_RATIO;
            F_compensate = PID_Calc(&CHASSIS.pid.leg_length_length[i],
                                     CHASSIS.fdb.leg[i].rod.L0, CHASSIS.ref.rod_L0[i]);
            CHASSIS.cmd.leg[i].rod.F = F_ff + F_compensate;
        }
    }

    CHASSIS.cmd.leg[0].rod.F -= roll_vel_limit_f;
    CHASSIS.cmd.leg[1].rod.F += roll_vel_limit_f;

    CalcVmc(CHASSIS.cmd.leg[0].rod.F, CHASSIS.cmd.leg[0].rod.Tp, CHASSIS.fdb.leg[0].J, CHASSIS.cmd.leg[0].joint.T);
    CalcVmc(CHASSIS.cmd.leg[1].rod.F, CHASSIS.cmd.leg[1].rod.Tp, CHASSIS.fdb.leg[1].J, CHASSIS.cmd.leg[1].joint.T);
}

static float LegFeedForward(float theta) { return BODY_MASS * GRAVITY * cosf(theta) / 2; }

static void CalcLQR(float k[2][6], float x[6], float T_Tp[2])
{
    T_Tp[0] = k[0][0]*x[0] + k[0][1]*x[1] + k[0][2]*x[2] + k[0][3]*x[3] + k[0][4]*x[4] + k[0][5]*x[5];
    T_Tp[1] = k[1][0]*x[0] + k[1][1]*x[1] + k[1][2]*x[2] + k[1][3]*x[3] + k[1][4]*x[4] + k[1][5]*x[5];
}

//* 各个模式下的控制

// Helper: set joint motor for torque mode
static inline void SetJointTorque(uint8_t idx, float torque) {
    CHASSIS.joint_motor[idx].Set_Control_Angle(0.0f);
    CHASSIS.joint_motor[idx].Set_Control_Omega(0.0f);
    CHASSIS.joint_motor[idx].Set_K_P(0.0f);
    CHASSIS.joint_motor[idx].Set_K_D(0.0f);
    CHASSIS.joint_motor[idx].Set_Control_Torque(torque);
}

// Helper: set joint motor for velocity mode via MIT (kp=0, kd=kp_vel, torque=0)
static inline void SetJointVelocity(uint8_t idx, float velocity, float kp_vel) {
    CHASSIS.joint_motor[idx].Set_Control_Angle(0.0f);
    CHASSIS.joint_motor[idx].Set_Control_Omega(velocity);
    CHASSIS.joint_motor[idx].Set_K_P(0.0f);
    CHASSIS.joint_motor[idx].Set_K_D(kp_vel);
    CHASSIS.joint_motor[idx].Set_Control_Torque(0.0f);
}

// Helper: set joint motor for position mode via MIT
static inline void SetJointPosition(uint8_t idx, float position, float kp, float kd) {
    CHASSIS.joint_motor[idx].Set_Control_Angle(position);
    CHASSIS.joint_motor[idx].Set_Control_Omega(0.0f);
    CHASSIS.joint_motor[idx].Set_K_P(kp);
    CHASSIS.joint_motor[idx].Set_K_D(kd);
    CHASSIS.joint_motor[idx].Set_Control_Torque(0.0f);
}

static void ConsoleZeroForce(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        SetJointVelocity(i, 0.0f, 0.0f);
    }

    CHASSIS.wheel_motor[0].set.vel = 0;
    CHASSIS.wheel_motor[1].set.vel = 0;
    CHASSIS.wheel_motor[0].set.value = PID_Calc(&CHASSIS.pid.wheel_stop[0], CHASSIS.wheel_motor[0].fdb.vel, 0);
    CHASSIS.wheel_motor[1].set.value = PID_Calc(&CHASSIS.pid.wheel_stop[1], CHASSIS.wheel_motor[1].fdb.vel, 0);
}

static void ConsoleCalibrate(void)
{
    SetJointVelocity(0, -CALIBRATE_VELOCITY, CALIBRATE_VEL_KP);
    SetJointVelocity(1,  CALIBRATE_VELOCITY, CALIBRATE_VEL_KP);
    SetJointVelocity(2,  CALIBRATE_VELOCITY, CALIBRATE_VEL_KP);
    SetJointVelocity(3, -CALIBRATE_VELOCITY, CALIBRATE_VEL_KP);

    CHASSIS.wheel_motor[0].set.tor = 0;
    CHASSIS.wheel_motor[1].set.tor = 0;
}

static void ConsoleOffHook(void)
{
    SetJointVelocity(0, -CALIBRATE_VELOCITY, CALIBRATE_VEL_KP);
    SetJointVelocity(1,  CALIBRATE_VELOCITY, CALIBRATE_VEL_KP);
    SetJointVelocity(2,  CALIBRATE_VELOCITY, CALIBRATE_VEL_KP);
    SetJointVelocity(3, -CALIBRATE_VELOCITY, CALIBRATE_VEL_KP);

    CHASSIS.wheel_motor[0].set.tor = 0;
    CHASSIS.wheel_motor[1].set.tor = 0;
}

static void ConsoleNormal(void)
{
    LocomotionController();
    LegTorqueController();

    float j_tor[4];
    j_tor[0] = CHASSIS.cmd.leg[0].joint.T[0] * (J0_DIRECTION);
    j_tor[1] = CHASSIS.cmd.leg[0].joint.T[1] * (J1_DIRECTION);
    j_tor[2] = CHASSIS.cmd.leg[1].joint.T[0] * (J2_DIRECTION);
    j_tor[3] = CHASSIS.cmd.leg[1].joint.T[1] * (J3_DIRECTION);

    for (uint8_t i = 0; i < 4; i++) {
        if (CHASSIS.step == JUMP_STEP_JUMP) {
            j_tor[i] = fp32_constrain(j_tor[i], MIN_JOINT_TORQUE_JUMP, MAX_JOINT_TORQUE_JUMP);
        } else {
            j_tor[i] = fp32_constrain(j_tor[i], MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
        }
        SetJointTorque(i, j_tor[i]);
    }

    CHASSIS.wheel_motor[0].set.tor = -(CHASSIS.cmd.leg[0].wheel.T * (W0_DIRECTION));
    CHASSIS.wheel_motor[1].set.tor = -(CHASSIS.cmd.leg[1].wheel.T * (W1_DIRECTION));
}

static void ConsoleDebug(void)
{
    LocomotionController();
    LegTorqueController();

    float j_tor[4];
    j_tor[0] = CHASSIS.cmd.leg[0].joint.T[0] * (J0_DIRECTION);
    j_tor[1] = CHASSIS.cmd.leg[0].joint.T[1] * (J1_DIRECTION);
    j_tor[2] = CHASSIS.cmd.leg[1].joint.T[0] * (J2_DIRECTION);
    j_tor[3] = CHASSIS.cmd.leg[1].joint.T[1] * (J3_DIRECTION);

    for (uint8_t i = 0; i < 4; i++) {
        j_tor[i] = fp32_constrain(j_tor[i], MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
        SetJointTorque(i, j_tor[i]);
    }
}

static void ConsolePosDebug(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        SetJointTorque(i, 0.0f);
    }

    LocomotionController();

    float phi1_phi4_l[2], phi1_phi4_r[2];
    CalcPhi1AndPhi4(CHASSIS.ref.rod_Angle[0], CHASSIS.ref.rod_L0[0], phi1_phi4_l);
    CalcPhi1AndPhi4(CHASSIS.ref.rod_Angle[1], CHASSIS.ref.rod_L0[1], phi1_phi4_r);

    if (!(std::isnan(phi1_phi4_l[0]) || std::isnan(phi1_phi4_l[1]) ||
          std::isnan(phi1_phi4_r[0]) || std::isnan(phi1_phi4_r[1]))) {
        float pos[4];
        pos[0] = theta_transform(phi1_phi4_l[0], -J0_ANGLE_OFFSET, J0_DIRECTION, 1);
        pos[1] = theta_transform(phi1_phi4_l[1], -J1_ANGLE_OFFSET, J1_DIRECTION, 1);
        pos[2] = theta_transform(phi1_phi4_r[0], -J2_ANGLE_OFFSET, J2_DIRECTION, 1);
        pos[3] = theta_transform(phi1_phi4_r[1], -J3_ANGLE_OFFSET, J3_DIRECTION, 1);

        pos[0] = fp32_constrain(pos[0], MIN_J0_ANGLE, MAX_J0_ANGLE);
        pos[1] = fp32_constrain(pos[1], MIN_J1_ANGLE, MAX_J1_ANGLE);
        pos[2] = fp32_constrain(pos[2], MIN_J2_ANGLE, MAX_J2_ANGLE);
        pos[3] = fp32_constrain(pos[3], MIN_J3_ANGLE, MAX_J3_ANGLE);

#define DEBUG_KP_VAL 17.0f
#define DEBUG_KD_VAL 2.0f
        for (uint8_t i = 0; i < 4; i++) {
            SetJointPosition(i, pos[i], DEBUG_KP_VAL, DEBUG_KD_VAL);
        }
    }

    CHASSIS.wheel_motor[0].set.tor = -(CHASSIS.cmd.leg[0].wheel.T * (W0_DIRECTION));
    CHASSIS.wheel_motor[1].set.tor = -(CHASSIS.cmd.leg[1].wheel.T * (W1_DIRECTION));
}

static void ConsoleStandUp(void)
{
    double joint_pos_l[2] = {0.0, 0.0};
    double joint_pos_r[2] = {0.0, 0.0};

    if (!(std::isnan(joint_pos_l[0]) || std::isnan(joint_pos_l[1]) ||
          std::isnan(joint_pos_r[0]) || std::isnan(joint_pos_r[1]))) {
        float pos[4];
        pos[0] = theta_transform((float)joint_pos_l[1], -J0_ANGLE_OFFSET, J0_DIRECTION, 1);
        pos[1] = theta_transform((float)joint_pos_l[0], -J1_ANGLE_OFFSET, J1_DIRECTION, 1);
        pos[2] = theta_transform((float)joint_pos_r[1], -J2_ANGLE_OFFSET, J2_DIRECTION, 1);
        pos[3] = theta_transform((float)joint_pos_r[0], -J3_ANGLE_OFFSET, J3_DIRECTION, 1);

        pos[0] = fp32_constrain(pos[0], MIN_J0_ANGLE, MAX_J0_ANGLE);
        pos[1] = fp32_constrain(pos[1], MIN_J1_ANGLE, MAX_J1_ANGLE);
        pos[2] = fp32_constrain(pos[2], MIN_J2_ANGLE, MAX_J2_ANGLE);
        pos[3] = fp32_constrain(pos[3], MIN_J3_ANGLE, MAX_J3_ANGLE);

        for (uint8_t i = 0; i < 4; i++) {
            SetJointPosition(i, pos[i], NORMAL_POS_KP, NORMAL_POS_KD);
        }
    }

    float feedforward = -220;
    PID_Calc(&CHASSIS.pid.stand_up, CHASSIS.fdb.body.phi, 0);
    CHASSIS.wheel_motor[0].set.value = (feedforward + CHASSIS.pid.stand_up.Get_Out()) * W0_DIRECTION;
    CHASSIS.wheel_motor[1].set.value = (feedforward + CHASSIS.pid.stand_up.Get_Out()) * W1_DIRECTION;
}

/******************************************************************/
/* Cmd                                                            */
/******************************************************************/

#define DM_DELAY 250

static void SendJointMotorCmd(void);
static void SendWheelMotorCmd(void);

void ChassisSendCmd(void)
{
    SendJointMotorCmd();
    SendWheelMotorCmd();
}

static void SendJointMotorCmd(void)
{
    if (CHASSIS.mode == CHASSIS_OFF) {
        CHASSIS.joint_motor[0].CAN_Send_Exit();
        CHASSIS.joint_motor[1].CAN_Send_Exit();
        delay_us(DM_DELAY);
        CHASSIS.joint_motor[2].CAN_Send_Exit();
        CHASSIS.joint_motor[3].CAN_Send_Exit();
    } else {
        // 检查电机是否使能，如果没有则使能
        for (uint8_t i = 0; i < 4; i++) {
            if (CHASSIS.joint_motor[i].Get_Status() == Motor_DM_Status_DISABLE) {
                CHASSIS.joint_motor[i].CAN_Send_Enter();
                if (i % 2 == 1) delay_us(DM_DELAY);
            }
        }
        delay_us(DM_DELAY);

        switch (CHASSIS.mode) {
            case CHASSIS_CALIBRATE: {
                // velocity mode + check save zero
                for (uint8_t i = 0; i < 4; i++) {
                    CHASSIS.joint_motor[i].TIM_Send_PeriodElapsedCallback();
                    if (i % 2 == 1) delay_us(DM_DELAY);
                }
                if (CALIBRATE.reached[0] && CALIBRATE.reached[1] &&
                    CALIBRATE.reached[2] && CALIBRATE.reached[3]) {
                    delay_us(DM_DELAY);
                    CHASSIS.joint_motor[0].CAN_Send_Save_Zero();
                    CHASSIS.joint_motor[1].CAN_Send_Save_Zero();
                    delay_us(DM_DELAY);
                    CHASSIS.joint_motor[2].CAN_Send_Save_Zero();
                    CHASSIS.joint_motor[3].CAN_Send_Save_Zero();
                }
            } break;
            default: {
                // 所有其他模式: 发送控制帧 (MIT模式)
                CHASSIS.joint_motor[0].TIM_Send_PeriodElapsedCallback();
                CHASSIS.joint_motor[1].TIM_Send_PeriodElapsedCallback();
                delay_us(DM_DELAY);
                CHASSIS.joint_motor[2].TIM_Send_PeriodElapsedCallback();
                CHASSIS.joint_motor[3].TIM_Send_PeriodElapsedCallback();
            } break;
        }
    }
}

static void SendWheelMotorCmd(void)
{
    switch (CHASSIS.mode) {
        case CHASSIS_FOLLOW_GIMBAL_YAW:
        case CHASSIS_CUSTOM:
        case CHASSIS_FREE: {
            LkMultipleTorqueControl(WHEEL_CAN,
                CHASSIS.wheel_motor[0].set.tor, CHASSIS.wheel_motor[1].set.tor, 0, 0);
        } break;
        case CHASSIS_STAND_UP: {
            LkMultipleIqControl(WHEEL_CAN,
                CHASSIS.wheel_motor[0].set.value, CHASSIS.wheel_motor[1].set.value, 0, 0);
        } break;
        case CHASSIS_CALIBRATE:
        case CHASSIS_OFF: {
            LkMultipleTorqueControl(WHEEL_CAN, 0, 0, 0, 0);
        } break;
        case CHASSIS_POS_DEBUG: {
            LkMultipleTorqueControl(WHEEL_CAN,
                CHASSIS.wheel_motor[0].set.tor, CHASSIS.wheel_motor[1].set.tor, 0, 0);
        } break;
        case CHASSIS_SAFE:
        default: {
            LkMultipleTorqueControl(WHEEL_CAN,
                CHASSIS.wheel_motor[0].set.value, CHASSIS.wheel_motor[1].set.value, 0, 0);
        }
    }
}

/******************************************************************/
/* Public                                                         */
/******************************************************************/

void SetCali(const fp32 motor_middle[4]) { (void)motor_middle; }
bool_t CmdCali(fp32 motor_middle[4]) { (void)motor_middle; return 1; }
void ChassisSetCaliData(const fp32 motor_middle[4]) { SetCali(motor_middle); }
bool_t ChassisCmdCali(fp32 motor_middle[4]) { return CmdCali(motor_middle); }

uint8_t ChassisGetStatus(void) { return 0; }
uint32_t ChassisGetDuration(void) { return CHASSIS.duration; }
float ChassisGetSpeedVx(void) { return CHASSIS.fdb.speed_vector.vx; }
float ChassisGetSpeedVy(void) { return CHASSIS.fdb.speed_vector.vy; }
float ChassisGetSpeedWz(void) { return CHASSIS.fdb.speed_vector.wz; }

#endif /* CHASSIS_BALANCE */
/*------------------------------ End of File ------------------------------*/
