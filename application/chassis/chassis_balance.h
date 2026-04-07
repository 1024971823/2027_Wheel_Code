/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_balance.h
  * @brief      平衡底盘控制器 (重构适配 DaMiao MC02 H7 平台)
  * @history
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *  V2.0.0     2025            Refactor        适配H7新中间件
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/
#ifndef CHASSIS_BALANCE_H
#define CHASSIS_BALANCE_H

#include "robot_param.h"

#if (CHASSIS_TYPE == CHASSIS_BALANCE)

#ifdef __cplusplus

#include "IMU_task.h"
#include "custom_typedef.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "alg_pid.h"
#include "alg_filter_kalman.h"
#include "dvc_motor_dm.h"
#include <math.h>
#include <string.h>

// clang-format off
#define JOINT_ERROR_OFFSET   ((uint8_t)1 << 0)
#define WHEEL_ERROR_OFFSET   ((uint8_t)1 << 1)
#define DBUS_ERROR_OFFSET    ((uint8_t)1 << 2)
#define IMU_ERROR_OFFSET     ((uint8_t)1 << 3)
#define FLOATING_OFFSET      ((uint8_t)1 << 4)
// clang-format on

/*-------------------- Simple Low-Pass Filter --------------------*/

typedef struct {
    float alpha;
    float out;
    bool  initialized;
} LowPassFilter_t;

static inline void LowPassFilterInit(LowPassFilter_t *lpf, float alpha) {
    lpf->alpha = alpha;
    lpf->out = 0.0f;
    lpf->initialized = false;
}

static inline float LowPassFilterCalc(LowPassFilter_t *lpf, float input) {
    if (!lpf->initialized) {
        lpf->out = input;
        lpf->initialized = true;
    } else {
        lpf->out = lpf->alpha * lpf->out + (1.0f - lpf->alpha) * input;
    }
    return lpf->out;
}

/*-------------------- Utility functions --------------------*/

static inline float theta_transform(float pos, float offset, int8_t direction, int8_t mode) {
    (void)mode;
    return pos * direction + offset;
}

static inline float fp32_constrain(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/*-------------------- Structural definition --------------------*/

typedef enum {
    CHASSIS_OFF,
    CHASSIS_SAFE,
    CHASSIS_STAND_UP,
    CHASSIS_CALIBRATE,
    CHASSIS_FOLLOW_GIMBAL_YAW,
    CHASSIS_FLOATING,
    CHASSIS_CRASHING,
    CHASSIS_FREE,
    CHASSIS_AUTO,
    CHASSIS_OFF_HOOK,
    CHASSIS_DEBUG,
    CHASSIS_POS_DEBUG,
    CHASSIS_CUSTOM
} ChassisMode_e;

typedef struct Leg
{
    struct rod
    {
        float Phi0;
        float dPhi0;
        float ddPhi0;

        float L0;
        float dL0;
        float ddL0;

        float Theta;
        float dTheta;
        float ddTheta;

        float F;
        float Tp;
    } rod;

    struct joint
    {
        float T1, T2;
        float Phi1, Phi4;
        float dPhi1, dPhi4;
    } joint;

    struct wheel
    {
        float Angle;
        float Velocity;
    } wheel;

    float J[2][2];
    float Fn;
    uint32_t take_off_time;
    uint32_t touch_time;
    bool is_take_off;
} Leg_t;

typedef struct Body
{
    float x;
    float x_dot;
    float x_dot_obv;
    float x_acc;
    float x_acc_obv;

    float x_accel;
    float y_accel;
    float z_accel;

    float gx, gy, gz;

    float phi;
    float phi_dot;

    float roll;
    float roll_dot;
    float pitch;
    float pitch_dot;
    float yaw;
    float yaw_dot;
} Body_t;

typedef struct
{
    float x_accel;
    float y_accel;
    float z_accel;
} World_t;

typedef struct LegState
{
    float theta;
    float theta_dot;
    float x;
    float x_dot;
    float phi;
    float phi_dot;
} LegState_t;

typedef struct
{
    Body_t body;
    World_t world;
    Leg_t leg[2];
    LegState_t leg_state[2];
    ChassisSpeedVector_t speed_vector;
} Fdb_t;

typedef struct
{
    Body_t body;
    LegState_t leg_state[2];
    float rod_L0[2];
    float rod_Angle[2];
    ChassisSpeedVector_t speed_vector;
} Ref_t;

typedef struct Cmd
{
    struct leg
    {
        struct rod_cmd
        {
            float F;
            float Tp;
        } rod;
        struct joint_cmd
        {
            float T[2];
            float Pos[2];
        } joint;
        struct wheel_cmd
        {
            float T;
        } wheel;
    } leg[2];
} Cmd_t;

typedef struct
{
    Class_PID yaw_angle;
    Class_PID yaw_velocity;
    Class_PID vel_add;
    Class_PID roll_angle;
    Class_PID pitch_angle;
    Class_PID leg_length_length[2];
    Class_PID leg_length_speed[2];
    Class_PID leg_angle_angle;
    Class_PID stand_up;
    Class_PID wheel_stop[2];
    Class_PID chassis_follow_gimbal;
} PID_t;

typedef struct LPF
{
    LowPassFilter_t leg_l0_accel_filter[2];
    LowPassFilter_t leg_phi0_accel_filter[2];
    LowPassFilter_t leg_theta_accel_filter[2];
    LowPassFilter_t support_force_filter[2];
    LowPassFilter_t roll;
} LPF_t;

/*-------------------- Wheel motor feedback (LK/MF9025) --------------------*/

typedef struct
{
    float pos;
    float vel;
    float tor;
    float value;
} WheelMotorData_t;

typedef struct
{
    WheelMotorData_t fdb;
    WheelMotorData_t set;
} WheelMotor_t;

/*-------------------- Main chassis struct --------------------*/

typedef struct
{
    const RC_ctrl_t *rc;
    const Imu_t *imu;
    ChassisMode_e mode;
    uint8_t error_code;
    int8_t step;
    uint32_t step_time;

    Class_Motor_DM_Normal joint_motor[4];
    WheelMotor_t wheel_motor[2];

    Ref_t ref;
    Fdb_t fdb;
    Cmd_t cmd;

    PID_t pid;
    LPF_t lpf;

    uint32_t last_time;
    uint32_t duration;
    float dyaw;
    uint16_t yaw_mid;
} Chassis_s;

typedef struct Calibrate
{
    uint32_t cali_cnt;
    float velocity[4];
    uint32_t stpo_time[4];
    bool reached[4];
    bool calibrated;
    bool toggle;
} Calibrate_s;

typedef struct GroundTouch
{
    uint32_t touch_time;
    float force[2];
    float support_force[2];
    bool touch;
} GroundTouch_s;

typedef struct
{
    struct
    {
        Class_Filter_Kalman<2, 1, 2> v_kf;
    } body;
} Observer_t;

#ifdef __cplusplus
extern "C" {
#endif

extern void ChassisInit(void);
extern void ChassisHandleException(void);
extern void ChassisSetMode(void);
extern void ChassisObserver(void);
extern void ChassisReference(void);
extern void ChassisConsole(void);
extern void ChassisSendCmd(void);

extern void SetCali(const fp32 motor_middle[4]);
extern bool_t CmdCali(fp32 motor_middle[4]);
extern void ChassisSetCaliData(const fp32 motor_middle[4]);
extern bool_t ChassisCmdCali(fp32 motor_middle[4]);

#ifdef __cplusplus
}
#endif

#endif /* __cplusplus */

#endif /* CHASSIS_BALANCE */
#endif /* CHASSIS_BALANCE_H */
/*------------------------------ End of File ------------------------------*/
