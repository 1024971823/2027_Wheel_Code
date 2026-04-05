/**
 * @file app_gimbal.cpp
 * @brief 云台应用层
 */

#include "app_gimbal.h"

#include "alg_basic.h"
#include "alg_filter_kalman.h"
#include "alg_matrix.h"
#include "dvc_motor_dji.h"

static Class_Motor_DJI_GM6020 gimbal_yaw_motor;
static Class_Filter_Kalman gimbal_filter_kalman;

static Class_Matrix_f32<2, 2> matrix_a;
static Class_Matrix_f32<2, 1> matrix_b;
static Class_Matrix_f32<2, 2> matrix_h;
static Class_Matrix_f32<2, 2> matrix_q;
static Class_Matrix_f32<2, 2> matrix_r;
static Class_Matrix_f32<2, 2> matrix_p;

void App_Gimbal_Init(const FDCAN_HandleTypeDef *hcan)
{
    gimbal_yaw_motor.PID_Angle.Init(12.0f, 0.0f, 0.0f, 0.0f, 10.0f, 10.0f);
    gimbal_yaw_motor.PID_Omega.Init(0.03f, 5.0f, 0.0f, 0.0f, 0.2f, 0.2f);
    gimbal_yaw_motor.Init(hcan, Motor_DJI_ID_0x206, Motor_DJI_Control_Method_ANGLE, 0, PI / 6);

    matrix_a[0][0] = 1.0f;
    matrix_a[0][1] = 0.001f;
    matrix_a[1][0] = 0.0f;
    matrix_a[1][1] = 1.0f;

    matrix_b[0][0] = 0.0f;
    matrix_b[1][0] = 0.0f;

    matrix_h[0][0] = 1.0f;
    matrix_h[0][1] = 0.0f;
    matrix_h[1][0] = 0.0f;
    matrix_h[1][1] = 1.0f;

    matrix_q[0][0] = 0.001f;
    matrix_q[0][1] = 0.0f;
    matrix_q[1][0] = 0.0f;
    matrix_q[1][1] = 0.1f;

    matrix_r[0][0] = 0.001f;
    matrix_r[0][1] = 0.0f;
    matrix_r[1][0] = 0.0f;
    matrix_r[1][1] = 1.0f;

    matrix_p[0][0] = 1.0f;
    matrix_p[0][1] = 0.0f;
    matrix_p[1][0] = 0.0f;
    matrix_p[1][1] = 1.0f;

    gimbal_filter_kalman.Init(matrix_a, matrix_b, matrix_h, matrix_q, matrix_r, matrix_p);
}

void App_Gimbal_CAN_RxCpltCallback(void)
{
    gimbal_yaw_motor.CAN_RxCpltCallback();
}

void App_Gimbal_Task_1ms_Callback(void)
{
    static uint32_t alive_counter = 0U;

    if (++alive_counter >= 100U)
    {
        alive_counter = 0U;
        gimbal_yaw_motor.TIM_100ms_Alive_PeriodElapsedCallback();
    }

    gimbal_yaw_motor.Set_Target_Angle(1.0f * PI);
    gimbal_yaw_motor.TIM_Calculate_PeriodElapsedCallback();

    gimbal_filter_kalman.Vector_Z[0][0] = gimbal_yaw_motor.Get_Now_Angle();
    gimbal_filter_kalman.Vector_Z[1][0] = gimbal_yaw_motor.Get_Now_Omega();
    gimbal_filter_kalman.TIM_Predict_PeriodElapsedCallback();
    gimbal_filter_kalman.TIM_Update_PeriodElapsedCallback();
}

void App_Gimbal_Vofa_Set_Debug_Variable(int32_t index, float value)
{
    switch (index)
    {
    case 0:
        gimbal_filter_kalman.Matrix_Q[0][0] = value;
        break;
    case 1:
        gimbal_filter_kalman.Matrix_Q[1][1] = value;
        break;
    case 2:
        gimbal_filter_kalman.Matrix_R[0][0] = value;
        break;
    case 3:
        gimbal_filter_kalman.Matrix_R[1][1] = value;
        break;
    default:
        break;
    }
}
