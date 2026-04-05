/**
 * @file app_chassis.cpp
 * @brief 底盘应用层
 */

#include "app_chassis.h"

#include "dvc_motor_stw.h"
#include "dvc_vofa.h"
#include "i6x.h"

namespace
{
enum Enum_Vofa_STW_Channel
{
    Vofa_STW_Channel_Target_Omega = 0,
    Vofa_STW_Channel_Now_Omega,
    Vofa_STW_Channel_Control_Torque,
    Vofa_STW_Channel_Now_Torque,
    Vofa_STW_Channel_Now_Angle,
    Vofa_STW_Channel_Status,
    Vofa_STW_Channel_Num,
};

Class_Motor_STW chassis_motor[4];
Class_Vofa_USB *chassis_vofa_usb = nullptr;
float vofa_stw_monitor_data[4][Vofa_STW_Channel_Num] = {0.0f};

void Update_Vofa_STW_Monitor_Data(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (int i = 0; i < 4; i++)
    {
        vofa_stw_monitor_data[i][Vofa_STW_Channel_Target_Omega] = chassis_motor[i].Get_Target_Omega();
        vofa_stw_monitor_data[i][Vofa_STW_Channel_Now_Omega] = chassis_motor[i].Get_Now_Omega();
        vofa_stw_monitor_data[i][Vofa_STW_Channel_Control_Torque] = chassis_motor[i].Get_Control_Torque();
        vofa_stw_monitor_data[i][Vofa_STW_Channel_Now_Torque] = chassis_motor[i].Get_Now_Torque();
        vofa_stw_monitor_data[i][Vofa_STW_Channel_Now_Angle] = chassis_motor[i].Get_Now_Angle();
        vofa_stw_monitor_data[i][Vofa_STW_Channel_Status] = static_cast<float>(chassis_motor[i].Get_Status());
    }
    __set_PRIMASK(primask);
}
}

void App_Chassis_Init(const FDCAN_HandleTypeDef *hcan, Class_Vofa_USB *vofa_usb)
{
    const uint8_t motor_ids[4] = {0x01, 0x02, 0x03, 0x04};

    chassis_vofa_usb = vofa_usb;
    for (int i = 0; i < 4; i++)
    {
        chassis_motor[i].PID_Omega.Init(0.5f, 0.0f, 0.0f, 0.0f, 5.0f, 18.0f);
        chassis_motor[i].Init(hcan, motor_ids[i], Motor_STW_Control_Method_OMEGA, 95.5f, 45.0f, 18.0f);
        chassis_motor[i].CAN_Send_Enter();
    }
}

void App_Chassis_CAN_RxCpltCallback(void)
{
    for (int i = 0; i < 4; i++)
    {
        chassis_motor[i].CAN_RxCpltCallback();
    }
}

void App_Chassis_Ctrl_Task_Loop(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    i6x_ctrl_t rc = *get_i6x_point();
    __set_PRIMASK(primask);

    if (rc.failsafe || rc.frame_lost)
    {
        for (int i = 0; i < 4; i++)
        {
            chassis_motor[i].Set_Target_Omega(0.0f);
            chassis_motor[i].TIM_Calculate_PeriodElapsedCallback();
        }
        return;
    }

    const int16_t deadband = 20;
    int16_t ch_speed = (rc.ch[1] > deadband || rc.ch[1] < -deadband) ? rc.ch[1] : 0;
    int16_t ch_turn = (rc.ch[3] > deadband || rc.ch[3] < -deadband) ? rc.ch[3] : 0;

    const float max_omega = STW_V_MAX;
    float speed = static_cast<float>(ch_speed) / 660.0f * max_omega;
    float turn = static_cast<float>(ch_turn) / 660.0f * max_omega;

    float left_omega = speed + turn;
    float right_omega = speed - turn;

    if (left_omega > max_omega)
    {
        left_omega = max_omega;
    }
    if (left_omega < -max_omega)
    {
        left_omega = -max_omega;
    }
    if (right_omega > max_omega)
    {
        right_omega = max_omega;
    }
    if (right_omega < -max_omega)
    {
        right_omega = -max_omega;
    }

    chassis_motor[0].Set_Target_Omega(left_omega);
    chassis_motor[1].Set_Target_Omega(left_omega);
    chassis_motor[2].Set_Target_Omega(-right_omega);
    chassis_motor[3].Set_Target_Omega(-right_omega);

    for (int i = 0; i < 4; i++)
    {
        chassis_motor[i].TIM_Calculate_PeriodElapsedCallback();
    }

    static uint32_t alive_counter = 0U;
    if (++alive_counter >= 100U)
    {
        alive_counter = 0U;
        for (int i = 0; i < 4; i++)
        {
            chassis_motor[i].TIM_100ms_Alive_PeriodElapsedCallback();
        }
    }
}

void App_Chassis_Monitor_Task_Loop(void)
{
    if (chassis_vofa_usb == nullptr)
    {
        return;
    }

    Update_Vofa_STW_Monitor_Data();

    chassis_vofa_usb->Set_Data(24,
                               &vofa_stw_monitor_data[0][0], &vofa_stw_monitor_data[0][1], &vofa_stw_monitor_data[0][2],
                               &vofa_stw_monitor_data[0][3], &vofa_stw_monitor_data[0][4], &vofa_stw_monitor_data[0][5],
                               &vofa_stw_monitor_data[1][0], &vofa_stw_monitor_data[1][1], &vofa_stw_monitor_data[1][2],
                               &vofa_stw_monitor_data[1][3], &vofa_stw_monitor_data[1][4], &vofa_stw_monitor_data[1][5],
                               &vofa_stw_monitor_data[2][0], &vofa_stw_monitor_data[2][1], &vofa_stw_monitor_data[2][2],
                               &vofa_stw_monitor_data[2][3], &vofa_stw_monitor_data[2][4], &vofa_stw_monitor_data[2][5],
                               &vofa_stw_monitor_data[3][0], &vofa_stw_monitor_data[3][1], &vofa_stw_monitor_data[3][2],
                               &vofa_stw_monitor_data[3][3], &vofa_stw_monitor_data[3][4], &vofa_stw_monitor_data[3][5]);
    chassis_vofa_usb->TIM_1ms_Write_PeriodElapsedCallback();
}
