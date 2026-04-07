/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis.c/h
  * @brief      底盘模块对外开放的接口函数
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Sep-18-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef CHASSIS_H
#define CHASSIS_H

#include "robot_param.h"

#if CHASSIS_TYPE != CHASSIS_NONE

// inline void ChassisSetCali(void);
// inline void ChassisCmdCali(void);

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

typedef enum __ChassisState
{
    CHASSIS_STATE_NORNAL, // 底盘正常状态
    CHASSIS_STATE_ERROR   // 底盘错误状态
} ChassisState_e;

#ifdef __cplusplus
extern "C" {
#endif

uint8_t ChassisGetStatus(void);
uint32_t ChassisGetDuration(void);
float ChassisGetSpeedVx(void);
float ChassisGetSpeedVy(void);
float ChassisGetSpeedWz(void);

#ifdef __cplusplus
}
#endif

#endif // CHASSIS_TYPE
#endif // CHASSIS_H
/*------------------------------ End of File ------------------------------*/
