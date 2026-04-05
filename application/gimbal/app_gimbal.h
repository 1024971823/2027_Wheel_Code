/**
 * @file app_gimbal.h
 * @brief 云台应用层
 */

#ifndef APP_GIMBAL_H
#define APP_GIMBAL_H

#include "drv_can.h"

void App_Gimbal_Init(const FDCAN_HandleTypeDef *hcan);

void App_Gimbal_CAN_RxCpltCallback(void);

void App_Gimbal_Task_1ms_Callback(void);

void App_Gimbal_Vofa_Set_Debug_Variable(int32_t index, float value);

#endif
