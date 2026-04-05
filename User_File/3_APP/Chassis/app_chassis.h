/**
 * @file app_chassis.h
 * @brief 底盘应用层
 */

#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "drv_can.h"

class Class_Vofa_USB;

void App_Chassis_Init(const FDCAN_HandleTypeDef *hcan, Class_Vofa_USB *vofa_usb);

void App_Chassis_CAN_RxCpltCallback(void);

void App_Chassis_Ctrl_Task_Loop(void);

void App_Chassis_Monitor_Task_Loop(void);

#endif
