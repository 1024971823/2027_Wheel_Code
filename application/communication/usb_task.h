#ifndef USB_TASK_H
#define USB_TASK_H

#include "robot_param.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void usb_task(void const * argument);

// 上位机控制指令接口 (原supervisory_computer_cmd.h)
float GetScCmdGimbalAngle(uint8_t axis);
float GetScCmdChassisSpeed(uint8_t axis);
float GetScCmdChassisVelocity(uint8_t axis);
float GetScCmdChassisHeight(void);
bool GetScCmdFire(void);
bool GetScCmdFricOn(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_TASK_H */
