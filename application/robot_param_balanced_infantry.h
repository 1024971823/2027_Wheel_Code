/**
  * @file       robot_param_balanced_infantry.h
  * @brief      这里是平衡步兵机器人参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_TYPE CHASSIS_BALANCE             // 选择底盘类型

// clang-format off
#define __SELF_BOARD_ID C_BOARD_BALANCE_CHASSIS // 本板ID
#define __GYRO_BIAS_YAW  0.003096855f           // 陀螺仪零飘，单位rad/s

#define __CONTROL_LINK_RC  CL_RC_DIRECT  // 控制链路选择：RC遥控器
#define __CONTROL_LINK_KM  CL_KM_RC      // 控制链路选择：键鼠数据

#define __VIRTUAL_GIMBAL_FROM VG_FROM_UART2 // 虚拟云台数据来源（用于云台底盘分离控制）

/*-------------------- Chassis --------------------*/
// 底盘任务相关宏定义
#define CHASSIS_TASK_INIT_TIME 357   // 任务开始空闲一段时间
#define CHASSIS_CONTROL_TIME_MS 2    // 底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_S (CHASSIS_CONTROL_TIME_MS / 1000.0f)   // 底盘任务控制间隔

// 底盘的遥控器相关宏定义 ---------------------
#define CHASSIS_MODE_CHANNEL   0  // 选择底盘状态 开关通道号
#define CHASSIS_X_CHANNEL      3  // 前后的遥控器通道号码
#define CHASSIS_Y_CHANNEL      2  // 左右的遥控器通道号码
#define CHASSIS_WZ_CHANNEL     2  // 旋转的遥控器通道号码
#define CHASSIS_ANGLE_CHANNEL  4  // 腿摆角的遥控器通道号码
#define CHASSIS_LENGTH_CHANNEL 4  // 腿长的遥控器通道号码
#define CHASSIS_ROLL_CHANNEL   4  // ROLL角的遥控器通道号码
#define CHASSIS_RC_DEADLINE    20 // 摇杆死区

// deadzone parameters ---------------------
#define WHEEL_DEADZONE (0.01f)  // (m/s)轮子速度死区

// ratio parameters ---------------------
#define FF_RATIO               (0.25f)   // 前馈比例系数
#define ROLL_VEL_LIMIT_FACTOR  (0.1f)    // roll角速度抑制比例系数

// motor parameters ---------------------
#define JOINT_CAN (1)
#define WHEEL_CAN (2)

#define J0_DIRECTION (-1)
#define J1_DIRECTION (-1)
#define J2_DIRECTION (1)
#define J3_DIRECTION (1)

#define W0_DIRECTION ( 1)
#define W1_DIRECTION (-1)

// DM控制参数
#define CALIBRATE_VEL_KP  (4.0f)  // 校准MIT速度控制KP
#define DEBUG_VEL_KP      (4.0f)  // 调试MIT速度控制KP
#define ZERO_FORCE_VEL_KP (4.0f)  // 无力MIT速度控制KP

#define NORMAL_POS_KP (20.0f) // 正常MIT位置控制KP
#define NORMAL_POS_KD (1.0f)  // 正常MIT位置控制KD

#define DEBUG_POS_KP (8.0f) // 调试MIT位置控制KP
#define DEBUG_POS_KD (0.8f) // 调试MIT位置控制KD

//physical parameters ---------------------
#define LEG_L1 (0.215f)  // (m)腿1长度
#define LEG_L2 (0.258f)  // (m)腿2长度
#define LEG_L3 (LEG_L2)  // (m)腿3长度
#define LEG_L4 (LEG_L1)  // (m)腿4长度
#define LEG_L5 (0.0f)    // (m)关节间距

#define BODY_MASS            (8.5f)      // (kg)机身重量
#define WHEEL_MASS           (0.65f)      // (kg)轮子重量
#define WHEEL_RADIUS         (0.0625f)    // (m)轮子半径
#define WHEEL_START_TORQUE   (0.3f)      // (Nm)轮子起动力矩
#define WHEEL_BASE           (0.51175f)  // (m)驱动轮轴距

#define J0_ANGLE_OFFSET     (-0.19163715f + M_PI) // (rad)关节0角度偏移量(电机0点到水平线的夹角)
#define J1_ANGLE_OFFSET     (0.19163715f)         // (rad)关节1角度偏移量(电机0点到水平线的夹角)
#define J2_ANGLE_OFFSET     (0.19163715f + M_PI)  // (rad)关节2角度偏移量(电机0点到水平线的夹角)
#define J3_ANGLE_OFFSET     (-0.19163715f)        // (rad)关节3角度偏移量(电机0点到水平线的夹角)

#define DLENGTH_DIRECTION  (-1) // ROLL角补偿量方向(腿长增加方向)

//upper_limit parameters ---------------------

#define MAX_DELTA_ROD_ANGLE (0.25f) // (rad)腿摆角最大变化量
#define MAX_TORQUE_PROTECT  (25.0f)  // (Nm)最大扭矩保护

#define MAX_DELTA_VEL_FDB_TO_REF (0.8f) // (m/s)速度反馈到参考速度的最大变化量

#define MAX_THETA      (1.0f)
#define MAX_THETA_DOT  (2.0f)
#define MAX_X          (1.0f)
#define MAX_X_DOT      (10.0f)
#define MAX_PHI        (1.0f)
#define MAX_PHI_DOT    (2.0f)

#define MAX_SPEED_INTEGRAL  (0.5f)
#define MAX_ROLL            (0.3f)
#define MAX_ROLL_VELOCITY   (1.0f)
#define MAX_YAW             (M_PI)
#define MAX_YAW_VELOCITY    (3.0f)

#define MAX_J0_ANGLE  (1.8f) // (rad)关节角度上限
#define MAX_J1_ANGLE  (0.0f) // (rad)关节角度上限
#define MAX_J2_ANGLE  (0.6f) // (rad)关节角度上限
#define MAX_J3_ANGLE  (1.8f) // (rad)关节角度上限

#define MAX_LEG_LENGTH       (0.35f)
#define MAX_LEG_ANGLE        (M_PI_2 + MAX_DELTA_ROD_ANGLE)
#define MAX_SPEED            (3.5f)
#define MAX_SPEED_VECTOR_VX  (3.5f)
#define MAX_SPEED_VECTOR_VY  (3.5f)
#define MAX_SPEED_VECTOR_WZ  (6.0f)

#define MAX_JOINT_TORQUE      (7.0f)   // (Nm)关节最大扭矩
#define MAX_JOINT_TORQUE_JUMP (20.0f)  // (Nm)跳跃时的关节最大扭矩
#define MAX_VEL_ADD           (1.0f)   // (m/s)速度增量上限
#define MAX_PITCH_VEL         (0.1f)   // (rad/s)pitch轴速度上限

#define MAX_TOUCH_INTERVAL    (200)    // (ms)最大离地时间，超过这个时间认为离地

//lower_limit parameters ---------------------

#define MIN_DELTA_ROD_ANGLE (-MAX_DELTA_ROD_ANGLE) // (rad)腿摆角最小变化量

#define MIN_DELTA_VEL_FDB_TO_REF (-MAX_DELTA_VEL_FDB_TO_REF) // (m/s)速度反馈到参考速度的最小变化量

#define MIN_THETA      (-MAX_THETA)
#define MIN_THETA_DOT  (-MAX_THETA_DOT)
#define MIN_X          (-MAX_X)
#define MIN_X_DOT      (-MAX_X_DOT)
#define MIN_PHI        (-MAX_PHI)
#define MIN_PHI_DOT    (-MAX_PHI_DOT)

#define MIN_SPEED_INTEGRAL  (-MAX_SPEED_INTEGRAL)
#define MIN_ROLL            (-MAX_ROLL)
#define MIN_ROLL_VELOCITY   (-MAX_ROLL_VELOCITY)
#define MIN_YAW             (-MAX_YAW)
#define MIN_YAW_VELOCITY    (-MAX_YAW_VELOCITY)

#define MIN_J0_ANGLE (-0.6f) // (rad)关节角度下限
#define MIN_J1_ANGLE (-1.8f) // (rad)关节角度下限
#define MIN_J2_ANGLE (-1.8f) // (rad)关节角度下限
#define MIN_J3_ANGLE ( 0.0f) // (rad)关节角度下限

#define MIN_LEG_LENGTH       ( 0.11f)
#define MIN_LEG_ANGLE        ( M_PI_2 - MAX_DELTA_ROD_ANGLE)
#define MIN_SPEED            (-MAX_SPEED)
#define MIN_SPEED_VECTOR_VX  (-MAX_SPEED_VECTOR_VX)
#define MIN_SPEED_VECTOR_VY  (-MAX_SPEED_VECTOR_VY)
#define MIN_SPEED_VECTOR_WZ  (-MAX_SPEED_VECTOR_WZ)

#define MIN_JOINT_TORQUE      (-MAX_JOINT_TORQUE)  // 
#define MIN_JOINT_TORQUE_JUMP (-MAX_JOINT_TORQUE_JUMP)  // 
#define MIN_VEL_ADD           (-MAX_VEL_ADD)    // (m/s)速度增量下限
#define MIN_PITCH_VEL         (-MAX_PITCH_VEL)  // (rad/s)pitch轴速度下限

//PID parameters ---------------------
//yaw轴跟踪角度环PID参数
#define KP_CHASSIS_YAW_ANGLE        (2.3f)
#define KI_CHASSIS_YAW_ANGLE        (1.0f)
#define KD_CHASSIS_YAW_ANGLE        (0.0f)
#define MAX_IOUT_CHASSIS_YAW_ANGLE  (0.5f)
#define MAX_OUT_CHASSIS_YAW_ANGLE   (5.0f)

//yaw轴跟踪速度环PID参数
#define KP_CHASSIS_YAW_VELOCITY        (2.2f)
#define KI_CHASSIS_YAW_VELOCITY        (0.5f)
#define KD_CHASSIS_YAW_VELOCITY        (0.0f)
#define MAX_IOUT_CHASSIS_YAW_VELOCITY  (0.5f)
#define MAX_OUT_CHASSIS_YAW_VELOCITY   (2.0f)

// vel_add PID参数
#define KP_CHASSIS_VEL_ADD        (0.1f) //0.1
#define KI_CHASSIS_VEL_ADD        (0.005f)//0.005
#define KD_CHASSIS_VEL_ADD        (0.001f)//0.001
#define MAX_IOUT_CHASSIS_VEL_ADD  (0.8f)//0.5
#define MAX_OUT_CHASSIS_VEL_ADD   (1.5f)//1.0

/*========== Start of locomotion control pid ==========*/

//roll轴跟踪角度环PID参数
#define KP_CHASSIS_ROLL_ANGLE        (0.3f)
#define KI_CHASSIS_ROLL_ANGLE        (0.0f)
#define KD_CHASSIS_ROLL_ANGLE        (0.2f)
#define MAX_IOUT_CHASSIS_ROLL_ANGLE  (0.0f)
#define MAX_OUT_CHASSIS_ROLL_ANGLE   (0.1f)
#define N_CHASSIS_ROLL_ANGLE         (0.1f)

//roll轴跟踪速度环PID参数
// #define KP_CHASSIS_ROLL_VELOCITY        (0.6f)
// #define KI_CHASSIS_ROLL_VELOCITY        (0.0001f)
// #define KD_CHASSIS_ROLL_VELOCITY        (0.005f)
// #define MAX_IOUT_CHASSIS_ROLL_VELOCITY  (0.01f)
// #define MAX_OUT_CHASSIS_ROLL_VELOCITY   (0.1f)

// 腿长跟踪长度环PID参数
#define KP_CHASSIS_LEG_LENGTH_LENGTH        (150.0f)
#define KI_CHASSIS_LEG_LENGTH_LENGTH        (0.0f)
#define KD_CHASSIS_LEG_LENGTH_LENGTH        (1500.0f)
#define MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH  (0.0f)
#define MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH   (40.0f)
#define N_LEG_LENGTH_LENGTH                 (0.1f)

// 腿长跟踪速度环PID参数
// #define KP_CHASSIS_LEG_LENGTH_SPEED 0.0f
// #define KI_CHASSIS_LEG_LENGTH_SPEED 0.0f
// #define KD_CHASSIS_LEG_LENGTH_SPEED 0.0f
// #define MAX_IOUT_CHASSIS_LEG_LENGTH_SPEED 0.0f
// #define MAX_OUT_CHASSIS_LEG_LENGTH_SPEED 0.0f

/*========== End of locomotion control pid ==========*/

// 起立用的pid
#define KP_CHASSIS_STAND_UP       (2000.0f)
#define KI_CHASSIS_STAND_UP       (0.0f)
#define KD_CHASSIS_STAND_UP       (10.0f)
#define MAX_IOUT_CHASSIS_STAND_UP (0.0f)
#define MAX_OUT_CHASSIS_STAND_UP  (2000.0f)

// 轮子停止用的pid
#define KP_CHASSIS_WHEEL_STOP       (0.3f)
#define KI_CHASSIS_WHEEL_STOP       (0.0f)
#define KD_CHASSIS_WHEEL_STOP       (0.1f)
#define MAX_IOUT_CHASSIS_WHEEL_STOP (0.0f)
#define MAX_OUT_CHASSIS_WHEEL_STOP  (200.0f)

// 云台跟随用的pid
#define KP_CHASSIS_FOLLOW_GIMBAL       (5.0f)
#define KI_CHASSIS_FOLLOW_GIMBAL       (0.0f)
#define KD_CHASSIS_FOLLOW_GIMBAL       (0.0f)
#define MAX_IOUT_CHASSIS_FOLLOW_GIMBAL (0.0f)
#define MAX_OUT_CHASSIS_FOLLOW_GIMBAL  (6.0f)

//LPF parameters ---------------------
#define LEG_DDL0_LPF_ALPHA           (0.9f)
#define LEG_DDPHI0_LPF_ALPHA         (0.9f)
#define LEG_DDTHETA_LPF_ALPHA        (0.9f)
#define LEG_SUPPORT_FORCE_LPF_ALPHA  (0.9f)
#define CHASSIS_ROLL_ALPHA           (0.5f)

//offest parameters ---------------------

#define X0_OFFSET (0.0f)   // 目标theta偏移量
#define X1_OFFSET (0.0f)   // 目标theta_dot偏移量
#define X2_OFFSET (-0.09f) // 目标x偏移量
#define X3_OFFSET (0.0f)   // 目标x_dot偏移量
#define X4_OFFSET (0.0f)   // 目标phi偏移量
#define X5_OFFSET (0.0f)   // 目标phi_dot偏移量

//other parameters ---------------------

// clang-format on
#endif /* INCLUDED_ROBOT_PARAM_H */
