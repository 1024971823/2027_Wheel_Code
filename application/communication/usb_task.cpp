/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       usb_task.cpp
  * @brief      通过USB串口与上位机通信，适配达妙MC02开发板(STM32H723)
  *             使用drv_usb中间件替代原F407 USB CDC直接操作
  * @note       达妙板USB通信通过drv_usb.h的USB_Transmit_Data()发送
  *             接收通过回调函数机制实现
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-24-2024     Penguin         1. done
  *  V2.0.0     Apr-06-2026     Copilot         1. 适配达妙MC02(H7)
  *                                             2. 使用drv_usb中间件
  *                                             3. 移除F407 USB依赖
  *                                             4. 新增CRC8_CRC16本地实现
  *                                             5. 移除不存在的supervisory_computer_cmd.h
  *
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/

#include "usb_task.h"

#include <stdbool.h>
#include <string.h>

#include "CRC8_CRC16.h"
#include "cmsis_os.h"
#include "data_exchange.h"
#include "drv_usb.h"
#include "macro_typedef.h"
#include "usb_debug.h"
#include "usb_typdef.h"
#include "gimbal.h"
#include "IMU.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t usb_high_water;
#endif

#define USB_TASK_CONTROL_TIME 1  // ms

#define USB_OFFLINE_THRESHOLD 100  // ms
#define USB_CONNECT_CNT 10

// clang-format off

#define SEND_DURATION_Debug       5   // ms
#define SEND_DURATION_Imu         5   // ms
#define SEND_DURATION_RobotStateInfo   10  // ms
#define SEND_DURATION_Event       10  // ms
#define SEND_DURATION_Pid         10  // ms
#define SEND_DURATION_AllRobotHp  10  // ms
#define SEND_DURATION_GameStatus  10  // ms
#define SEND_DURATION_RobotMotion 10  // ms
#define SEND_DURATION_GroundRobotPosition   10// ms
#define SEND_DURATION_RfidStatus   10// ms
#define SEND_DURATION_RobotStatus  10// ms
#define SEND_DURATION_JointState   10// ms
#define SEND_DURATION_Buff         10// ms

// clang-format on

#define USB_RX_DATA_SIZE 256  // byte
#define USB_RECEIVE_LEN 150   // byte
#define HEADER_SIZE 4         // byte

#define CheckDurationAndSend(send_name)                                                  \
    do {                                                                                 \
        if ((HAL_GetTick() - LAST_SEND_TIME.send_name) >= SEND_DURATION_##send_name) {   \
            LAST_SEND_TIME.send_name = HAL_GetTick();                                    \
            UsbSend##send_name##Data();                                                  \
        }                                                                                \
    } while (0)

// Variable Declarations
static uint8_t USB_RX_BUF[USB_RX_DATA_SIZE];
static volatile uint16_t USB_RX_LEN = 0;
static volatile bool USB_RX_FLAG = false;

static const Imu_t *IMU;
static const ChassisSpeedVector_t *FDB_SPEED_VECTOR;

// 判断USB连接状态用到的一些变量
static bool USB_OFFLINE = true;
static uint32_t RECEIVE_TIME = 0;
static uint32_t LATEST_RX_TIMESTAMP = 0;
static uint32_t CONTINUE_RECEIVE_CNT = 0;

// 数据发送结构体
// clang-format off
static SendDataDebug_s       SEND_DATA_DEBUG;
static SendDataImu_s         SEND_DATA_IMU;
static SendDataRobotStateInfo_s   SEND_DATA_ROBOT_STATE_INFO;
static SendDataEvent_s       SEND_DATA_EVENT;
static SendDataPidDebug_s    SEND_DATA_PID;
static SendDataAllRobotHp_s  SEND_DATA_ALL_ROBOT_HP;
static SendDataGameStatus_s  SEND_DATA_GAME_STATUS;
static SendDataRobotMotion_s SEND_ROBOT_MOTION_DATA;
static SendDataGroundRobotPosition_s SEND_GROUND_ROBOT_POSITION_DATA;
static SendDataRfidStatus_s  SEND_RFID_STATUS_DATA;
static SendDataRobotStatus_s SEND_ROBOT_STATUS_DATA;
static SendDataJointState_s  SEND_JOINT_STATE_DATA;
static SendDataBuff_s        SEND_BUFF_DATA;

// clang-format on

// 数据接收结构体
static ReceiveDataRobotCmd_s RECEIVE_ROBOT_CMD_DATA;
static ReceiveDataPidDebug_s RECEIVE_PID_DEBUG_DATA;
static ReceiveDataVirtualRc_s RECEIVE_VIRTUAL_RC_DATA;

// 机器人控制指令数据
RobotCmdData_t ROBOT_CMD_DATA;
static RC_ctrl_t VIRTUAL_RC_CTRL;

// 发送数据间隔时间
typedef struct
{
    uint32_t Debug;
    uint32_t Imu;
    uint32_t RobotStateInfo;
    uint32_t Event;
    uint32_t Pid;
    uint32_t AllRobotHp;
    uint32_t GameStatus;
    uint32_t RobotMotion;
    uint32_t GroundRobotPosition;
    uint32_t RfidStatus;
    uint32_t RobotStatus;
    uint32_t JointState;
    uint32_t Buff;
} LastSendTime_t;
static LastSendTime_t LAST_SEND_TIME;

/*******************************************************************************/
/* USB Receive Callback                                                        */
/*******************************************************************************/

/**
 * @brief  USB接收回调函数，由drv_usb中间件调用
 * @param  Buffer: 接收数据缓冲区
 * @param  Length: 接收数据长度
 */
static void UsbRxCallback(uint8_t *Buffer, uint16_t Length)
{
    if (Length > USB_RX_DATA_SIZE) Length = USB_RX_DATA_SIZE;
    memcpy(USB_RX_BUF, Buffer, Length);
    USB_RX_LEN = Length;
    USB_RX_FLAG = true;
}

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/

static void UsbSendData(void);
static void UsbReceiveData(void);
static void UsbInit(void);

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/

static void UsbSendDebugData(void);
static void UsbSendImuData(void);
static void UsbSendRobotStateInfoData(void);
static void UsbSendEventData(void);
static void UsbSendAllRobotHpData(void);
static void UsbSendGameStatusData(void);
static void UsbSendRobotMotionData(void);
static void UsbSendGroundRobotPositionData(void);
static void UsbSendRfidStatusData(void);
static void UsbSendRobotStatusData(void);
static void UsbSendJointStateData(void);
static void UsbSendBuffData(void);

/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/

static void GetCmdData(void);
static void GetVirtualRcCtrlData(void);

/******************************************************************/
/* Task                                                           */
/******************************************************************/

/**
 * @brief      USB任务主函数
 * @param[in]  argument: 任务参数
 * @retval     None
 */
void usb_task(void const *argument)
{
    (void)argument;

    Publish(&ROBOT_CMD_DATA, const_cast<char *>(ROBOT_CMD_DATA_NAME));
    Publish(const_cast<bool *>(&USB_OFFLINE), const_cast<char *>(USB_OFFLINE_NAME));
    Publish(&VIRTUAL_RC_CTRL, const_cast<char *>(VIRTUAL_RC_NAME));

    // USB已由Task_Init中的USB_Init()初始化
    // 注册本任务的接收回调
    USB_Init(UsbRxCallback);

    vTaskDelay(10);  // 等待USB设备就绪
    UsbInit();

    while (1) {
        UsbSendData();
        UsbReceiveData();
        GetCmdData();
        GetVirtualRcCtrlData();

        if (HAL_GetTick() - RECEIVE_TIME > USB_OFFLINE_THRESHOLD) {
            USB_OFFLINE = true;
            CONTINUE_RECEIVE_CNT = 0;
        } else if (CONTINUE_RECEIVE_CNT > USB_CONNECT_CNT) {
            USB_OFFLINE = false;
        } else {
            CONTINUE_RECEIVE_CNT++;
        }

        vTaskDelay(USB_TASK_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        usb_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/

/**
 * @brief      USB初始化
 */
static void UsbInit(void)
{
    // 订阅数据
    IMU = (const Imu_t *)Subscribe((char *)IMU_NAME);
    FDB_SPEED_VECTOR = (const ChassisSpeedVector_t *)Subscribe((char *)CHASSIS_FDB_SPEED_NAME);

    // 数据置零
    memset(&LAST_SEND_TIME, 0, sizeof(LastSendTime_t));
    memset(&RECEIVE_ROBOT_CMD_DATA, 0, sizeof(ReceiveDataRobotCmd_s));
    memset(&RECEIVE_PID_DEBUG_DATA, 0, sizeof(ReceiveDataPidDebug_s));
    memset(&RECEIVE_VIRTUAL_RC_DATA, 0, sizeof(ReceiveDataVirtualRc_s));
    memset(&ROBOT_CMD_DATA, 0, sizeof(RobotCmdData_t));
    memset(&VIRTUAL_RC_CTRL, 0, sizeof(RC_ctrl_t));

    /*******************************************************************************/
    /* Serial                                                                     */
    /*******************************************************************************/
    
    // 1.初始化调试数据包
    SEND_DATA_DEBUG.frame_header.sof = SEND_SOF;
    SEND_DATA_DEBUG.frame_header.len = (uint8_t)(sizeof(SendDataDebug_s) - 6);
    SEND_DATA_DEBUG.frame_header.id = DEBUG_DATA_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_DATA_DEBUG.frame_header), sizeof(SEND_DATA_DEBUG.frame_header));
    for (uint8_t i = 0; i < DEBUG_PACKAGE_NUM; i++) {
        SEND_DATA_DEBUG.packages[i].type = 1;
        SEND_DATA_DEBUG.packages[i].name[0] = '\0';
    }
    
    // 2.初始化IMU数据包
    SEND_DATA_IMU.frame_header.sof = SEND_SOF;
    SEND_DATA_IMU.frame_header.len = (uint8_t)(sizeof(SendDataImu_s) - 6);
    SEND_DATA_IMU.frame_header.id = IMU_DATA_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_DATA_IMU.frame_header), sizeof(SEND_DATA_IMU.frame_header));

    /*******************************************************************************/
    /* Referee                                                                     */
    /*******************************************************************************/
    
    // 3.初始化机器人信息数据包
    SEND_DATA_ROBOT_STATE_INFO.frame_header.sof = SEND_SOF;
    SEND_DATA_ROBOT_STATE_INFO.frame_header.len = (uint8_t)(sizeof(SendDataRobotStateInfo_s) - 6);
    SEND_DATA_ROBOT_STATE_INFO.frame_header.id = ROBOT_STATE_INFO_DATA_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_DATA_ROBOT_STATE_INFO.frame_header), sizeof(SEND_DATA_ROBOT_STATE_INFO.frame_header));
    SEND_DATA_ROBOT_STATE_INFO.data.type.chassis = CHASSIS_TYPE;
    SEND_DATA_ROBOT_STATE_INFO.data.type.gimbal = GIMBAL_TYPE;
    SEND_DATA_ROBOT_STATE_INFO.data.type.shoot = SHOOT_TYPE;
    SEND_DATA_ROBOT_STATE_INFO.data.type.arm = MECHANICAL_ARM_TYPE;
    
    // 4.初始化事件数据包
    SEND_DATA_EVENT.frame_header.sof = SEND_SOF;
    SEND_DATA_EVENT.frame_header.len = (uint8_t)(sizeof(SendDataEvent_s) - 6);
    SEND_DATA_EVENT.frame_header.id = EVENT_DATA_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_DATA_EVENT.frame_header), sizeof(SEND_DATA_EVENT.frame_header));

    // 5.初始化pid调参数据
    SEND_DATA_PID.frame_header.sof = SEND_SOF;
    SEND_DATA_PID.frame_header.len = (uint8_t)(sizeof(SendDataPidDebug_s) - 6);
    SEND_DATA_PID.frame_header.id = PID_DEBUG_DATA_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_DATA_PID.frame_header), sizeof(SEND_DATA_PID.frame_header));

    // 6.初始化所有机器人血量数据
    SEND_DATA_ALL_ROBOT_HP.frame_header.sof = SEND_SOF;
    SEND_DATA_ALL_ROBOT_HP.frame_header.len = (uint8_t)(sizeof(SendDataAllRobotHp_s) - 6);
    SEND_DATA_ALL_ROBOT_HP.frame_header.id = ALL_ROBOT_HP_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_DATA_ALL_ROBOT_HP.frame_header),
        sizeof(SEND_DATA_ALL_ROBOT_HP.frame_header));

    // 7.初始化比赛状态数据
    SEND_DATA_GAME_STATUS.frame_header.sof = SEND_SOF;
    SEND_DATA_GAME_STATUS.frame_header.len = (uint8_t)(sizeof(SendDataGameStatus_s) - 6);
    SEND_DATA_GAME_STATUS.frame_header.id = GAME_STATUS_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_DATA_GAME_STATUS.frame_header),
        sizeof(SEND_DATA_GAME_STATUS.frame_header));
    
    // 8.初始化机器人运动数据
    SEND_ROBOT_MOTION_DATA.frame_header.sof = SEND_SOF;
    SEND_ROBOT_MOTION_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRobotMotion_s) - 6);
    SEND_ROBOT_MOTION_DATA.frame_header.id = ROBOT_MOTION_DATA_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_ROBOT_MOTION_DATA.frame_header),
        sizeof(SEND_ROBOT_MOTION_DATA.frame_header));
    
    // 9.初始化地面机器人位置数据
    SEND_GROUND_ROBOT_POSITION_DATA.frame_header.sof = SEND_SOF;
    SEND_GROUND_ROBOT_POSITION_DATA.frame_header.len = (uint8_t)(sizeof(SendDataGroundRobotPosition_s) - 6);
    SEND_GROUND_ROBOT_POSITION_DATA.frame_header.id = GROUND_ROBOT_POSITION_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_GROUND_ROBOT_POSITION_DATA.frame_header),
        sizeof(SEND_GROUND_ROBOT_POSITION_DATA.frame_header));

    // 10.初始化RFID状态数据
    SEND_RFID_STATUS_DATA.frame_header.sof = SEND_SOF;
    SEND_RFID_STATUS_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRfidStatus_s) - 6);
    SEND_RFID_STATUS_DATA.frame_header.id = RFID_STATUS_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_RFID_STATUS_DATA.frame_header),
        sizeof(SEND_RFID_STATUS_DATA.frame_header));

    // 11.初始化机器人状态数据
    SEND_ROBOT_STATUS_DATA.frame_header.sof = SEND_SOF;
    SEND_ROBOT_STATUS_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRobotStatus_s) - 6);
    SEND_ROBOT_STATUS_DATA.frame_header.id = ROBOT_STATUS_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_ROBOT_STATUS_DATA.frame_header),
        sizeof(SEND_ROBOT_STATUS_DATA.frame_header));
    
    // 12.初始化云台状态数据
    SEND_JOINT_STATE_DATA.frame_header.sof = SEND_SOF;
    SEND_JOINT_STATE_DATA.frame_header.len = (uint8_t)(sizeof(SendDataJointState_s) - 6);
    SEND_JOINT_STATE_DATA.frame_header.id = JOINT_STATE_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_JOINT_STATE_DATA.frame_header),
        sizeof(SEND_JOINT_STATE_DATA.frame_header));

    // 13.初始化机器人增益和底盘能量数据
    SEND_BUFF_DATA.frame_header.sof = SEND_SOF;
    SEND_BUFF_DATA.frame_header.len = (uint8_t)(sizeof(SendDataBuff_s) - 6);
    SEND_BUFF_DATA.frame_header.id = BUFF_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_BUFF_DATA.frame_header),
        sizeof(SEND_BUFF_DATA.frame_header));
}

/**
 * @brief      用USB发送数据
 */
static void UsbSendData(void)
{
    CheckDurationAndSend(Debug);
    CheckDurationAndSend(Imu);
    CheckDurationAndSend(RobotStateInfo);
    CheckDurationAndSend(Event);
    CheckDurationAndSend(AllRobotHp);
    CheckDurationAndSend(GameStatus);
    CheckDurationAndSend(RobotMotion);
    CheckDurationAndSend(GroundRobotPosition);
    CheckDurationAndSend(RfidStatus);
    CheckDurationAndSend(RobotStatus);
    CheckDurationAndSend(JointState);
    CheckDurationAndSend(Buff);
}

/**
 * @brief      USB接收数据（回调驱动）
 */
static void UsbReceiveData(void)
{
    if (!USB_RX_FLAG) return;
    USB_RX_FLAG = false;

    uint16_t len = USB_RX_LEN;
    if (len == 0) return;

    uint8_t *sof_address = USB_RX_BUF;
    uint8_t *rx_data_end = USB_RX_BUF + len;

    while (sof_address < rx_data_end) {
        // 寻找帧头位置
        while (sof_address < rx_data_end && *sof_address != RECEIVE_SOF) {
            sof_address++;
        }
        if (sof_address >= rx_data_end) break;

        // 检查剩余长度是否够帧头
        if ((rx_data_end - sof_address) < HEADER_SIZE) break;

        // 检查CRC8校验
        bool crc8_ok = verify_CRC8_check_sum(sof_address, HEADER_SIZE);
        if (crc8_ok) {
            uint8_t data_len = sof_address[1];
            uint8_t data_id = sof_address[2];
            uint32_t total_len = (uint32_t)HEADER_SIZE + data_len + 2;

            // 检查剩余长度是否足够
            if ((uint32_t)(rx_data_end - sof_address) < total_len) break;

            // 检查整包CRC16校验
            bool crc16_ok = verify_CRC16_check_sum(sof_address, total_len);
            if (crc16_ok) {
                switch (data_id) {
                    case ROBOT_CMD_DATA_RECEIVE_ID:
                        if (total_len <= sizeof(ReceiveDataRobotCmd_s))
                            memcpy(&RECEIVE_ROBOT_CMD_DATA, sof_address, sizeof(ReceiveDataRobotCmd_s));
                        break;
                    case PID_DEBUG_DATA_RECEIVE_ID:
                        if (total_len <= sizeof(ReceiveDataPidDebug_s))
                            memcpy(&RECEIVE_PID_DEBUG_DATA, sof_address, sizeof(ReceiveDataPidDebug_s));
                        break;
                    case VIRTUAL_RC_DATA_RECEIVE_ID:
                        if (total_len <= sizeof(ReceiveDataVirtualRc_s))
                            memcpy(&RECEIVE_VIRTUAL_RC_DATA, sof_address, sizeof(ReceiveDataVirtualRc_s));
                        break;
                    default:
                        break;
                }
                if (*((uint32_t *)(&sof_address[4])) > LATEST_RX_TIMESTAMP) {
                    LATEST_RX_TIMESTAMP = *((uint32_t *)(&sof_address[4]));
                    RECEIVE_TIME = HAL_GetTick();
                }
            }
            sof_address += total_len;
        } else {
            sof_address++;
        }
    }
}

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/

static void UsbSendDebugData(void)
{
    SEND_DATA_DEBUG.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_DEBUG, sizeof(SendDataDebug_s));
    USB_Transmit_Data((uint8_t *)&SEND_DATA_DEBUG, sizeof(SendDataDebug_s));
}

static void UsbSendImuData(void)
{
    if (IMU == NULL) return;

    SEND_DATA_IMU.time_stamp = HAL_GetTick();
    SEND_DATA_IMU.data.yaw = IMU->angle[AX_Z];
    SEND_DATA_IMU.data.pitch = IMU->angle[AX_Y];
    SEND_DATA_IMU.data.roll = IMU->angle[AX_X];
    SEND_DATA_IMU.data.yaw_vel = IMU->gyro[AX_Z];
    SEND_DATA_IMU.data.pitch_vel = IMU->gyro[AX_Y];
    SEND_DATA_IMU.data.roll_vel = IMU->gyro[AX_X];

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
    USB_Transmit_Data((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
}

static void UsbSendRobotStateInfoData(void)
{
    SEND_DATA_ROBOT_STATE_INFO.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_ROBOT_STATE_INFO, sizeof(SendDataRobotStateInfo_s));
    USB_Transmit_Data((uint8_t *)&SEND_DATA_ROBOT_STATE_INFO, sizeof(SendDataRobotStateInfo_s));
}

static void UsbSendEventData(void)
{
    SEND_DATA_EVENT.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_EVENT, sizeof(SendDataEvent_s));
    USB_Transmit_Data((uint8_t *)&SEND_DATA_EVENT, sizeof(SendDataEvent_s));
}

static void UsbSendAllRobotHpData(void)
{
    SEND_DATA_ALL_ROBOT_HP.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_ALL_ROBOT_HP, sizeof(SendDataAllRobotHp_s));
    USB_Transmit_Data((uint8_t *)&SEND_DATA_ALL_ROBOT_HP, sizeof(SendDataAllRobotHp_s));
}

static void UsbSendGameStatusData(void)
{
    SEND_DATA_GAME_STATUS.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_GAME_STATUS, sizeof(SendDataGameStatus_s));
    USB_Transmit_Data((uint8_t *)&SEND_DATA_GAME_STATUS, sizeof(SendDataGameStatus_s));
}

static void UsbSendRobotMotionData(void)
{
    if (FDB_SPEED_VECTOR == NULL) return;

    SEND_ROBOT_MOTION_DATA.time_stamp = HAL_GetTick();
    SEND_ROBOT_MOTION_DATA.data.speed_vector.vx = FDB_SPEED_VECTOR->vx;
    SEND_ROBOT_MOTION_DATA.data.speed_vector.vy = FDB_SPEED_VECTOR->vy;
    SEND_ROBOT_MOTION_DATA.data.speed_vector.wz = FDB_SPEED_VECTOR->wz;

    append_CRC16_check_sum((uint8_t *)&SEND_ROBOT_MOTION_DATA, sizeof(SendDataRobotMotion_s));
    USB_Transmit_Data((uint8_t *)&SEND_ROBOT_MOTION_DATA, sizeof(SendDataRobotMotion_s));
}

static void UsbSendGroundRobotPositionData(void)
{
    SEND_GROUND_ROBOT_POSITION_DATA.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_GROUND_ROBOT_POSITION_DATA, sizeof(SendDataGroundRobotPosition_s));
    USB_Transmit_Data((uint8_t *)&SEND_GROUND_ROBOT_POSITION_DATA, sizeof(SendDataGroundRobotPosition_s));
}

static void UsbSendRfidStatusData(void)
{
    SEND_RFID_STATUS_DATA.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_RFID_STATUS_DATA, sizeof(SendDataRfidStatus_s));
    USB_Transmit_Data((uint8_t *)&SEND_RFID_STATUS_DATA, sizeof(SendDataRfidStatus_s));
}

static void UsbSendRobotStatusData(void)
{
    SEND_ROBOT_STATUS_DATA.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_ROBOT_STATUS_DATA, sizeof(SendDataRobotStatus_s));
    USB_Transmit_Data((uint8_t *)&SEND_ROBOT_STATUS_DATA, sizeof(SendDataRobotStatus_s));
}

static void UsbSendJointStateData(void)
{
    SEND_JOINT_STATE_DATA.time_stamp = HAL_GetTick();
#if GIMBAL_TYPE != GIMBAL_NONE
    SEND_JOINT_STATE_DATA.data.pitch = CmdGimbalJointState(AX_PITCH);
    SEND_JOINT_STATE_DATA.data.yaw = CmdGimbalJointState(AX_YAW);
#endif
    append_CRC16_check_sum((uint8_t *)&SEND_JOINT_STATE_DATA, sizeof(SendDataJointState_s));
    USB_Transmit_Data((uint8_t *)&SEND_JOINT_STATE_DATA, sizeof(SendDataJointState_s));
}

static void UsbSendBuffData(void)
{
    SEND_BUFF_DATA.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_BUFF_DATA, sizeof(SendDataBuff_s));
    USB_Transmit_Data((uint8_t *)&SEND_BUFF_DATA, sizeof(SendDataBuff_s));
}

/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/

static void GetCmdData(void)
{
    ROBOT_CMD_DATA.speed_vector.vx = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vx;
    ROBOT_CMD_DATA.speed_vector.vy = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vy;
    ROBOT_CMD_DATA.speed_vector.wz = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.wz;

    ROBOT_CMD_DATA.chassis.yaw = RECEIVE_ROBOT_CMD_DATA.data.chassis.yaw;
    ROBOT_CMD_DATA.chassis.pitch = RECEIVE_ROBOT_CMD_DATA.data.chassis.pitch;
    ROBOT_CMD_DATA.chassis.roll = RECEIVE_ROBOT_CMD_DATA.data.chassis.roll;
    ROBOT_CMD_DATA.chassis.leg_length = RECEIVE_ROBOT_CMD_DATA.data.chassis.leg_lenth;

    ROBOT_CMD_DATA.gimbal.yaw = RECEIVE_ROBOT_CMD_DATA.data.gimbal.yaw;
    ROBOT_CMD_DATA.gimbal.pitch = RECEIVE_ROBOT_CMD_DATA.data.gimbal.pitch;

    ROBOT_CMD_DATA.shoot.fire = RECEIVE_ROBOT_CMD_DATA.data.shoot.fire;
    ROBOT_CMD_DATA.shoot.fric_on = RECEIVE_ROBOT_CMD_DATA.data.shoot.fric_on;
}

static void GetVirtualRcCtrlData(void)
{
    memcpy(&VIRTUAL_RC_CTRL, &RECEIVE_VIRTUAL_RC_DATA.data, sizeof(RC_ctrl_t));
}

/*******************************************************************************/
/* Public Function                                                             */
/*******************************************************************************/

/**
 * @brief 修改调试数据包
 * @param index 索引(0 ~ DEBUG_PACKAGE_NUM-1)
 * @param data  发送数据
 * @param name  数据名称
 */
void ModifyDebugDataPackage(uint8_t index, float data, const char *name)
{
    if (index >= DEBUG_PACKAGE_NUM) return;

    SEND_DATA_DEBUG.packages[index].data = data;

    if (SEND_DATA_DEBUG.packages[index].name[0] != '\0') return;

    uint8_t i = 0;
    while (name[i] != '\0' && i < 10) {
        SEND_DATA_DEBUG.packages[index].name[i] = name[i];
        i++;
    }
}

/**
 * @brief 获取上位机控制指令：云台姿态
 */
float GetScCmdGimbalAngle(uint8_t axis)
{
    if (axis == AX_YAW) return ROBOT_CMD_DATA.gimbal.yaw;
    if (axis == AX_PITCH) return ROBOT_CMD_DATA.gimbal.pitch;
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：底盘线速度
 */
float GetScCmdChassisSpeed(uint8_t axis)
{
    if (axis == AX_X) return ROBOT_CMD_DATA.speed_vector.vx;
    if (axis == AX_Y) return ROBOT_CMD_DATA.speed_vector.vy;
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：底盘角速度
 */
float GetScCmdChassisVelocity(uint8_t axis)
{
    if (axis == AX_Z) return ROBOT_CMD_DATA.speed_vector.wz;
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：底盘离地高度
 */
float GetScCmdChassisHeight(void)
{
    return ROBOT_CMD_DATA.chassis.leg_length;
}

/**
 * @brief 获取上位机控制指令：开火
 */
bool GetScCmdFire(void)
{
    return ROBOT_CMD_DATA.shoot.fire;
}

/**
 * @brief 获取上位机控制指令：启动摩擦轮
 */
bool GetScCmdFricOn(void)
{
    return ROBOT_CMD_DATA.shoot.fric_on;
}
/*------------------------------ End of File ------------------------------*/
