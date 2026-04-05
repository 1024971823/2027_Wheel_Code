/**
 * @file tsk_config_and_callback.cpp
 * @brief 任务调度与硬件回调
 */

#include "tsk_config_and_callback.h"

#include "app_chassis.h"
#include "app_gimbal.h"
#include "bsp_bmi088.h"
#include "bsp_buzzer.h"
#include "bsp_key.h"
#include "bsp_power.h"
#include "bsp_w25q64jv.h"
#include "bsp_ws2812.h"
#include "drv_wdg.h"
#include "dvc_vofa.h"
#include "i6x.h"
#include "remote_control.h"
#include "sys_timestamp.h"

Class_Vofa_USB Vofa_USB;
char Vofa_Variable_Assignment_List[][VOFA_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH] = {"q00", "q11", "r00", "r11"};

int32_t red = 0;
int32_t green = 12;
int32_t blue = 12;
bool red_minus_flag = false;
bool green_minus_flag = false;
bool blue_minus_flag = true;

bool init_finished = false;

uint32_t debug_last_unknown_can_id = 0;
uint32_t debug_unknown_can_count = 0;

void Serial_USB_Call_Back(uint8_t *Buffer, uint16_t Length)
{
    Vofa_USB.USB_RxCallback(Buffer, Length);
    App_Gimbal_Vofa_Set_Debug_Variable(Vofa_USB.Get_Variable_Index(), Vofa_USB.Get_Variable_Value());
}

void SPI2_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length)
{
    (void)Tx_Buffer;
    (void)Rx_Buffer;
    (void)Tx_Length;
    (void)Rx_Length;

    if ((SPI2_Manage_Object.Activate_GPIOx == BMI088_ACCEL__SPI_CS_GPIO_Port &&
         SPI2_Manage_Object.Activate_GPIO_Pin == BMI088_ACCEL__SPI_CS_Pin) ||
        (SPI2_Manage_Object.Activate_GPIOx == BMI088_GYRO__SPI_CS_GPIO_Port &&
         SPI2_Manage_Object.Activate_GPIO_Pin == BMI088_GYRO__SPI_CS_Pin))
    {
        BSP_BMI088.SPI_RxCpltCallback();
    }
}

void CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    (void)Buffer;

    switch (Header.Identifier)
    {
    case 0x206:
        App_Gimbal_CAN_RxCpltCallback();
        break;
    case 0x00:
        debug_unknown_can_count++;
        App_Chassis_CAN_RxCpltCallback();
        break;
    default:
        debug_last_unknown_can_id = Header.Identifier;
        break;
    }
}

/* UART5 遥控器回调已迁移至 remote_control.c :: remote_control_rx_callback()
 * 通过 bsp_rc → drv_uart 中间件注册, 见 remote_control_init() */

void OSPI2_Polling_Callback()
{
    BSP_W25Q64JV.OSPI_StatusMatchCallback();
}

void OSPI2_Rx_Callback(uint8_t *Buffer)
{
    (void)Buffer;
    BSP_W25Q64JV.OSPI_RxCallback();
}

void OSPI2_Tx_Callback(uint8_t *Buffer)
{
    (void)Buffer;
    BSP_W25Q64JV.OSPI_TxCallback();
}

void Task3600s_Callback()
{
    SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
}

void Task1s_Callback()
{
}

void Task1ms_Callback()
{
    static int mod10 = 0;
    mod10++;
    if (mod10 == 10)
    {
        mod10 = 0;

        if (red >= 18)
        {
            red_minus_flag = true;
        }
        else if (red == 0)
        {
            red_minus_flag = false;
        }

        if (green >= 18)
        {
            green_minus_flag = true;
        }
        else if (green == 0)
        {
            green_minus_flag = false;
        }

        if (blue >= 18)
        {
            blue_minus_flag = true;
        }
        else if (blue == 0)
        {
            blue_minus_flag = false;
        }

        red += red_minus_flag ? -1 : 1;
        green += green_minus_flag ? -1 : 1;
        blue += blue_minus_flag ? -1 : 1;

        BSP_WS2812.Set_RGB(red, green, blue);
        BSP_WS2812.TIM_10ms_Write_PeriodElapsedCallback();
    }

    BSP_Buzzer.Set_Sound(0.0f, 0.0f);

    BSP_Key.TIM_1ms_Process_PeriodElapsedCallback();
    static int mod50 = 0;
    mod50++;
    if (mod50 == 50)
    {
        mod50 = 0;
        BSP_Key.TIM_50ms_Read_PeriodElapsedCallback();
    }

    App_Gimbal_Task_1ms_Callback();

    static int mod128 = 0;
    mod128++;
    if (mod128 == 128)
    {
        mod128 = 0;
        BSP_BMI088.TIM_128ms_Calculate_PeriodElapsedCallback();
    }

    TIM_1ms_CAN_PeriodElapsedCallback();
    TIM_1ms_IWDG_PeriodElapsedCallback();
}

void Task125us_Callback()
{
    BSP_BMI088.TIM_125us_Calculate_PeriodElapsedCallback();
}

void Task10us_Callback()
{
    BSP_BMI088.TIM_10us_Calculate_PeriodElapsedCallback();
}

void Task_Init()
{
    SYS_Timestamp.Init(&htim5);

    USB_Init(Serial_USB_Call_Back);
    SPI_Init(&hspi2, SPI2_Callback);
    SPI_Init(&hspi6, nullptr);
    CAN_Init(&hfdcan1, CAN1_Callback);
    ADC_Init(&hadc1, 1);
    OSPI_Init(&hospi2, OSPI2_Polling_Callback, OSPI2_Rx_Callback, OSPI2_Tx_Callback);

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim8);

    Vofa_USB.Init(4, reinterpret_cast<const char **>(Vofa_Variable_Assignment_List));

    BSP_WS2812.Init(0, 0, 0);
    BSP_Buzzer.Init();
    BSP_Power.Init();
    BSP_Key.Init();
    BSP_BMI088.Init();

    App_Gimbal_Init(&hfdcan1);
    App_Chassis_Init(&hfdcan1, &Vofa_USB);

    remote_control_init();

    BSP_W25Q64JV.Init();

    init_finished = true;
}

void Task_Loop()
{
    Namespace_SYS_Timestamp::Delay_Millisecond(1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (!init_finished)
    {
        return;
    }

    if (GPIO_Pin == BMI088_ACCEL__INTERRUPT_Pin || GPIO_Pin == BMI088_GYRO__INTERRUPT_Pin)
    {
        BSP_BMI088.EXTI_Flag_Callback(GPIO_Pin);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM13)
    {
        HAL_IncTick();
        return;
    }

    if (!init_finished)
    {
        return;
    }

    if (htim->Instance == TIM4)
    {
        Task10us_Callback();
    }
    else if (htim->Instance == TIM5)
    {
        Task3600s_Callback();
    }
    else if (htim->Instance == TIM6)
    {
        Task1s_Callback();
    }
    else if (htim->Instance == TIM7)
    {
        Task1ms_Callback();
    }
    else if (htim->Instance == TIM8)
    {
        Task125us_Callback();
    }
}

extern "C" void RTOS_Ctrl_Task_Loop(void)
{
    App_Chassis_Ctrl_Task_Loop();
}

extern "C" void RTOS_Remote_Task_Loop(void)
{
    static int mod10 = 0;
    mod10++;
    if (mod10 == 10)
    {
        mod10 = 0;

        if (red >= 18)
        {
            red_minus_flag = true;
        }
        else if (red == 0)
        {
            red_minus_flag = false;
        }

        if (green >= 18)
        {
            green_minus_flag = true;
        }
        else if (green == 0)
        {
            green_minus_flag = false;
        }

        if (blue >= 18)
        {
            blue_minus_flag = true;
        }
        else if (blue == 0)
        {
            blue_minus_flag = false;
        }

        red += red_minus_flag ? -1 : 1;
        green += green_minus_flag ? -1 : 1;
        blue += blue_minus_flag ? -1 : 1;

        BSP_WS2812.Set_RGB(red, green, blue);
        BSP_WS2812.TIM_10ms_Write_PeriodElapsedCallback();
    }

    /* 遥控器掉线检测 + 数据可在此处理 */
    (void)get_remote_control_point();
    (void)get_i6x_point();
}

extern "C" void RTOS_Monitor_Task_Loop(void)
{
    App_Chassis_Monitor_Task_Loop();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
