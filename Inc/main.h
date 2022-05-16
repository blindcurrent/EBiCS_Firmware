/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <arm_math.h>


/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Hall_1_Pin GPIO_PIN_0
#define Hall_1_GPIO_Port GPIOA
#define Hall_1_EXTI_IRQn EXTI0_IRQn
#define Hall_2_Pin GPIO_PIN_1
#define Hall_2_GPIO_Port GPIOA
#define Hall_2_EXTI_IRQn EXTI1_IRQn
#define Hall_3_Pin GPIO_PIN_2
#define Hall_3_GPIO_Port GPIOA
#define Hall_3_EXTI_IRQn EXTI2_IRQn
#define Throttle_Pin GPIO_PIN_3
#define Throttle_GPIO_Port GPIOA
#define Phase_Current1_Pin GPIO_PIN_4
#define Phase_Current1_GPIO_Port GPIOA
#define Phase_Current_2_Pin GPIO_PIN_5
#define Phase_Current_2_GPIO_Port GPIOA
#define Phase_Current_3_Pin GPIO_PIN_6
#define Phase_Current_3_GPIO_Port GPIOA
#define Temperature_Pin GPIO_PIN_1
#define Temperature_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOB
#define LIGHT_Pin GPIO_PIN_9
#define LIGHT_GPIO_Port GPIOB
#define PAS_Pin GPIO_PIN_8
#define PAS_GPIO_Port GPIOB
#define Brake_Pin GPIO_PIN_11
#define Brake_GPIO_Port GPIOA
#define Speed_EXTI5_Pin GPIO_PIN_5
#define Speed_EXTI5_GPIO_Port GPIOB
#define Speed_EXTI5_EXTI_IRQn EXTI9_5_IRQn
#define PAS_EXTI8_Pin GPIO_PIN_8
#define PAS_EXTI8_GPIO_Port GPIOB
#define PAS_EXTI8_EXTI_IRQn EXTI9_5_IRQn



/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */




void runPIcontrol();


void UART_IdleItCallback(void);

extern volatile uint16_t switchtime[3];

extern const q31_t DEG_0;
extern const q31_t DEG_plus60;
extern const q31_t DEG_plus120;
extern const q31_t DEG_plus180;
extern const q31_t DEG_minus60;
extern const q31_t DEG_minus120;
extern const q31_t Q31_DEGREE;
extern const int32_t MAX_INT32;
extern const int32_t MIN_INT32;


typedef enum {HALL_STATE_SIXSTEP = 0, HALL_STATE_EXTRAPOLATION = 1, HALL_STATE_PLL = 2} hall_angle_state_t;

typedef enum {MOTOR_STATE_NORMAL = 0, 
                MOTOR_STATE_BLOCKED = 1, 
                MOTOR_STATE_PLL_ERROR = 2, 
                MOTOR_STATE_HALL_ERROR = 3, 
                MOTOR_STATE_CHIP_OVER_TEMPERATURE = 4,
                MOTOR_STATE_BATTERY_UNDERVOLTAGE = 5,
                MOTOR_STATE_OVER_SPEED = 6,
                MOTOR_STATE_DBG_ERROR = 10,
                } motor_error_state_t;

typedef enum {VOLTAGE_STATE_NORMAL = 0, VOLTAGE_STATE_CRITICAL = 1, VOLTAGE_STATE_UNDER_VOLTAGE_ERROR = 2} voltage_state_t;

typedef enum {TEMP_STATE_NOMRAL = 0, TEMP_STATE_CRITICAL = 1, TEMP_STATE_OVER_TEMP_ERROR = 2} temperature_state_t;

typedef struct
{
    int32_t         i32_battery_voltage_Vx10;
    int32_t         i32_battery_voltage_uncorrected_Vx10;
    voltage_state_t enum_voltage_state;
    int32_t         i32_battery_current_mA;
    int32_t         i32_battery_power_W_x10;
    uint8_t         ui8_remaining_distance_km;
    uint16_t        ui16_remaining_time_min;
} BatteryData_t;

typedef struct
{
    q31_t           q31_temperature_adc_cumulated;
    uint8_t         ui8_shift;
    q31_t           q31_temperature_degrees;
    temperature_state_t enum_temperature_state;
} TemperatureData_t;

typedef struct
{
    TemperatureData_t ChipTemperatureData;
    TemperatureData_t MotorTemperatureData;
    BatteryData_t     BatteryData;
} ADC_Data_t;

typedef struct 
{
    // PAS data (pedal speed)
    // input
    volatile uint8_t ui8_PAS_flag;                  // indicate pas event : HAL_GPIO_EXTI_Callback
    volatile uint32_t uint32_PAS_counter;           // 8kHz counter (tim3) : HAL_TIM_PeriodElapsedCallback
    // output
    uint32_t       uint32_PAS;                      // tics between two pas sensor events (smoothed/filtered version of uint32_PAS_counter) -> todo: can be static in pas module
    uint32_t       uint32_PAS_omega_x10;            // angular frequency in rad/s x 10
    uint32_t       uint32_PAS_rpm;
    uint8_t        uint8_pedaling;                  // 1 if pedaling, 0 else

    // torque sensor data
    uint16_t       uint16_torque_adc_offset;
    // input
    volatile uint16_t       uint16_torque_adc;      // raw adc data (was ui16_reg_adc_value) : HAL_ADC_ConvCpltCallback
    // output
    uint32_t       uint32_torque_Nm_x10;            // torque value in Nm x 10
} PedalData_t;

typedef struct
{
    // state and flags
    hall_angle_state_t enum_hall_angle_state;       // state of the rotor angle estimation state machine
    uint8_t hall_angle_detect_flag;                 // autodetect

    // hall module input
    volatile uint8_t ui8_hall_timeout_flag;         // HAL_TIM_PeriodElapsedCallback
    volatile uint16_t ui16_timertics;               // timertics between two hall events
    volatile uint32_t uint32_tics_filtered;         // filtered version of ui16_timertics

    // hall module output
    q31_t q31_rotorposition_absolute;
    q31_t q31_rotorposition_hall;
    int32_t i32_omega_el;

} HallData_t;


typedef struct
{   
    uint32_t        uint32_SPEED_kmh_x10;
    uint16_t        ui16_wheel_time_ms;
} WheelSpeedData_t;



typedef struct
{

    uint8_t         ui8_lights;
    uint8_t         ui8_walk_assist;
    int32_t         i32_i_d_mA;
    int32_t         i32_i_q_mA;
	q31_t          	u_d;
	q31_t          	u_q;
	int32_t         i32_U_magnitude;
    int32_t         i32_U_angle;
	int8_t 		    i8_assist_level;
    //
    int32_t         i32_power_Wx10_for_display;

    //
    uint8_t         ui8_dbg_log_value;
    uint16_t        ui16_dbg_value;
    uint16_t        ui16_dbg_value2;
    uint8_t         ui8_go;
    uint8_t         ui8_log;

    PedalData_t PedalData;
    WheelSpeedData_t WheelSpeedData;
    HallData_t HallData;
    ADC_Data_t ADC_Data;


}MotorState_t;


typedef struct
{
	int32_t       	gain_p;
	int32_t       	gain_i;
	int32_t       	limit_output_min_shifted;
    int32_t         limit_integral_min_shifted;
	int32_t       	limit_output_max_shifted;
    int32_t         limit_integral_max_shifted;
	int32_t       	recent_value;
	int32_t       	setpoint;
	int32_t       	integral_part;
	int32_t       	max_step_shifted;
	int32_t       	out_shifted;
	uint8_t       	shift;
    int8_t          id;

}PI_control_t;

void disable_pwm(void);
void trigger_motor_error(motor_error_state_t err);

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
