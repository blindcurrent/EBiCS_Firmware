
#include "stm32f1xx_hal.h"
#include "system.h"
#include "display_debug.h"

void MX_IWDG_Init(void);

IWDG_HandleTypeDef hiwdg;

extern uint8_t ui8_g_motor_error_state_hall_count; // from hall.c
    
static uint32_t ui32_motor_error_state = MOTOR_STATE_NORMAL;

static uint8_t ui8_pwm_enabled_flag = 0;

uint8_t is_pwm_enabled()
{
    return ui8_pwm_enabled_flag;
}

uint32_t get_motor_error_state()
{
    return ui32_motor_error_state;
}

inline static uint8_t is_motor_error_set(motor_error_state_t err)
{
    if(ui32_motor_error_state & (1 << err))
        return 1;
    else
        return 0;
}

inline static void clear_motor_error_flag(motor_error_state_t err)
{
    if(ui32_motor_error_state & (1 << err))
    {
        ui32_motor_error_state &= ~(1 << err);       // beware: bit masking is not atomic
    }
}

uint32_t process_errors(MotorState_t *pMS)
{

    if(ui8_g_motor_error_state_hall_count == 0)
    {
        const uint32_t mask = (1 << MOTOR_STATE_BLOCKED) | (1 << MOTOR_STATE_HALL_ERROR) | (1 << MOTOR_STATE_PLL_ERROR) | (1 << MOTOR_STATE_DBG_ERROR) | (1 << MOTOR_STATE_OVER_SPEED);
        if(ui32_motor_error_state & mask)
        {
            ui32_motor_error_state &= ~mask; // clear errors
        }               
    }           
    
    //if(pMS->u_q >= _U_MAX && pMS->CurrentData.q31_battery_current_mA < -2000) // todo: && !recuperation_flag
    //{
    //    // uncontrolled recuperation due to overspeed
    //    if(!is_motor_error_set(MOTOR_STATE_OVER_SPEED))
    //        trigger_motor_error(MOTOR_STATE_OVER_SPEED);
    //}

    // voltage
    if(pMS->ADC_Data.BatteryData.enum_voltage_state == VOLTAGE_STATE_UNDER_VOLTAGE_ERROR)
    {
        if(!is_motor_error_set(MOTOR_STATE_BATTERY_UNDERVOLTAGE))   
            trigger_motor_error(MOTOR_STATE_BATTERY_UNDERVOLTAGE);
    }
    else
    {
        if(is_motor_error_set(MOTOR_STATE_BATTERY_UNDERVOLTAGE))   
            clear_motor_error_flag(MOTOR_STATE_BATTERY_UNDERVOLTAGE);
    }

    // chip temperature
    if(pMS->ADC_Data.ChipTemperatureData.enum_temperature_state == TEMP_STATE_OVER_TEMP_ERROR)
    {
        if(!is_motor_error_set(MOTOR_STATE_CHIP_OVER_TEMPERATURE))
            trigger_motor_error(MOTOR_STATE_CHIP_OVER_TEMPERATURE);
    }
    else
    {
        if(is_motor_error_set(MOTOR_STATE_CHIP_OVER_TEMPERATURE))
            clear_motor_error_flag(MOTOR_STATE_CHIP_OVER_TEMPERATURE);
    }

    #if DISPLAY_TYPE == DISPLAY_TYPE_DEBUG
    // testing error triggering
    if (pMS->ui16_dbg_value2)
    {
        trigger_motor_error(MOTOR_STATE_DBG_ERROR);
        info_printf("[DBG_ERROR]\n");
        pMS->ui16_dbg_value2 = 0;
    }
    #endif
    
    //
    // error triggering
    //

    if(ui32_motor_error_state != MOTOR_STATE_NORMAL)
    {
        disable_pwm();
    }
    
    return ui32_motor_error_state;

}

void enable_pwm_with_dynamic_switchtime()
{
	TIM1->CCR1 = switchtime[0]; //1023;
	TIM1->CCR2 = switchtime[1]; //1023;
	TIM1->CCR3 = switchtime[2]; //1023;
    //
    //MS.HallData.enum_hall_angle_state = HALL_STATE_SIXSTEP;
    ui8_pwm_enabled_flag = 1;
    //
	SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
}

void enable_pwm()
{
	TIM1->CCR1 = 1023;
	TIM1->CCR2 = 1023;
	TIM1->CCR3 = 1023;
    //
    //MS.HallData.enum_hall_angle_state = HALL_STATE_SIXSTEP;
    ui8_pwm_enabled_flag = 1;
    //
	SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
}

void disable_pwm()
{
   	CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE);
    //MS.HallData.enum_hall_angle_state = HALL_STATE_SIXSTEP;
    ui8_pwm_enabled_flag = 0;
}

void trigger_motor_error(motor_error_state_t err)
{
    //disable_pwm();
    ui32_motor_error_state |= (1 << err);   // beware: bit masking is not atomic
    ui8_g_motor_error_state_hall_count = 100;
#if DISPLAY_TYPE == DISPLAY_TYPE_DEBUG
    switch(err)
    {
        case MOTOR_STATE_BLOCKED:
            info_printf("\nERR: MOTOR_STATE_BLOCKED\n\n");
            break;
        case MOTOR_STATE_PLL_ERROR:
            info_printf("\nERR: MOTOR_STATE_PLL_ERROR\n\n");
            break;
        case MOTOR_STATE_HALL_ERROR:
            info_printf("\nERR: MOTOR_STATE_HALL_ERROR\n\n");
            break;
        case MOTOR_STATE_OVER_SPEED:
            info_printf("\nERR: OVER_SPEED\n\n");
            break;
        case MOTOR_STATE_CHIP_OVER_TEMPERATURE:
            info_printf("\nERR: OVER_TEMPERATURE\n\n");
            break;
        case MOTOR_STATE_BATTERY_UNDERVOLTAGE:
            info_printf("\nERR: BATTERY_UNDERVOLTAGE\n\n");
            break;
        case MOTOR_STATE_DBG_ERROR:
            info_printf("\nERR: MOTOR_STATE_DBG_ERROR\n\n");
            break;
        default:
            info_printf("\nERR: UNKNOWN\n");
            break;        
    }
#endif
}


void init_watchdog()
{
#ifdef ACTIVATE_WATCHDOG
    if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
    {
        #if DISPLAY_TYPE == DISPLAY_TYPE_DEBUG
        info_printf("watchdog reset!\n");
        while(ui8_g_UART_TxCplt_flag == 0) {};
        #endif
        // do not continue here if reset from watchdog
        while(1){}
        //__HAL_RCC_CLEAR_RESET_FLAGS();
    }
    else
    {
        #if DISPLAY_TYPE == DISPLAY_TYPE_DEBUG
        info_printf("regular reset\n\n");
        while(ui8_g_UART_TxCplt_flag == 0) {};
        #endif
    }
    // start independent watchdog
    MX_IWDG_Init();
#endif

}

/* IWDG init function */
void MX_IWDG_Init(void)
{
  // RM0008 - Table 96 IWDG timout period in seconds:
  // (IWDG_PRESCALER) * (Period + 1) / f_LSI
  // datasheet STM32F103x4 -> f_LSI = 40'000 Hz
  // 
  // 4 * 500 / 40000 = 0.05s
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 500;
  // start the watchdog timer
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
