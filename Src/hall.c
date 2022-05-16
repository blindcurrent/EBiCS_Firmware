#include "hall.h"
#include "varia.h"
#include "stm32f1xx_hal.h"

#include "display_debug.h"
#include "debug.h"
#include "system.h"

const q31_t DEG_0 = 0;
const q31_t DEG_plus60   =  715827882;
const q31_t DEG_plus120  =  1431655765;
const q31_t DEG_plus180  =  2147483647;
const q31_t DEG_minus60  = -715827882;
const q31_t DEG_minus120 = -1431655765;
const q31_t Q31_DEGREE   =  11930464;

// extern data
uint8_t ui8_g_motor_error_state_hall_count = 0;
extern uint8_t ui8_internal_SPEED_control_flag; // from wheel_speed.c


static q31_t q31_rotorposition_motor_specific = SPEC_ANGLE;
static int16_t i16_hall_order = 1;
static int8_t i8_recent_rotor_direction = 1;
static volatile uint8_t ui8_hall_state_old = 0;
    
// hall state machine
static uint8_t ui8_six_step_hall_count = 2;
static uint8_t ui8_extrapolation_hall_count = 5;

// pll
static q31_t q31_rotorposition_PLL = 0;
static q31_t q31_pll_angle_per_tic = 0;
static q31_t q31_speed_pll_i = 0;
static q31_t q31_pll_abs_delta = 715827882; // +60 degree

// autodetect
static volatile uint8_t ui8_hall_state__ = 0; // share hall_state with autodetect
static volatile uint8_t ui8_hall_case__ = 0; // share hall_case with autodetect

void init_hall_data(HallData_t *pHallData)
{
    pHallData->enum_hall_angle_state = HALL_STATE_SIXSTEP;
    pHallData->hall_angle_detect_flag = 0;
    //
    pHallData->ui8_hall_timeout_flag = 0;
    pHallData->uint32_tics_filtered = HALL_TIMEOUT;
    pHallData->ui16_timertics = HALL_TIMEOUT;


    pHallData->q31_rotorposition_absolute = 0;
    pHallData->q31_rotorposition_hall = 0;

    //#ifdef READ_SPEC_ANGLE_FROM_EEPROM
    //    EE_ReadVariable(EEPROM_POS_SPEC_ANGLE, &MP.spec_angle);
    //
    //    // set motor specific angle to value from emulated EEPROM only if valid
    //    if (MP.spec_angle != 0xFFFF)
    //    {
    //        q31_rotorposition_motor_specific = MP.spec_angle << 16;
    //        EE_ReadVariable(EEPROM_POS_HALL_ORDER, &i16_hall_order);
    //    }
    //#endif

    //q31_rotorposition_motor_specific = -167026406;
    //q31_rotorposition_motor_specific = -1729917383; // -145 degrees
    q31_rotorposition_motor_specific = -1789569706; // -150 degrees
    //q31_rotorposition_motor_specific = -1849222030; // -155 degrees
    //q31_rotorposition_motor_specific = -1801499903;  // -152 degrees
    //q31_rotorposition_motor_specific = 0;
    i16_hall_order = 1;
    // set absolute position to corresponding hall pattern.


    // void get_standstill_position()
	//HAL_Delay(100);
    uint8_t ui8_hall_state = GPIOA->IDR & 0b111; //Mask input register with Hall 1 - 3 bits
    switch (ui8_hall_state)
    {
    //6 cases for forward direction
    case 2:
        pHallData->q31_rotorposition_hall = DEG_0 + q31_rotorposition_motor_specific;
        break;
    case 6:
        pHallData->q31_rotorposition_hall = DEG_plus60 + q31_rotorposition_motor_specific;
        break;
    case 4:
        pHallData->q31_rotorposition_hall = DEG_plus120 + q31_rotorposition_motor_specific;
        break;
    case 5:
        pHallData->q31_rotorposition_hall = DEG_plus180 + q31_rotorposition_motor_specific;
        break;
    case 1:
        pHallData->q31_rotorposition_hall = DEG_minus120 + q31_rotorposition_motor_specific;
        break;
    case 3:
        pHallData->q31_rotorposition_hall = DEG_minus60 + q31_rotorposition_motor_specific;
        break;
    }

    ui8_hall_state_old = ui8_hall_state;
    pHallData->q31_rotorposition_absolute = pHallData->q31_rotorposition_hall + (DEG_plus60 >> 1);
}

/**
   * @brief Processing incoming hall event from HAL_GPIO_EXTI_Callback.
   * @param[in]  pHallData
   * @param[in]  ui16_tim2_recent   time since last hall transition.
   * @param[in]  ui8_hall_state     current hall state.
   * @param[out] q31_rotorposition_hall (via pHallData)
   * @param[out] q31_pll_angle_per_tic
   */

void process_hall_event(HallData_t *pHallData, uint16_t ui16_tim2_recent, uint8_t ui8_hall_state)
{
    static uint8_t ui8_hall_error_count = 0;
    static uint8_t ui8_pll_reset_count = 0;
    uint8_t ui8_hall_case = 0;

    if(is_pwm_enabled() == 0)
    {
        pHallData->enum_hall_angle_state = HALL_STATE_SIXSTEP;
        ui8_six_step_hall_count = 2;
        ui8_extrapolation_hall_count = 5;
    }

    //    
    static const uint8_t ui8_timertics_shift = 3;   // change the shift also below
    static uint32_t uint32_tics_cumulated = HALL_TIMEOUT << 3;

    //                                                      1  2  3  4  5  6
    const uint8_t next_expected_forward_hall_state[7] = {0, 3, 6, 2, 5, 1, 4};

    //
    // process incoming hall event
    //

    if (ui8_hall_state == next_expected_forward_hall_state[ui8_hall_state_old])
    {

        ui8_hall_case = ui8_hall_state_old * 10 + ui8_hall_state;
        ui8_hall_state_old = ui8_hall_state;

        if (pHallData->ui8_hall_timeout_flag)
        {
            // no hall event for a long time @ see HAL_TIM_PeriodElapsedCallback
            pHallData->ui8_hall_timeout_flag = 0;
            uint32_tics_cumulated = HALL_TIMEOUT << ui8_timertics_shift;
            pHallData->i32_omega_el = 1;
        }
        else
        {
            pHallData->ui16_timertics = ui16_tim2_recent; //save timertics since last hall event
            uint32_tics_cumulated -= uint32_tics_cumulated >> ui8_timertics_shift;
            uint32_tics_cumulated += ui16_tim2_recent;
            pHallData->uint32_tics_filtered = uint32_tics_cumulated >> ui8_timertics_shift;
            ui8_internal_SPEED_control_flag = 1;
            pHallData->i32_omega_el = (523600 / pHallData->uint32_tics_filtered);

            if (ui8_six_step_hall_count > 0)
            {
                --ui8_six_step_hall_count;
            }

            // clear motor error after a certain amount of valid hall transitions
            if (ui8_g_motor_error_state_hall_count > 0)
            {
                --ui8_g_motor_error_state_hall_count;
            }
        }

        switch (ui8_hall_case) //12 cases for each transition from one stage to the next. 6x forward, 6x reverse
        {
        //6 cases for forward direction
        case 64:
            pHallData->q31_rotorposition_hall = DEG_plus120 * i16_hall_order + q31_rotorposition_motor_specific;
            i8_recent_rotor_direction = 1;
            break;
        case 45:
            pHallData->q31_rotorposition_hall = DEG_plus180 * i16_hall_order + q31_rotorposition_motor_specific;
            i8_recent_rotor_direction = 1;
            break;
        case 51:
            pHallData->q31_rotorposition_hall = DEG_minus120 * i16_hall_order + q31_rotorposition_motor_specific;
            i8_recent_rotor_direction = 1;
            break;
        case 13:
            pHallData->q31_rotorposition_hall = DEG_minus60 * i16_hall_order + q31_rotorposition_motor_specific;
            i8_recent_rotor_direction = 1;
            break;
        case 32:
            pHallData->q31_rotorposition_hall = DEG_0 * i16_hall_order + q31_rotorposition_motor_specific;
            i8_recent_rotor_direction = 1;
            break;
        case 26:
            pHallData->q31_rotorposition_hall = DEG_plus60 * i16_hall_order + q31_rotorposition_motor_specific;
            i8_recent_rotor_direction = 1;
            break;

            //6 cases for reverse direction
            /*
        case 46:
            pHallData->q31_rotorposition_hall = DEG_plus120 * i16_hall_order + q31_rotorposition_motor_specific;
            i8_recent_rotor_direction = -1;
            break;
        case 62:
            pHallData->q31_rotorposition_hall = DEG_plus60 * i16_hall_order + q31_rotorposition_motor_specific;
            i8_recent_rotor_direction = -1;
            break;
        case 23:
            pHallData->q31_rotorposition_hall = DEG_0 * i16_hall_order + q31_rotorposition_motor_specific;
            i8_recent_rotor_direction = -1;
            break;
        case 31:
            pHallData->q31_rotorposition_hall = DEG_minus60 * i16_hall_order + q31_rotorposition_motor_specific;
            i8_recent_rotor_direction = -1;
            break;
        case 15:
            pHallData->q31_rotorposition_hall = DEG_minus120 * i16_hall_order + q31_rotorposition_motor_specific;
            i8_recent_rotor_direction = -1;
            break;
        case 54:
            pHallData->q31_rotorposition_hall = DEG_plus180 * i16_hall_order + q31_rotorposition_motor_specific;
            i8_recent_rotor_direction = -1;
            break;
        */
        } // end case

        if (pHallData->enum_hall_angle_state != HALL_STATE_SIXSTEP)
        {

            // ------------------------------------------- feed the phase locked loop
            q31_t delta = pHallData->q31_rotorposition_hall - q31_rotorposition_PLL;
            q31_t q31_speed_pll_p = delta >> P_FACTOR_PLL;
            q31_speed_pll_i += delta >> I_FACTOR_PLL;
            q31_pll_angle_per_tic = q31_speed_pll_i + q31_speed_pll_p;
            // -------------------------------------------

            if (delta < 0)
                q31_pll_abs_delta = -delta;
            else
                q31_pll_abs_delta = delta;
            
            if(pHallData->enum_hall_angle_state == HALL_STATE_EXTRAPOLATION)
            {
                if (q31_pll_abs_delta < Q31_DEGREE * 12)
                {
                    if (ui8_extrapolation_hall_count > 0)
                    {
                        --ui8_extrapolation_hall_count;
                    }
                }
                else
                {
                    // require the pll error to be smaller than 10 deg for several consecutive hall transitions
                    ui8_extrapolation_hall_count = 5;

                    // reset pll
                    if(ui8_pll_reset_count >= 20)
                    {
                        q31_speed_pll_i = (DEG_plus60 / pHallData->ui16_timertics) * 31;
                        q31_pll_angle_per_tic = q31_speed_pll_i;
                        q31_rotorposition_PLL = pHallData->q31_rotorposition_hall;
                        ui8_pll_reset_count = 0;
                    }
                    else
                    {
                        ui8_pll_reset_count += 1;
                    }

                }
            }
            else
            {
                ui8_extrapolation_hall_count = 5;
            }
        }
        else
        {
            ui8_extrapolation_hall_count = 5;
            // reset pll
            // 500'000 / 16000 = 31
            q31_speed_pll_i = (DEG_plus60 / pHallData->ui16_timertics) * 31;
            q31_pll_angle_per_tic = q31_speed_pll_i;
            q31_rotorposition_PLL = pHallData->q31_rotorposition_hall;
        }

        ui8_hall_error_count = 0;
    }
    else
    {
        // noticed that it happens from time to time that a hall state appears twice consecutively
        // guess: the signal is bouncing and we read the pin value with a slight delay in the interrupt
        //
        // exemplary hall sequence with error: 4 5 5 3 2
        // if that happens, the code enters this else statement
        // try to recover from it by setting ui8_hall_state_old to the expected hall state
        // the PLL loop should survive a missed hall event, q31_pll_angle_per_tic is simply not updated this time
        // in order to keep the extrapolation running, q31_rotorposition_hall is advanced by 60 degrees.
        // if everything goes well, the correct hall sequence is recovered in the next interrupt
        // if not -> trigger MOTOR_STATE_HALL_ERROR..

        if (ui8_hall_error_count >= 3)
        {
            if (is_pwm_enabled())
            {
                // only trigger the error when pwm is enabled
                // otherwise it can cause unnecessary delay of the motor power

                // TODO -> error triggering in main loop
                trigger_motor_error(MOTOR_STATE_HALL_ERROR);
                //pHallData->ui8_hall_error_flag = 1;
            }
        }
        else
        {
            ++ui8_hall_error_count;
        }

        if (ui8_hall_error_count == 1)
        {
            // try to bridge a missed hall event
            ui8_hall_state_old = next_expected_forward_hall_state[ui8_hall_state];
            #if DISPLAY_TYPE == DISPLAY_TYPE_DEBUG
            info_printf("\n+++ FIXED MISSED HALL EVENT +++\n");
            #endif
        }
        else
        {
            ui8_hall_state_old = ui8_hall_state;
        }
        pHallData->q31_rotorposition_hall += DEG_plus60;
    }

    // ------------------------- debug
    //info_printf("%u : %u\n", pHallData->enum_hall_angle_state, ui16_tim2_recent);

    //i16_fast_log_sample_buffer[0] = pHallData->q31_rotorposition_absolute >> 16;
    //i16_fast_log_sample_buffer[1] = ui16_tim2_recent;
    //fast_log_data();
    // -------------------------

    // store data for autodetect
    ui8_hall_state__ = ui8_hall_state;
    ui8_hall_case__ = ui8_hall_case;
}

/**
   * @brief Estimate the rotor angle in HAL_ADCEx_InjectedConvCpltCallback and run hall state machine.
   * @param[in]  pHallData
   * @param[in]  ui16_tim2_recent   time since last hall transition.
   * @param[out] q31_rotorposition_absolute (via pHallData)
   */
void estimate_rotor_angle(HallData_t *pHallData, uint16_t ui16_tim2_recent)
{ 
    if (pHallData->hall_angle_detect_flag == 1)
    {
        // angle detection is active
        return;
    }


    // Motor Blocked error
    //if(ui16_tim2_recent > ui16_timertics + (ui16_timertics >> 2))  // ui16_timertics * 5/4 was too sensitive
    if (ui16_tim2_recent > (pHallData->ui16_timertics * 2))  // todo try 3/2
    {
        trigger_motor_error(MOTOR_STATE_BLOCKED);
    }

    //
    // estimate angle
    //

    switch (pHallData->enum_hall_angle_state)
    {
    case HALL_STATE_SIXSTEP:
        pHallData->q31_rotorposition_absolute = pHallData->q31_rotorposition_hall + (DEG_plus60 >> 1);
        q31_rotorposition_PLL = pHallData->q31_rotorposition_hall;

        break;

    case HALL_STATE_EXTRAPOLATION:

        // extrapolation method
        // interpolate angle between two hallevents by scaling timer2 tics, 10923<<16 is 715827883 = 60°
        //pHallData->q31_rotorposition_absolute = pHallData->q31_rotorposition_hall + (q31_t)(i16_hall_order * i8_recent_rotor_direction * ((10923 * ui16_tim2_recent) / pHallData->ui16_timertics) << 16);
        pHallData->q31_rotorposition_absolute = pHallData->q31_rotorposition_hall + (q31_t)(((10923 * ((int32_t) ui16_tim2_recent)) / ((int32_t) pHallData->ui16_timertics)) << 16);
        // let the pll run in parallel (einschwingen)
        q31_rotorposition_PLL += q31_pll_angle_per_tic;

        break;

    case HALL_STATE_PLL:
        // PLL
        q31_rotorposition_PLL += q31_pll_angle_per_tic;
        pHallData->q31_rotorposition_absolute = q31_rotorposition_PLL;

        break;

    default:
        break;
    }
    
    // 
    // run the hall state machine
    //

    switch (pHallData->enum_hall_angle_state)
    {
    case HALL_STATE_SIXSTEP:    
        if (pHallData->uint32_tics_filtered < SIXSTEPTHRESHOLD_UP && ui8_six_step_hall_count == 0)
        {
            pHallData->enum_hall_angle_state = HALL_STATE_EXTRAPOLATION;
        }
        break;

    case HALL_STATE_EXTRAPOLATION:
        ui8_six_step_hall_count = 2;
        
        if (pHallData->ui16_timertics > SIXSTEPTHRESHOLD_DOWN)
        {
            pHallData->enum_hall_angle_state = HALL_STATE_SIXSTEP;
        }

        if (ui8_extrapolation_hall_count == 0)
        {
            pHallData->enum_hall_angle_state = HALL_STATE_PLL;
            #if DISPLAY_TYPE == DISPLAY_TYPE_DEBUG
            info_printf("[PLL TRANSITION (%d)]\n", q31_degree_to_degree(q31_pll_abs_delta));
            #endif
        }
        break;

    case HALL_STATE_PLL:
        ui8_six_step_hall_count = 2;

        if (q31_pll_abs_delta > (Q31_DEGREE * 20))
        {
            // ERROR
            //trigger_motor_error(MOTOR_STATE_PLL_ERROR)

            // PLL seems to be in trouble
            // fallback to extraploation
            ui8_extrapolation_hall_count = 10;
            pHallData->enum_hall_angle_state = HALL_STATE_EXTRAPOLATION;
            #if DISPLAY_TYPE == DISPLAY_TYPE_DEBUG
            info_printf("[PLL-ERROR]\n");
            #endif
        }

        if (pHallData->ui16_timertics > SIXSTEPTHRESHOLD_DOWN)
        {
            pHallData->enum_hall_angle_state = HALL_STATE_SIXSTEP;
            #if DISPLAY_TYPE == DISPLAY_TYPE_DEBUG
            info_printf("[PLL->6STEP]\n");
            #endif
        }
        break;

    default:
        break;
    }
}

void autodetect(MotorState_t *pMS)
{
    info_printf("\nAUTODETECT\n");
    uint8_t ui8_hall_state_old_autodetect = ui8_hall_state__;
    pMS->HallData.hall_angle_detect_flag = 1;
    // effects of hall_angle_detect_flag
    //  - q31_rotorposition_absolute is not updated in estimate_rotor_angle
    //  - runPIcontrol: does nothing and returns immediately (open loop control -> u_d is set in this function)

    pMS->HallData.q31_rotorposition_absolute = DEG_plus180;
    uint8_t zerocrossing = 0; // nulldurchgang
    q31_t diffangle = 0;
    //
    pMS->u_q = 0;
    pMS->u_d = 250;

    enable_pwm();
    //SET_BIT(TIM1->BDTR, TIM_BDTR_MOE);
    //ui8_pwm_enabled_flag = 1;
    //
    HAL_Delay(5);
    for (uint16_t i = 0; i < 1080; i++)
    {
        pMS->HallData.q31_rotorposition_absolute += 11930465; //drive motor in open loop with steps of 1°
        HAL_Delay(5);
        if (pMS->HallData.q31_rotorposition_absolute > -60 && pMS->HallData.q31_rotorposition_absolute < 300)
        {
            // the rotor is at zero degrees absolute position
            // ui8_hall_case -> the last recorded hall transition
            // zerocrossing -> the next expected hall transition
            // example: after the transition 64, the transition 45 should follow
            // diffangle: angle of the 'zerocrossing' hall event
            // the referece is zero at the transition 32 / 23 (this choice is arbitrary)
            switch (ui8_hall_case__) //12 cases for each transition from one stage to the next. 6x forward, 6x reverse
            {
            //6 cases for forward direction
            case 64:
                zerocrossing = 45;
                diffangle = DEG_plus180;
                break;
            case 45:
                zerocrossing = 51;
                diffangle = DEG_minus120;
                break;
            case 51:
                zerocrossing = 13;
                diffangle = DEG_minus60;
                break;
            case 13:
                zerocrossing = 32;
                diffangle = DEG_0;
                break;
            case 32:
                zerocrossing = 26;
                diffangle = DEG_plus60;
                break;
            case 26:
                zerocrossing = 64;
                diffangle = DEG_plus120;
                break;

            //6 cases for reverse direction
            case 46:
                zerocrossing = 62;
                diffangle = -DEG_plus60;
                break;
            case 62:
                zerocrossing = 23;
                diffangle = -DEG_0;
                break;
            case 23:
                zerocrossing = 31;
                diffangle = -DEG_minus60;
                break;
            case 31:
                zerocrossing = 15;
                diffangle = -DEG_minus120;
                break;
            case 15:
                zerocrossing = 54;
                diffangle = -DEG_plus180;
                break;
            case 54:
                zerocrossing = 46;
                diffangle = -DEG_plus120;
                break;

            } // end case
        }

        if (ui8_hall_state_old_autodetect != ui8_hall_state__)
        {
            // a hall transition ocured 
            ui8_hall_state_old_autodetect = ui8_hall_state__;

            if (ui8_hall_case__ == zerocrossing)
            {
                //q31_rotorposition_motor_specific = q31_rotorposition_absolute-diffangle-(1<<31);
                q31_rotorposition_motor_specific = pMS->HallData.q31_rotorposition_absolute - diffangle - DEG_plus180;
                info_printf("   ZEROCROSSING: angle: %d, hallstate:  %d, hallcase %d \n", q31_degree_to_degree(pMS->HallData.q31_rotorposition_absolute), ui8_hall_state__, ui8_hall_case__);
            }
            else
            {
                info_printf("angle: %d, hallstate:  %d, hallcase %d \n", q31_degree_to_degree(pMS->HallData.q31_rotorposition_absolute), ui8_hall_state__, ui8_hall_case__);
            }
        }
    }
    pMS->u_d = 0;
    disable_pwm();
    pMS->HallData.hall_angle_detect_flag = 0;

    //HAL_FLASH_Unlock();
    //EE_WriteVariable(EEPROM_POS_SPEC_ANGLE, q31_rotorposition_motor_specific>>16);
    //if(i8_recent_rotor_direction == 1){
    //	EE_WriteVariable(EEPROM_POS_HALL_ORDER, 1);
    //	i16_hall_order = 1;
    //}
    //else{
    //	EE_WriteVariable(EEPROM_POS_HALL_ORDER, -1);
    //	i16_hall_order = -1;
    //}
    //HAL_FLASH_Lock();

#if (DISPLAY_TYPE == DISPLAY_TYPE_DEBUG)
    info_printf("Motor specific angle:  %d (q31_degree), %d (degree), hall_order %d \n ", q31_rotorposition_motor_specific, q31_degree_to_degree(q31_rotorposition_motor_specific), i16_hall_order);
#endif

    HAL_Delay(5);
}
