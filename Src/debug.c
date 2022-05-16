
#include "debug.h"
#include "display_debug.h"
#include "varia.h"
#include "system.h"
#include "motor_control_v2.h"
#include "current_target.h"

extern uint8_t ui8_g_motor_error_state_hall_count; // from hall.c

uint8_t ui8_e_print_log_info_flag = 0;

// -------------------------------------------------------- fast log

uint8_t ui8_fast_log_state = 0;
#if defined(FAST_LOOP_LOG)
int16_t i16_fast_log_sample_buffer[FAST_LOG_N_DATA_STREAMS];

static int16_t i16_fast_log_buffer[FAST_LOG_N_DATA_STREAMS][FAST_LOG_N_SAMPLES];

static uint8_t ui8_fast_log_k = 0;

void fast_log_data()
{
    static uint8_t skip = 1;
    if(skip > 0)
    {
        skip = skip - 1;
    }

    if(ui8_fast_log_state == 1)
    {
        if(ui8_fast_log_k >= FAST_LOG_N_SAMPLES)
        {
            ui8_fast_log_state = 2;
            ui8_fast_log_k = 0;
            skip = 1;
        }
        else if(skip == 0)
        {
            for(uint8_t i = 0; i < FAST_LOG_N_DATA_STREAMS; ++i)
            {
                i16_fast_log_buffer[i][ui8_fast_log_k] = i16_fast_log_sample_buffer[i];
            }
            skip = FAST_LOG_SKIP;
            ++ui8_fast_log_k;
        }
    }
}

static void fast_log_write()
{
    if(ui8_fast_log_state == 2)
    {
        if(ui8_fast_log_k >= FAST_LOG_N_SAMPLES)
        {
            ui8_fast_log_state = 0;
            ui8_fast_log_k = 0;
            debug_printf("done sampling\n");
        }
        else
        {
#if FAST_LOG_N_DATA_STREAMS == 1
            debug_printf("%d\n", i16_fast_log_buffer[0][ui8_fast_log_k]);
#elif FAST_LOG_N_DATA_STREAMS == 2
            debug_printf("%d %d\n", i16_fast_log_buffer[0][ui8_fast_log_k], i16_fast_log_buffer[1][ui8_fast_log_k]);
#endif
            ++ui8_fast_log_k;
        }

    }
}

#endif
// -------------------------------------------------------- fast log

void debug_comm(MotorState_t *pMS)
{
#if (DISPLAY_TYPE == DISPLAY_TYPE_DEBUG && defined(FAST_LOOP_LOG))

    fast_log_write();

#elif (DISPLAY_TYPE == DISPLAY_TYPE_DEBUG)

    static uint8_t ui8_dbg_print_counter = 0;

    // print values for debugging

    if (ui8_dbg_print_counter == 0)
    {

        switch (pMS->ui8_dbg_log_value)
        {
        case 0:
            // system state
            //debug_printf("err  : %u\nhall : %u\ntemp : %u\nbatt :%u\n\n",
            //             get_motor_error_state(),
            //             pMS->HallData.enum_hall_angle_state,
            //             pMS->ADC_Data.ChipTemperatureData.enum_temperature_state,
            //             pMS->ADC_Data.BatteryData.enum_voltage_state);

            // debug_printf("err  : %u (%u)\n",
            //                 get_motor_error_state(), ui8_g_motor_error_state_hall_count);

            debug_printf("display: %d battery: %d\n", pMS->i32_power_Wx10_for_display / 10, pMS->ADC_Data.BatteryData.i32_battery_power_W_x10/10);
            break;
        case 1:
            // -------------------------------------- motor control
            if(ui8_e_print_log_info_flag && ui8_g_UART_TxCplt_flag)
            {
                debug_printf("[ctrl state] Umag Uangle\n");
                ui8_e_print_log_info_flag = 0;
            }
            else
            {
                print_motor_control_info();
            }
            break;
        case 2:
            // -------------------------------------- motor control 2
            //print_motor_control_info();
            if(ui8_e_print_log_info_flag && ui8_g_UART_TxCplt_flag)
            {
                debug_printf("ud uq U Psi_f [mV/s]\n");
                ui8_e_print_log_info_flag = 0;
            }
            else
            {
                int32_t psi_f = (pMS->i32_U_magnitude * pMS->ADC_Data.BatteryData.i32_battery_voltage_Vx10 * 100) / (3547 * pMS->HallData.i32_omega_el);
                debug_printf("%6d %6d %6d %6d\n", pMS->u_d, pMS->u_q, pMS->i32_U_magnitude, psi_f);
            }
            break;
        case 3:
            // -------------------------------------- battery voltage  + temperature
            if(ui8_e_print_log_info_flag && ui8_g_UART_TxCplt_flag)
            {
                debug_printf("V [Vx10] V uncorr [Vx10] Temp [deg] dist[km] time[min]\n");
                ui8_e_print_log_info_flag = 0;
            }
            else
            {
                debug_printf("%6d %6d %6d %6d %6d\n", pMS->ADC_Data.BatteryData.i32_battery_voltage_Vx10, 
                                                pMS->ADC_Data.BatteryData.i32_battery_voltage_uncorrected_Vx10, 
                                                pMS->ADC_Data.ChipTemperatureData.q31_temperature_degrees, 
                                                pMS->ADC_Data.BatteryData.ui8_remaining_distance_km,
                                                pMS->ADC_Data.BatteryData.ui16_remaining_time_min);
            }
            break;
        case 4:
            // -------------------------------------- current
            if(ui8_e_print_log_info_flag && ui8_g_UART_TxCplt_flag)
            {
                debug_printf("id [mA] iq [mA] Ib [mA]\n");
                ui8_e_print_log_info_flag = 0;
            }
            else
            {
                debug_printf("%6d %6d %6d\n", pMS->i32_i_d_mA, pMS->i32_i_q_mA, pMS->ADC_Data.BatteryData.i32_battery_current_mA);
            }
            break;
        case 5:
            // -------------------------------------- current target module
            if(ui8_e_print_log_info_flag && ui8_g_UART_TxCplt_flag)
            {
                debug_printf("[boost counter] active limits\n");
                ui8_e_print_log_info_flag = 0;
            }
            else
            {
                print_current_target_info();
            }
            break;
        case 6:
            // -------------------------------------- pedal data
            if(ui8_e_print_log_info_flag && ui8_g_UART_TxCplt_flag)
            {
                debug_printf("torque [Nmx10] rpm\n");
                ui8_e_print_log_info_flag = 0;
            }
            else
            {
                //debug_printf("%u %u\n", pMS->PedalData.uint32_torque_Nm_x10, pMS->PedalData.uint32_PAS_rpm);
                debug_printf("%u %u\n", 10 * pMS->PedalData.uint16_torque_adc / CAL_TORQUE, pMS->PedalData.uint32_PAS_rpm);
            }
            break;
        case 7:
            // -------------------------------------- hall module
            if(ui8_e_print_log_info_flag && ui8_g_UART_TxCplt_flag)
            {
                debug_printf("hall state | velocity [kmhx10]\n");
                ui8_e_print_log_info_flag = 0;
            }
            else
            {
                debug_printf("%u %u\n", pMS->HallData.enum_hall_angle_state, pMS->WheelSpeedData.uint32_SPEED_kmh_x10);
            }
            break;
        default:
            break;
        }

        // ui8_dbg_print_counter = 16;
        ui8_dbg_print_counter = 6;
    }
    else
    {
        --ui8_dbg_print_counter;
    }

#endif
}