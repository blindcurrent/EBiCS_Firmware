#include "current_target.h"
#include "display_debug.h"
#include "varia.h"
#include "system.h"

static int32_t compute_iq_target_motoring(void);

static int32_t get_max_power_motoring(void);
static int32_t get_max_battery_current_motoring(void);
static int32_t get_max_phase_current_motoring(void);
static void limit_iq_target_motoring(int32_t *p_i32_iq_target_mA);

static int32_t compute_iq_target_recuperation(void);
static int32_t get_max_phase_current_recuperation(void);
static void limit_iq_target_recuperation(int32_t *p_i32_iq_target_mA);

static int32_t compute_iq_target_walk_assist(void);

// -------------------------------------
// static variables - avoid excessive parameter lists
    
static int32_t i32_omega_el;
static int32_t i32_pedal_torque_Nm_x10;
static int32_t i32_pas_omega_x10;
static int32_t i32_wheel_speed_kmh_x10;
static int32_t i32_id_mA;
static int32_t i32_u_d;
static int32_t i32_u_q;
static int32_t i32_battery_voltage_V_x10;
static int32_t i32_battery_voltage_uncorrected_Vx10;
static int32_t i32_battery_power_W_x10;
static int8_t  i8_assist_level;
static voltage_state_t enum_voltage_state;
static temperature_state_t enum_temperature_state;

// limit flags
static uint8_t ui8_battery_current_limit_motoring_active = 0;
static uint8_t ui8_phase_current_limit_motoring_active = 0;
static uint8_t ui8_power_limit_motoring_active = 0;
//
static uint8_t ui8_phase_current_limit_recuperation_active = 0;
static uint8_t ui8_voltage_limit_recuperation_active = 0;

// other static variables
static uint8_t ui8_motoring = 0;
static uint8_t ui8_recuperation = 0;
static uint8_t ui8_idle_flag = 0;
static int32_t i32_power_Wx10_for_display_temp = 0;

// -------------------------------------
extern uint16_t ui16_boost_counter;
// -------------------------------------

void print_current_target_info()
{
#if (DISPLAY_TYPE == DISPLAY_TYPE_DEBUG)
    if(ui8_motoring)
    {
        char power_str[4] = "";
        char i_batt_str[4] = "";
        char i_ph_str[4] = "";
        if(ui8_power_limit_motoring_active)
            strcpy(power_str, "P");
        if(ui8_battery_current_limit_motoring_active)
            strcpy(i_batt_str, "Ib");
        if(ui8_phase_current_limit_motoring_active)
            strcpy(i_ph_str, "Iph");
        debug_printf("[%d] motoring limits: %s %s %s\n", ui16_boost_counter, power_str, i_batt_str, i_ph_str);        
    }
    if(ui8_recuperation)
    {
        char v_str[4] = "";
        char i_ph_str[4] = "";
        if(ui8_voltage_limit_recuperation_active)
            strcpy(v_str, "V");
        if(ui8_phase_current_limit_recuperation_active)
            strcpy(i_ph_str, "Iph");
        debug_printf("[%d] regen limits: %s %s\n", ui16_boost_counter, v_str, i_ph_str);        

    }
#endif
}

static int32_t power_assist_factor_x10(int8_t level)
{
    int32_t factor_x10 = 0;
    switch (level)
    {
    case 0:
        factor_x10 = 0;
        break;
    case 1:
        factor_x10 = 10;
        break;
    case 2:
        factor_x10 = 15;
        break;
    case 3:
        factor_x10 = 20;
        break;
    case 4:
        factor_x10 = 25;
        break;
    case 5:
        factor_x10 = 30;
        break;
    default:
        factor_x10 = 0;
        break;
    }
    return factor_x10;
}

static int32_t torque_assist_factor_x10(int8_t level)
{
    int32_t factor_x10 = 0;
    switch (level)
    {
    case 0:
        factor_x10 = 0;
        break;
    case 1:
        factor_x10 = 5;
        break;
    case 2:
        factor_x10 = 10;
        break;
    case 3:
        factor_x10 = 15;
        break;
    case 4:
        factor_x10 = 20;
        break;
    case 5:
        factor_x10 = 25;
        break;
    default:
        factor_x10 = 0;
        break;
    }
    return factor_x10;
}

/**
   * @brief Computes the target quadrature current component iq.
   * @param[in]  pMS
   * @return i32_iq_target_mA 
   */
int32_t compute_iq_target(MotorState_t *pMS)
{
    static int32_t i32_iq_target_sum = 0;
    static int32_t i32_power_for_display_sum = 0;
    int32_t i32_iq_target_mA = 0;
    static uint16_t print_counter = 0;
    
    i8_assist_level = 0;

    if (pMS->i8_assist_level > 0 && pMS->i8_assist_level <= 5)
    {
        ui8_motoring = 1;
        i8_assist_level = pMS->i8_assist_level;
    }
    else
    {
        ui8_motoring = 0;
    }

    if (pMS->i8_assist_level < 0 && pMS->i8_assist_level >= -5)
    {
        ui8_recuperation = 1;
        i8_assist_level = pMS->i8_assist_level;
    }
    else
    {
        ui8_recuperation = 0;
    }

    // update static variables
    i32_omega_el = limit_int32(1, 50000, pMS->HallData.i32_omega_el);
    i32_pedal_torque_Nm_x10 = limit_uint32(0, 2000, pMS->PedalData.uint32_torque_Nm_x10);
    i32_pas_omega_x10 = limit_uint32(0, 200, pMS->PedalData.uint32_PAS_omega_x10);
    i32_wheel_speed_kmh_x10 = limit_uint32(1, 1000, pMS->WheelSpeedData.uint32_SPEED_kmh_x10);
    i32_id_mA = pMS->i32_i_d_mA;
    i32_u_d = pMS->u_d;
    i32_u_q = pMS->u_q;
    i32_battery_voltage_V_x10 = pMS->ADC_Data.BatteryData.i32_battery_voltage_Vx10;
    i32_battery_voltage_uncorrected_Vx10 = pMS->ADC_Data.BatteryData.i32_battery_voltage_uncorrected_Vx10;
    i32_battery_power_W_x10 = pMS->ADC_Data.BatteryData.i32_battery_power_W_x10;
    enum_voltage_state = pMS->ADC_Data.BatteryData.enum_voltage_state;
    enum_temperature_state = pMS->ADC_Data.ChipTemperatureData.enum_temperature_state;

    if(pMS->ui8_walk_assist)
    {
        i32_iq_target_mA = compute_iq_target_walk_assist();
    }
    else if (ui8_motoring)
    {
        i32_iq_target_mA = compute_iq_target_motoring();

        limit_iq_target_motoring(&i32_iq_target_mA);
        if(pMS->HallData.enum_hall_angle_state != 2)
        {
            if(i32_iq_target_mA > 15000)
            {
                // rauhes anfahren verhindern
                i32_iq_target_mA = 15000;
            }
        }
    }
    else if (ui8_recuperation)
    {
        i32_iq_target_mA = compute_iq_target_recuperation();
        limit_iq_target_recuperation(&i32_iq_target_mA);
    }
    else
    {
        i32_iq_target_mA = 0;
        i32_power_Wx10_for_display_temp = 0;
        i32_power_for_display_sum = 0;
    }

    if(i32_iq_target_mA == 0)
    {
        ui8_idle_flag = 1;
        ui16_boost_counter = BOOST_TIME;
    }
    else
    {
        ui8_idle_flag = 0;
    }
    
    // some filtering of the target value for a smoother ride
    i32_iq_target_sum -= (i32_iq_target_sum >> 6);
    i32_iq_target_sum += i32_iq_target_mA;
    i32_iq_target_mA = i32_iq_target_sum >> 6;

    i32_power_for_display_sum -= (i32_power_for_display_sum >> 6);
    i32_power_for_display_sum += i32_power_Wx10_for_display_temp;
    pMS->i32_power_Wx10_for_display = i32_power_for_display_sum >> 6;

    
    if (print_counter >= 250)
    {
        //debug_printf("%d %d\n", i32_wheel_speed_kmh_x10, i32_iq_target_mA);
        //debug_printf("%d %u\n", i32_wheel_speed_kmh_x10, pMS->WheelSpeedData.uint32_SPEED_kmh_x10);
        print_counter = 0;
    }
    else
    {
        ++print_counter;
    }
    
    //i32_iq_target_mA = 0;

    //if(ui8_motoring && is_pwm_enabled())
    //{
    //    if(i32_iq_target_mA < 50)
    //    {
    //        i32_iq_target_mA = 50;
    //    }
    //}

    return i32_iq_target_mA;
}

/**
   * @brief Sub-function of compute_iq_target.
   * @param[in]     i32_wheel_speed_kmh_x10 (static)
   * @param[in]     i8_assist_level (static)
   * @param[out]    i32_power_Wx10_for_display_temp (static)
   * @return        i32_iq_target_mA 
   */
static int32_t compute_iq_target_walk_assist()
{
    int32_t i32_iq_target_mA = 0;
    if(i8_assist_level >= 3)
        i32_iq_target_mA = 20000;
    else if(i8_assist_level == 2)
        i32_iq_target_mA = 15000;
    else
        i32_iq_target_mA = 10000;

    i32_iq_target_mA = map(i32_wheel_speed_kmh_x10, 45, 55, i32_iq_target_mA, 0);

    i32_power_Wx10_for_display_temp = i32_battery_power_W_x10;

    return i32_iq_target_mA;
}

/**
   * @brief Sub-function of compute_iq_target.
   * Computes the target iq for motoring operation. 
   * There is a torque based formulation for low speeds and a power based formulation for high speeds.
   * In the medium speed range, the two targets are blended.
   * @param[in]     i32_omega_el (static)
   * @param[in]     i32_pedal_torque_Nm_x10 (static)
   * @param[in]     i32_pas_omega_x10 (static)
   * @param[in]     i32_wheel_speed_kmh_x10 (static)
   * @param[in]     i8_assist_level (static)
   * @param[out]    i32_power_Wx10_for_display_temp
   * @return        i32_iq_target_mA 
   */
static int32_t compute_iq_target_motoring()
{
    //static uint8_t print_counter = 0;

    // torque target
    // Tel = (3 / 2) * TRANSMISSION_RATIO * (PSI_F * i_q)
    // i_q = 2 * Tel / (3 * TRANSMISSION_RATIO * PSI_F)
    int32_t i32_iq_torque_mA = (((2 * i32_pedal_torque_Nm_x10) << 10 ) / (3 * TRANSMISSION_RATIO * PSI_F_SHIFT10)) * 10 * torque_assist_factor_x10(i8_assist_level);

    // power target
    int32_t i32_target_power_mW = i32_pedal_torque_Nm_x10 * i32_pas_omega_x10 * power_assist_factor_x10(i8_assist_level);
    
    // todo max_power = f(assist_level)
    int32_t i32_max_power_mW = get_max_power_motoring();
    //
    if (i32_target_power_mW > i32_max_power_mW)
    {
        i32_target_power_mW = i32_max_power_mW;
        ui8_power_limit_motoring_active = 1;
    }
    else
    {
        ui8_power_limit_motoring_active = 0;
    }

    // Pel = (3 / 2) * (omega_el * PSI_F * i_q)
    // i_q = 2 * Pel / (3 * omega_el * PSI_F)
    int32_t i32_iq_power_mA = ((2 * i32_target_power_mW) << 10) / (3 * i32_omega_el * PSI_F_SHIFT10);

    // blend
    int32_t i32_iq_target_mA;
    const int32_t va = 110;
    const int32_t vb = 200;
    const int32_t delta = vb - va;

    if (i32_wheel_speed_kmh_x10 < va)
    {
        i32_iq_target_mA = i32_iq_torque_mA;
        i32_power_Wx10_for_display_temp = i32_battery_power_W_x10;
    }
    else if (i32_wheel_speed_kmh_x10 < vb)
    {
        int32_t alpha = i32_wheel_speed_kmh_x10 - va;
        i32_iq_target_mA = (alpha * i32_iq_power_mA + (delta - alpha) * i32_iq_torque_mA) / delta;
        i32_power_Wx10_for_display_temp = ( (alpha * (i32_target_power_mW / 100)) + (delta - alpha) * i32_battery_power_W_x10 ) / delta;
    }
    else
    {
        i32_iq_target_mA = i32_iq_power_mA;
        i32_power_Wx10_for_display_temp = i32_target_power_mW / 100;
    }

    if(i32_power_Wx10_for_display_temp > 9500)
    {
        i32_power_Wx10_for_display_temp = 9500;
    }

    //if (print_counter >= 4)
    //{
    //    debug_printf("%d %d %d\n", i32_iq_t_mA, i32_iq_p_mA, i32_iq_target_mA);
    //    print_counter = 0;
    //}
    //else
    //{
    //    ++print_counter;
    //}

    return i32_iq_target_mA;
}

static int32_t get_max_power_motoring()
{
    // assist level                               1       2       3       4       5 
    static int32_t MOTOR_POWER_MAX_ARRAY[6] = {0, 200000, 400000, 600000, 800000, 900000}; // in mW
    int32_t i32_ret = MOTOR_POWER_MAX_ARRAY[i8_assist_level];
    if( (enum_voltage_state == VOLTAGE_STATE_NORMAL) && (enum_temperature_state == TEMP_STATE_NOMRAL) )
    {
        i32_ret += MOTOR_POWER_BOOST_DELTA_mW * ui16_boost_counter / BOOST_TIME;
    }
    else
    {
        if(i32_ret > MOTOR_POWER_MAX_REDUCED_mW)
        {
            i32_ret = MOTOR_POWER_MAX_REDUCED_mW;
        }
    }
    return i32_ret;
}

static int32_t get_max_battery_current_motoring()
{
    int32_t i32_ret;
    if( (enum_voltage_state == VOLTAGE_STATE_NORMAL) && (enum_temperature_state == TEMP_STATE_NOMRAL) )
    {
        i32_ret = BATTERY_CURRENT_MAX_mA;
        i32_ret += BATTERY_CURRENT_BOOST_DELTA_mA * ui16_boost_counter / BOOST_TIME;
    }
    else
    {
        i32_ret = BATTERY_CURRENT_MAX_REDUCED_mA;
    }
    return i32_ret;
}

static int32_t get_max_phase_current_motoring()
{
    int32_t i32_ret;
    if( (enum_voltage_state == VOLTAGE_STATE_NORMAL) && (enum_temperature_state == TEMP_STATE_NOMRAL) )
    {
        i32_ret = PHASE_CURRENT_MAX_mA;
        i32_ret += PHASE_CURRENT_BOOST_DELTA_mA * ui16_boost_counter / BOOST_TIME;
    }
    else
    {
        i32_ret = PHASE_CURRENT_MAX_REDUCED_mA;
    }
    return i32_ret;
}

/**
   * @brief Sub-function of compute_iq_target.
   * Limits the target iq for motoring operation. 
   * @param[out]    p_i32_iq_target_mA
   * @param[in]     i32_id_mA (static)
   * @param[in]     i32_u_d (static)
   * @param[in]     i32_u_q (static)
   * @param[in]     i32_wheel_speed_kmh_x10 (static)
   */
static void limit_iq_target_motoring(int32_t *p_i32_iq_target_mA)
{
    static int32_t i32_iq_max_B_cumulated_Ax10;

    // ---------------------------- limit battery current
    const uint8_t ui8_ibatt_lim_shift = 7;
    int32_t IB_MAX_mA = get_max_battery_current_motoring();
    if(i32_u_q > 100)
    {
        int32_t iq_max_B = (IB_MAX_mA * 2365 - i32_id_mA * i32_u_d) / (i32_u_q * 100); //  in A x10
        i32_iq_max_B_cumulated_Ax10 -= i32_iq_max_B_cumulated_Ax10 >> ui8_ibatt_lim_shift;
        i32_iq_max_B_cumulated_Ax10 += iq_max_B;
        iq_max_B = (i32_iq_max_B_cumulated_Ax10 >> ui8_ibatt_lim_shift) * 100;
        if (*p_i32_iq_target_mA > iq_max_B)
        {
            *p_i32_iq_target_mA = iq_max_B;
            ui8_battery_current_limit_motoring_active = 1;
        }
        else
        {
            ui8_battery_current_limit_motoring_active = 0;
        }
    }
    else
    {
        i32_iq_max_B_cumulated_Ax10 = (IB_MAX_mA / 100) << ui8_ibatt_lim_shift;
    }

    // ---------------------------- limit phase current
    int32_t i32_Iph_max_mA = get_max_phase_current_motoring();
    if (*p_i32_iq_target_mA > i32_Iph_max_mA)
    {
        *p_i32_iq_target_mA = i32_Iph_max_mA;
        ui8_phase_current_limit_motoring_active = 1;
    }
    else
    {
        ui8_phase_current_limit_motoring_active = 0;
    }

    if (*p_i32_iq_target_mA < 0)
    {
        *p_i32_iq_target_mA = 0;
    }

    // ---------------------------- limit velocity

    int32_t i32_speedlimit_min = SPEEDLIMIT_KMH_X10;
    int32_t i32_speedlimit_max = SPEEDLIMIT_KMH_X10 + 20;
    if(i32_wheel_speed_kmh_x10 > i32_speedlimit_min)
    {
        *p_i32_iq_target_mA = map(i32_wheel_speed_kmh_x10, i32_speedlimit_min, i32_speedlimit_max, *p_i32_iq_target_mA, 0);
    }
}

/**
   * @brief Sub-function of compute_iq_target.
   * Computes the target iq for recuperation operation. 
   * The recuperation current is power based.
   * Below a certain minimum speed the recuperation target current is set to zero.
   * @param[in]     i32_wheel_speed_kmh_x10 (static)
   * @param[in]     i32_omega_el (static)
   * @param[in]     i8_assist_level (static)
   * @param[out]    i32_power_Wx10_for_display_temp
   * @return        i32_iq_target_mA 
   */
static int32_t compute_iq_target_recuperation()
{
    int32_t i32_iq_target_mA, i32_iq_power_mA, i32_iq_torque_mA;

    // omega_el = v_kmh / (3.6 * U) * 2pi * p = v_kmh * 18.75
    // T =  (3/2) * p * psi_f * i_q
    // P =  (3/2) * omega_el * psi_f * i_q

    // v_kmh = 16 -> omega_el = 300 (middle of blending)
    // with psi_f = 0.028
    // iq = -10000 mA, -> P = -126000 mW
    // iq = -17000 mA, -> P = -214200 mW
    // iq = -25000 mA, -> P = -315000 mW

    // ---------------------------------------- power based current target for high speed
    int32_t i32_target_power_mW = 0;
    switch(i8_assist_level)
    {
        case -5:
            i32_target_power_mW = -600000;
            break;
        case -4:
            i32_target_power_mW = -600000;
            break;
        case -3:
            i32_target_power_mW = -600000;
            break;
        case -2:
            i32_target_power_mW = -450000;
            break;
        case -1:
            i32_target_power_mW = -250000;
            break;
        case 0:
            i32_target_power_mW = 0;
            break;
        default:
            i32_target_power_mW = 0;
            break;
    }

    // numerator close to i32 overflow
    i32_iq_power_mA = ((2 * i32_target_power_mW) << 10) / (3 * i32_omega_el * PSI_F_SHIFT10);
    
    // ---------------------------------------- torque based current target for low speed
    
    switch(i8_assist_level)
    {
        case -5:
            i32_iq_torque_mA = -30000;
            break;
        case -4:
            i32_iq_torque_mA = -30000;
            break;
        case -3:
            i32_iq_torque_mA = -30000;
            break;
        case -2:
            i32_iq_torque_mA = -18000;
            break;
        case -1:
            i32_iq_torque_mA = -10000;
            break;
        case 0:
            i32_iq_torque_mA = 0;
            break;
        default:
            i32_iq_torque_mA = 0;
            break;
    }
    
    const int32_t va = 120;
    const int32_t vb = 200;
    const int32_t delta = vb - va;

    if (i32_wheel_speed_kmh_x10 < va)
    {
        i32_iq_target_mA = i32_iq_torque_mA;
    }
    else if (i32_wheel_speed_kmh_x10 < vb)
    {
        int32_t alpha = i32_wheel_speed_kmh_x10 - va;
        i32_iq_target_mA = (alpha * i32_iq_power_mA + (delta - alpha) * i32_iq_torque_mA) / delta;
    }
    else
    {
        i32_iq_target_mA = i32_iq_power_mA;
    }

    i32_power_Wx10_for_display_temp = i32_battery_power_W_x10;

    return i32_iq_target_mA;


}

static int32_t get_max_phase_current_recuperation()
{
    return REGEN_PHASE_CURRENT_MAX_mA;
}

/**
   * @brief Sub-function of compute_iq_target.
   * Limits the target iq for recuperation operation. 
   * @param[in]     i32_wheel_speed_kmh_x10 (static)
   * @param[in]     i32_battery_voltage_V_x10 (static)
   * @param[out]    p_i32_iq_target_mA
   */
static void limit_iq_target_recuperation(int32_t *p_i32_iq_target_mA)
{
    int32_t i32_iq_target_mA_temp = *p_i32_iq_target_mA;
    
    // ---------------------------- limit voltage
    if(i32_battery_voltage_uncorrected_Vx10 > 530)
    {
        i32_iq_target_mA_temp = map(i32_battery_voltage_uncorrected_Vx10, 530, 540, i32_iq_target_mA_temp, 0);
        ui8_voltage_limit_recuperation_active = 1;
    }
    else
    {
        ui8_voltage_limit_recuperation_active = 0;
    }
    
    // ---------------------------- limit velocity
    // no regeneration at very low velocities
    i32_iq_target_mA_temp = map(i32_wheel_speed_kmh_x10, 35, 55, 0, i32_iq_target_mA_temp);
    // no regeneration at very high velocities
    i32_iq_target_mA_temp = map(i32_wheel_speed_kmh_x10, 400, 450, i32_iq_target_mA_temp, 0);
    
    
    // ---------------------------- limit phase current
    int32_t i32_Iph_max_mA = get_max_phase_current_recuperation();
    if(i32_iq_target_mA_temp < i32_Iph_max_mA)
    {
        i32_iq_target_mA_temp = i32_Iph_max_mA;
        ui8_phase_current_limit_recuperation_active = 1;
    }
    else
    {
        ui8_phase_current_limit_recuperation_active = 0;
    }
    //
    if( i32_iq_target_mA_temp > 0)
    {
        i32_iq_target_mA_temp = 0;
    }

    *p_i32_iq_target_mA = i32_iq_target_mA_temp;
}
