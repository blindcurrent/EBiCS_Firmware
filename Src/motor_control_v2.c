
#include "stm32f1xx_hal.h"
#include "motor_control_v2.h"
#include "current_target.h"
#include "system.h"
#include "svpwm.h"
#include "display_debug.h"
#include "varia.h"

typedef enum
{
    IDLE = 0,
    MTPA = 1,
    RAMP_DOWN = 3
} motor_control_state_t;

extern uint8_t PI_flag;
extern uint16_t ui16_motor_init_state_timeout;
extern volatile uint8_t ui8_g_UART_TxCplt_flag;
extern IWDG_HandleTypeDef hiwdg;

static PI_control_t PIid;
static PI_control_t PIiq;
static PI_control_t PIangle;
    
static int32_t i32_U_magnitude, i32_U_angle;
static int32_t i32_u_d_temp, i32_u_q_temp;
static motor_control_state_t CONTROL_STATE = IDLE;


// motor_control_v2_utils
int32_t get_angle(int32_t xd, int32_t xq);
int32_t get_magnitude(int32_t xd, int32_t xq);
void get_dq_coordinates(int32_t i32_mag, int32_t i32_angle, int32_t *p_i32_xd, int32_t *p_i32_xq);

static void control_loop(int32_t i32_i_d_mA, int32_t i32_i_q_mA, int32_t i32_i_q_target_mA, MotorState_t *pMS);
static int32_t PI_control_v2(PI_control_t *PI_c);
static void compute_dynamic_switchtime_v2(MotorState_t *pMS);

void print_motor_control_info()
{
#if (DISPLAY_TYPE == DISPLAY_TYPE_DEBUG)
    debug_printf("[%d] %6d %6d\n", CONTROL_STATE, i32_U_magnitude, q31_degree_to_degree(i32_U_angle));
    //debug_printf("ctrl state: %d\nint: %d\nset: %d\nval: %d\n\n", CONTROL_STATE, PIangle.integral_part, PIangle.setpoint, PIangle.recent_value);
    //debug_printf("ctrl state: %d\nu_d: %d\nu_q: %d\nmag: %d\n\n", CONTROL_STATE, i32_u_d_temp, i32_u_q_temp, i32_U_magnitude);
#endif
}


void init_motor_control_v2(void)
{
    int32_t out_min, out_max;

// --------------------------------------------- PIid
#define I_FACTOR_I_D_ 1
#define P_FACTOR_I_D_ 0
#define SHIFT_ID_ 12

    PIid.gain_p = P_FACTOR_I_D_;
    PIid.gain_i = I_FACTOR_I_D_;
    out_min = -((_U_MAX / 2) << SHIFT_ID_);
    PIid.limit_output_min_shifted = out_min;
    PIid.limit_integral_min_shifted = out_min / I_FACTOR_I_D_;
    out_max = (_U_MAX / 2) << SHIFT_ID_;
    PIid.limit_output_max_shifted = out_max;
    PIid.limit_integral_max_shifted = out_max / I_FACTOR_I_D_;
    PIid.recent_value = 0;
    PIid.setpoint = 0;
    PIid.integral_part = 0;
    PIid.max_step_shifted = 4 << SHIFT_ID_;
    PIid.out_shifted = 0;
    PIid.shift = SHIFT_ID_;
    PIid.id = 0;

// ---------------------------------------------- PIiq
#define I_FACTOR_I_Q_ 10
#define P_FACTOR_I_Q_ 0
#define SHIFT_IQ_ 12

    PIiq.gain_p = P_FACTOR_I_Q_;
    PIiq.gain_i = I_FACTOR_I_Q_;
    PIiq.limit_output_min_shifted = 0;
    PIiq.limit_integral_min_shifted = 0;
    out_max = (_U_MAX << SHIFT_IQ_);
    PIiq.limit_output_max_shifted = out_max;
    PIiq.limit_integral_max_shifted = out_max / I_FACTOR_I_Q_;
    PIiq.recent_value = 0;
    PIiq.setpoint = 0;
    PIiq.integral_part = 0;
    PIiq.max_step_shifted = 4 << SHIFT_IQ_;
    PIiq.out_shifted = 0;
    PIiq.shift = SHIFT_IQ_;
    PIiq.id = 1;

// ---------------------------------------------- angle control
#define I_FACTOR_ANGLE_ 300
#define P_FACTOR_ANGLE_ 0
#define SHIFT_ANGLE_ 0

    PIangle.gain_p = P_FACTOR_ANGLE_;
    PIangle.gain_i = I_FACTOR_ANGLE_;
    out_min = (-Q31_DEGREE * 30) << SHIFT_ANGLE_;
    PIangle.limit_output_min_shifted = out_min;
    PIangle.limit_integral_min_shifted = out_min / I_FACTOR_ANGLE_;
    out_max = (Q31_DEGREE * 30) << SHIFT_ANGLE_;
    PIangle.limit_output_max_shifted = out_max;
    PIangle.limit_integral_max_shifted = out_max / I_FACTOR_ANGLE_;
    PIangle.recent_value = 0;
    PIangle.setpoint = 0;
    PIangle.integral_part = 0;
    PIangle.max_step_shifted = Q31_DEGREE << SHIFT_ANGLE_;
    PIangle.out_shifted = 0;
    PIangle.shift = SHIFT_ANGLE_;
    PIangle.id = 2;
}

static int32_t PI_control_v2(PI_control_t *PI_c)
{
    int32_t i32_delta, i32_p, i32_i, i32_out_shifted, i32_ret;
    // the entire PI control is done with shifted values
    //only the return value is shifted back

    i32_delta = PI_c->setpoint - PI_c->recent_value;

    // integrate error
    if ((PI_c->integral_part <= PI_c->limit_integral_max_shifted) && (PI_c->integral_part >= PI_c->limit_integral_min_shifted))
    {
        PI_c->integral_part += i32_delta;
    }
    if (PI_c->integral_part > PI_c->limit_integral_max_shifted)
    {
        PI_c->integral_part = PI_c->limit_integral_max_shifted;
    }
    if (PI_c->integral_part < PI_c->limit_integral_min_shifted)
    {
        PI_c->integral_part = PI_c->limit_integral_min_shifted;
    }

    i32_p = i32_delta * PI_c->gain_p;
    i32_i = PI_c->integral_part * PI_c->gain_i;

    i32_out_shifted = i32_p + i32_i;

    if (i32_out_shifted >= PI_c->limit_output_max_shifted)
    {
        i32_out_shifted = PI_c->limit_output_max_shifted;
    }

    if (i32_out_shifted <= PI_c->limit_output_min_shifted)
    {
        i32_out_shifted = PI_c->limit_output_min_shifted;
    }

    PI_c->out_shifted = i32_out_shifted;
    i32_ret = i32_out_shifted >> PI_c->shift;
    return i32_ret;
}

void run_motor_control_v2(MotorState_t *pMS)
{
    // -----------------------------------------
    // feed the dog
#ifdef ACTIVATE_WATCHDOG
    //if(MS.ui16_dbg_value2 == 0)   // trigger the watchdog
    {
        HAL_IWDG_Refresh(&hiwdg);
    }
#endif
    
    process_errors(pMS);
    
    if(pMS->HallData.hall_angle_detect_flag == 1)
    {
        return;
    }

#ifndef OPEN_LOOP_CONTROL
    
    // -----------------------------------------
    // CLOSED LOOP CONTROL
    // -----------------------------------------

    int32_t i32_i_d_mA = pMS->i32_i_d_mA;
    int32_t i32_i_q_mA = pMS->i32_i_q_mA;

    // ---------------------------------
    int32_t i_q_target_mA = compute_iq_target(pMS);

    // ---------------------------------
    //int32_t i_q_target_mA = 0;
    //if (pMS->ui8_go)
    //{
    //    if(pMS->ui16_dbg_value > 60) pMS->ui16_dbg_value = 60;
    //    i_q_target_mA = pMS->ui16_dbg_value * 100;
    //}
    // ---------------------------------

    control_loop(i32_i_d_mA, i32_i_q_mA, i_q_target_mA, pMS);

#else
    
    // -----------------------------------------
    // OPEN LOOP CONTROL
    // -----------------------------------------
    
    if(pMS->ui8_go && get_motor_error_state() == MOTOR_STATE_NORMAL)
    {
        pMS->u_q = pMS->ui16_dbg_value;
        pMS->u_d = 0;

        if(!is_pwm_enabled())
        {
            enable_pwm();
        }
    }
    else
    {
        pMS->u_q = 0;
        pMS->u_d = 0;
        if(is_pwm_enabled())
        {
            disable_pwm();
        }
    }


#endif

    PI_flag = 0;
}


static void control_loop(int32_t i32_i_d_mA, int32_t i32_i_q_mA, int32_t i32_i_q_target_mA, MotorState_t *pMS)
{

    int32_t i32_wheel_speed_kmh_x10 = pMS->WheelSpeedData.uint32_SPEED_kmh_x10;
    
    if (get_motor_error_state() != MOTOR_STATE_NORMAL)
    {
        CONTROL_STATE = IDLE;
    }
    
    if( (CONTROL_STATE == MTPA) )
    {
        if( (i32_wheel_speed_kmh_x10 < 20) && (ui16_motor_init_state_timeout == 0) && (pMS->u_q < 400) )
        {
            // turn off pwm at low speeds
            CONTROL_STATE = IDLE;
        }
    }
        
    int32_t i32_i_q_target_ABS = (i32_i_q_target_mA > 0) ? i32_i_q_target_mA : -i32_i_q_target_mA;


    if (CONTROL_STATE == IDLE)
    {

        // todo reset pi-structs
        PIiq.integral_part = 0;
        PIid.integral_part = 0;
        PIangle.integral_part = 0;

        if (is_pwm_enabled())
        {
            disable_pwm();
        }
        //
        if ((i32_i_q_target_ABS > 200) || (i32_wheel_speed_kmh_x10 > 100))
        {
            if (get_motor_error_state() == MOTOR_STATE_NORMAL)
            {
                if (i32_wheel_speed_kmh_x10 < 400)
                {
                    compute_dynamic_switchtime_v2(pMS);
                    enable_pwm_with_dynamic_switchtime();

                    ui16_motor_init_state_timeout = 16 * 3; // 3s timeout
                    CONTROL_STATE = MTPA;
                }
            }
        }

        i32_u_d_temp = 0;
        i32_u_q_temp = 0;
        i32_U_angle = 0;
        i32_U_magnitude = 0;
        pMS->u_d = 0;
        pMS->u_q = 0;
        pMS->i32_i_d_mA = 0;
        pMS->i32_i_q_mA = 0;
    }

    if (CONTROL_STATE == MTPA)
    {
        // torque control
        PIiq.setpoint = i32_i_q_target_mA;
        PIiq.recent_value = i32_i_q_mA;
        i32_u_q_temp = PI_control_v2(&PIiq);

        // flux control
        if(i32_i_q_target_ABS > 1000)
        {
            PIid.setpoint = 0;
            PIid.recent_value = i32_i_d_mA;
            i32_u_d_temp = PI_control_v2(&PIid);
        }
        else
        {
            PIid.setpoint = 0;
            PIid.recent_value = 0;
            PIid.integral_part = 0;
            i32_u_d_temp = 0;
        }
            
        i32_U_angle = get_angle(i32_u_d_temp, i32_u_q_temp);
        i32_U_magnitude = get_magnitude(i32_u_d_temp, i32_u_q_temp);

        if (i32_U_magnitude > _U_MAX)
        {
            i32_u_q_temp = i32_u_q_temp * _U_MAX / i32_U_magnitude;
            i32_u_d_temp = i32_u_d_temp * _U_MAX / i32_U_magnitude;
        }
    }

    pMS->u_d = i32_u_d_temp;
    pMS->u_q = i32_u_q_temp;
    pMS->i32_U_magnitude = i32_U_magnitude;
    pMS->i32_U_angle = i32_U_angle;
}

static void compute_dynamic_switchtime_v2(MotorState_t *pMS)
{
    uint32_t u_q = MOTOR_KV * _T / (pMS->ADC_Data.BatteryData.i32_battery_voltage_Vx10 * (pMS->HallData.uint32_tics_filtered) / 10);

    if (u_q > _U_MAX)
    {
        u_q = _U_MAX;
    }

    if (u_q < 200)
    {
        u_q = 0;
    }

    pMS->u_q = (q31_t) u_q;
    pMS->u_d = 0;
    PIiq.integral_part = (u_q << PIiq.shift) / PIiq.gain_i;
    PIid.integral_part = 0;
    
    //
#if DISPLAY_TYPE == DISPLAY_TYPE_DEBUG
    info_printf("enable dynamic pwm, u_q = %u\n", u_q);
#endif

    pMS->HallData.q31_rotorposition_absolute = pMS->HallData.q31_rotorposition_hall + (DEG_plus60 >> 1);
    compute_svpwm_switchtime(pMS, pMS->HallData.q31_rotorposition_absolute);
}

/*
void test_motor_control_v2(void)
{
    // *****************************************************
    // measuring cpu cycles on arm cortex m cpus
    // https://www.embeddedcomputing.com/technology/processing/measuring-code-execution-time-on-arm-cortex-m-mcus
    
    #define  ARM_CM_DEMCR      (*(uint32_t *)0xE000EDFC)
    #define  ARM_CM_DWT_CTRL   (*(uint32_t *)0xE0001000)
    #define  ARM_CM_DWT_CYCCNT (*(uint32_t *)0xE0001004)
    static uint32_t  start, stop, cycles;

    static uint8_t first_run = 1;
    if (first_run)
    {
        if (ARM_CM_DWT_CTRL != 0) {        // See if DWT is available
            ARM_CM_DEMCR      |= 1 << 24;  // Set bit 24
            ARM_CM_DWT_CYCCNT  = 0;
            ARM_CM_DWT_CTRL   |= 1 << 0;   // Set bit 0
        }
        first_run = 0;
    }

    // *****************************************************
    
    static uint8_t ui8_count = 0;

    // ------------------------- test division and multiplication
    //#define N_VAR_ 1
    //__disable_irq();
    //volatile int32_t i32_a = MAX_INT32;
    //volatile int32_t i32_b = 2344;
    //start = ARM_CM_DWT_CYCCNT;
    //// i32_a = i32_a * i32_b; // 5 cycles
    //i32_a = i32_a / i32_b; // 10-13 cycles
    //stop = ARM_CM_DWT_CYCCNT;
    //cycles = stop - start;
    //__enable_irq();
    //debug_printf("[%u]\n", cycles);


    // ------------------------- test arm_sin_cos_q31
    //#define N_VAR_ 7
    //__disable_irq();
    //int32_t i32_in_values[N_VAR_] = {0, MAX_INT32, MIN_INT32, DEG_plus60, DEG_minus60, 1073741824, -1073741824};
    //int32_t i32_in, i32_sin, i32_cos;
    //i32_in = i32_in_values[ui8_count];
    //start = ARM_CM_DWT_CYCCNT;
    //arm_sin_cos_q31(i32_in, &i32_sin, &i32_cos);
    //stop = ARM_CM_DWT_CYCCNT;
    //cycles = stop - start;
    //__enable_irq();
    //debug_printf("%u [%u] %d | s: %d   c: %d\n", ui8_count, cycles, i32_in, i32_sin, i32_cos);

    // ------------------------- get_dq_coordinates
    //#define N_VAR_ 5
    //__disable_irq();
    //int32_t i32_angles[N_VAR_] = {0, 1073741824, -1073741824, 357913941, -357913941};
    //int32_t i32_mag[N_VAR_] = {2000, 2000, 2000, 10000, 10000};
    //int32_t i32_xd, i32_xq;
    //start = ARM_CM_DWT_CYCCNT;
    //get_dq_coordinates(i32_mag[ui8_count], i32_angles[ui8_count], &i32_xd, &i32_xq);
    //stop = ARM_CM_DWT_CYCCNT;
    //cycles = stop - start;
    //__enable_irq();
    //debug_printf("%u [%u] %d %d | xd: %d   xq: %d\n", ui8_count, cycles, i32_mag[ui8_count], i32_angles[ui8_count], i32_xd, i32_xq);


    // ------------------------- arm_sqrt_q31(q31_in, p_mag);
    //#define N_VAR_ 5
    //__disable_irq();
    //int16_t i32_in[N_VAR_] = {0, 4, 64, 10000, 10000};
    //int32_t i32_out;
    //q31_t q31_out;
    //start = ARM_CM_DWT_CYCCNT;
    //arm_sqrt_q31(i32_in[ui8_count], &q31_out);
    //stop = ARM_CM_DWT_CYCCNT;
    //cycles = stop - start;
    //__enable_irq();
    //debug_printf("[%u] %d | %d\n", cycles, i32_in[ui8_count], q31_out);

    // ------------------------- get_magnitude
    //#define N_VAR_ 5
    //__disable_irq();
    //volatile int32_t i32_xd[N_VAR_] = {0, 0, 10000, -10000, -1000};
    //volatile int32_t i32_xq[N_VAR_] = {0, 10000, 10000, 10000, 2000};
    //int32_t i32_mag;
    //start = ARM_CM_DWT_CYCCNT;
    //i32_mag = get_magnitude(i32_xd[ui8_count], i32_xq[ui8_count]);
    //stop = ARM_CM_DWT_CYCCNT;
    //cycles = stop - start;
    //__enable_irq();
    //debug_printf("%u [%u] %d %d | mag: %d\n", ui8_count, cycles, i32_xd[ui8_count], i32_xq[ui8_count], i32_mag);
    
    // ------------------------- get_angle
    //#define N_VAR_ 5
    //__disable_irq();
    //volatile int32_t i32_xd[N_VAR_] = {0, 0, 10000, -10000, -1000};
    //volatile int32_t i32_xq[N_VAR_] = {0, 10000, 10000, 10000, 2000};
    //int32_t i32_angle;
    //start = ARM_CM_DWT_CYCCNT;
    //i32_angle = get_angle(i32_xd[ui8_count], i32_xq[ui8_count]);
    ////i32_mag = (int32_t) isqrt(i32_xd[ui8_count] * i32_xd[ui8_count] + i32_xq[ui8_count] * i32_xq[ui8_count]);   // isqrt takes slightly more cycles than arm_sqrt_q31
    //stop = ARM_CM_DWT_CYCCNT;
    //cycles = stop - start;
    //__enable_irq();
    //debug_printf("%u [%u] %d %d | angle: %d (%d)\n", ui8_count, cycles, i32_xd[ui8_count], i32_xq[ui8_count], i32_angle, q31_degree_to_degree(i32_angle));
    
    // ------------------------- arm_clarke_q31
    //#define N_VAR_ 5
    //__disable_irq();
    //volatile int32_t i32_x1[N_VAR_] = {0, 1000, 0, 0, -1000};
    //volatile int32_t i32_x2[N_VAR_] = {0, 0, 10000, -100, 2000};
    //volatile int32_t i32_x3[N_VAR_] = {0, -1000, -10000, 100, -1000};
    //int32_t i32_x_alpha, i32_x_beta;
    //start = ARM_CM_DWT_CYCCNT;
    //arm_clarke_q31(i32_x1[ui8_count], i32_x2[ui8_count], &i32_x_alpha, &i32_x_beta);
    //stop = ARM_CM_DWT_CYCCNT;
    //cycles = stop - start;
    //__enable_irq();
    //debug_printf("%u [%u] %d %d %d | %d %d\n", ui8_count, cycles, i32_x1[ui8_count], i32_x2[ui8_count], i32_x3[ui8_count], i32_x_alpha, i32_x_beta);
    
    // ------------------------- arm_park_q31
    //#define N_VAR_ 5
    //__disable_irq();
    //volatile int32_t i32_x_alpha[N_VAR_] = {0, 1000, 0, 0, -1000};
    //volatile int32_t i32_x_beta[N_VAR_] = {0, 0, 10000, -100, 2000};
    //volatile int32_t i32_angle[N_VAR_] = {0, 0, 536870912, 1908874353, -596523235};
    //int32_t i32_x_d, i32_x_q, i32_sin, i32_cos;
    //start = ARM_CM_DWT_CYCCNT;
    //arm_sin_cos_q31(i32_angle[ui8_count], &i32_sin, &i32_cos);
    //arm_park_q31(i32_x_alpha[ui8_count], i32_x_beta[ui8_count], &i32_x_d, &i32_x_q, i32_sin, i32_cos); 
    //stop = ARM_CM_DWT_CYCCNT;
    //cycles = stop - start;
    //__enable_irq();
    //debug_printf("%u [%u] %d %d %d | %d %d\n", ui8_count, cycles, i32_x_alpha[ui8_count], i32_x_beta[ui8_count], i32_angle[ui8_count], i32_x_d, i32_x_q);
    
    // ------------------------- arm_inv_park_q31
    #define N_VAR_ 5
    __disable_irq();
    volatile int32_t i32_x_d[N_VAR_] = {0, 1000, 1000, 1000, -500};
    volatile int32_t i32_x_q[N_VAR_] = {0, 0, 0, 0, 2000};
    volatile int32_t i32_angle[N_VAR_] = {0, 0, 1073741824, -1073741824, -596523235};
    int32_t i32_x_alpha, i32_x_beta, i32_sin, i32_cos;
    start = ARM_CM_DWT_CYCCNT;
    arm_sin_cos_q31(i32_angle[ui8_count], &i32_sin, &i32_cos);
    i32_sin = 0;
    i32_cos = MIN_INT32;
    arm_inv_park_q31(i32_x_d[ui8_count], i32_x_q[ui8_count], &i32_x_alpha, &i32_x_beta, i32_sin, i32_cos); 
    stop = ARM_CM_DWT_CYCCNT;
    cycles = stop - start;
    __enable_irq();
    //debug_printf("%u [%u] %d %d %d sin: %d cos: %d | %d %d\n", ui8_count, cycles, i32_x_d[ui8_count], i32_x_q[ui8_count], i32_angle[ui8_count], i32_sin, i32_cos, i32_x_alpha, i32_x_beta);
    i32_sin = MIN_INT32 >> 31;
    debug_printf("MIN_INT32 >> 31 = %d\n", i32_sin);

    ++ui8_count;
    if(ui8_count >= N_VAR_) 
    {
        ui8_count = 0;
    }
}
*/