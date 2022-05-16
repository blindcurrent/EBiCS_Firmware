
#include "stm32f1xx_hal.h"

#include "phase_current.h"
#include "debug.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
	
static uint8_t  char_dyn_adc_state;
static uint8_t  char_dyn_adc_state_old=1;

extern uint8_t PI_flag;

extern uint16_t ui16_ph1_offset;
extern uint16_t ui16_ph2_offset;
extern uint16_t ui16_ph3_offset;


static void dyn_adc_state();
static void set_inj_channel(uint8_t state);



/**
   * @brief Reading phase currents from adc.
   * @param[out] i32_ph1_current
   * @param[out] i32_ph2_current
   */
void sample_phase_currents(int32_t *i32_ph1_current, int32_t *i32_ph2_current)
{
    int32_t temp1, temp2;

#ifdef DISABLE_DYNAMIC_ADC
    i16_ph1_current = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    i16_ph2_current = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
#else

    switch (char_dyn_adc_state) //read in according to state
    {
    case 1: //Phase C at high dutycycles, read from A+B directly
    {
        temp1 = (int32_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        *i32_ph1_current = temp1;

        temp2 = (int32_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
        *i32_ph2_current = temp2;
    }
    break;
    case 2: //Phase A at high dutycycles, read from B+C (A = -B -C)
    {

        temp2 = (int32_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
        *i32_ph2_current = temp2;

        temp1 = (int32_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        *i32_ph1_current = -(temp2 + temp1);
    }
    break;
    case 3: //Phase B at high dutycycles, read from A+C (B=-A-C)
    {
        temp1 = (int32_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        *i32_ph1_current = temp1;
        temp2 = (int32_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
        *i32_ph2_current = -(temp1 + temp2);
    }
    break;

    case 0: //timeslot too small for ADC
    {
        //do nothing
    }
    break;

    } // end case
#endif
}


/**
   * @brief Compute d-q space vector coordinates of phase currents.
   * Always 32 values are accumulated. After 32 readings PI_flag is set to one 
   * which triggers the main control loop which runs at (16'000/32 = 500Hz)
   * @param[in] i32_ph1_current
   * @param[in] i32_ph2_current
   * @param[in] q31_theta
   * @param[out] id (via MS)
   * @param[out] iq (via MS)
   * @param[out] PI_flag
   */

void transform_currents(int32_t i32_ph1_current, int32_t i32_ph2_current, q31_t q31_theta, MotorState_t* MS)
{

    // passing i32 variables to arm_clarke_q31 doesnt work
    // need to cast to i16 - do not understand this
    int16_t i16_ph1_temp = (int16_t) i32_ph1_current;
    int16_t i16_ph2_temp = (int16_t) i32_ph2_current;

    static uint8_t ui8_foc_counter = 0;
    static q31_t q31_i_d_sum = 0;
    static q31_t q31_i_q_sum = 0;

    q31_t q31_i_alpha = 0;
    q31_t q31_i_beta = 0;
    q31_t q31_i_d = 0;
    q31_t q31_i_q = 0;

    q31_t sinevalue = 0, cosinevalue = 0;

    // Clark transformation
    arm_clarke_q31((q31_t)i16_ph1_temp, (q31_t)i16_ph2_temp, &q31_i_alpha, &q31_i_beta);

    // Park transformation
    arm_sin_cos_q31(q31_theta, &sinevalue, &cosinevalue);
    arm_park_q31(q31_i_alpha, q31_i_beta, &q31_i_d, &q31_i_q, sinevalue, cosinevalue);
        
    //i16_fast_log_sample_buffer[0] = q31_i_d;
    //i16_fast_log_sample_buffer[1] = q31_i_q;
    //fast_log_data();

    q31_i_d_sum += q31_i_d;
    q31_i_q_sum += q31_i_q;

    if (ui8_foc_counter == 31)
    {
        MS->i32_i_d_mA = (q31_i_d_sum * CAL_I) / 32;
        MS->i32_i_q_mA = (q31_i_q_sum * CAL_I) / 32;
        q31_i_d_sum = 0;
        q31_i_q_sum = 0;
        ui8_foc_counter = 0;
        PI_flag = 1;
    }
    
    ++ui8_foc_counter;
}



void dynamic_phase_current_sampling()
{

#ifndef DISABLE_DYNAMIC_ADC
	//get the Phase with highest duty cycle for dynamic phase current reading
	dyn_adc_state();
	//set the according injected channels to read current at Low-Side active time

	if (char_dyn_adc_state != char_dyn_adc_state_old)
    {
	    set_inj_channel(char_dyn_adc_state);
        char_dyn_adc_state_old = char_dyn_adc_state;
	}
#endif

}


/**
   * @brief Determines the sampling window for the phase current reading.
   * @param[out] TIM1->CCR4
   * @param[out] char_dyn_adc_state
   */
void dyn_adc_state()
{
    uint16_t tph1N = _T - switchtime[0];
    uint16_t tph2N = _T - switchtime[1];
    uint16_t tph3N = _T - switchtime[2];


	if (switchtime[2] > switchtime[0] && switchtime[2] > switchtime[1])
    {
		char_dyn_adc_state = 1; // 180 - 300 degrees -> phase 3 at high duty cycles
        // measure phase 1 & 2
		if(switchtime[2] > 1500)
        {
            if( (tph1N > (2 * (tph3N + PWM_DEAD_TIME))) && (tph2N > (2 * (tph3N + PWM_DEAD_TIME))) )
                TIM1->CCR4 =  switchtime[2] - TRIGGER_OFFSET_ADC;
            else
		        TIM1->CCR4 = TRIGGER_DEFAULT;
        }
		else
        {
            TIM1->CCR4 = TRIGGER_DEFAULT;
        }
	}
	else if (switchtime[0] > switchtime[1] && switchtime[0] > switchtime[2]) 
    {
		char_dyn_adc_state = 2; // -60 - +60 degrees -> phase 1 at high duty cycles
        // measure phase 2 & 3
		if(switchtime[0] > 1500)
        {
            if( (tph2N > (2 * (tph1N + PWM_DEAD_TIME))) &&  (tph3N > (2 * (tph1N + PWM_DEAD_TIME))) )
                TIM1->CCR4 =  switchtime[0] - TRIGGER_OFFSET_ADC;
            else
                TIM1->CCR4 = TRIGGER_DEFAULT;
        }
		else
        {
            TIM1->CCR4 = TRIGGER_DEFAULT;
        }
	}
    else //if (switchtime[1]>switchtime[0] && switchtime[1]>switchtime[2])
    {
		char_dyn_adc_state = 3; // 60 - 180 degrees -> phase 2 at high duty cycles
        // measure phase 3 & 1
		if(switchtime[1] > 1500)
        {
            if( (tph1N > (2 * (tph2N + PWM_DEAD_TIME))) && (tph3N > (2 * (tph2N + PWM_DEAD_TIME))) )
                TIM1->CCR4 =  switchtime[1] - TRIGGER_OFFSET_ADC;
            else
                TIM1->CCR4 = TRIGGER_DEFAULT;
        }
		else
        {
            TIM1->CCR4 = TRIGGER_DEFAULT;
        }
	}
}


/**
   * @brief Configure phases for current reading.
   * @param[in] state
   */
void set_inj_channel(uint8_t state)
{
    switch (state)
    {
    case 1: //Phase C at high dutycycles, read current from phase A + B
    {
        ADC1->JSQR = 0b00100000000000000000; //ADC1 injected reads phase A JL = 0b00, JSQ4 = 0b00100 (decimal 4 = channel 4)
        ADC1->JOFR1 = ui16_ph1_offset;
        ADC2->JSQR = 0b00101000000000000000; //ADC2 injected reads phase B, JSQ4 = 0b00101, decimal 5
        ADC2->JOFR1 = ui16_ph2_offset;
    }
    break;
    case 2: //Phase A at high dutycycles, read current from phase C + B
    {
        ADC1->JSQR = 0b00110000000000000000; //ADC1 injected reads phase C, JSQ4 = 0b00110, decimal 6
        ADC1->JOFR1 = ui16_ph3_offset;
        ADC2->JSQR = 0b00101000000000000000; //ADC2 injected reads phase B, JSQ4 = 0b00101, decimal 5
        ADC2->JOFR1 = ui16_ph2_offset;
    }
    break;

    case 3: //Phase B at high dutycycles, read current from phase A + C
    {
        ADC1->JSQR = 0b00100000000000000000; //ADC1 injected reads phase A JL = 0b00, JSQ4 = 0b00100 (decimal 4 = channel 4)
        ADC1->JOFR1 = ui16_ph1_offset;
        ADC2->JSQR = 0b00110000000000000000; //ADC2 injected reads phase C, JSQ4 = 0b00110, decimal 6
        ADC2->JOFR1 = ui16_ph3_offset;
    }
    break;
    }
}