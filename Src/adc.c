#include "adc.h"
#include "display_debug.h"


// extern data from main.c
extern volatile uint16_t adcData[8];
extern volatile uint8_t ui8_adc_offset_done_flag;
extern volatile uint8_t ui8_adc_regular_flag;

extern uint16_t ui16_ph1_offset;
extern uint16_t ui16_ph2_offset;
extern uint16_t ui16_ph3_offset;
extern uint16_t ui16_torque_offset;

static int32_t i32_battery_voltage_Vx10_cumulated = 0;
static int32_t i32_battery_voltage_uncorrected_Vx10_cumulated;
static int32_t i32_battery_current_mA_cumulated = 0;
static const uint8_t ui8_battery_voltage_shift = 12;
static const uint8_t ui8_battery_current_shift = 7;


void init_adc_data(ADC_Data_t *pADC_Data)
{
    pADC_Data->BatteryData.enum_voltage_state = VOLTAGE_STATE_NORMAL;
    pADC_Data->BatteryData.i32_battery_current_mA = 0;
    pADC_Data->BatteryData.i32_battery_power_W_x10 = 0;
    pADC_Data->ChipTemperatureData.ui8_shift = 5;
    pADC_Data->ChipTemperatureData.enum_temperature_state = TEMP_STATE_NOMRAL;
    pADC_Data->MotorTemperatureData.ui8_shift = 5;
}

void calibrate_adc_offset(ADC_Data_t *pADC_Data)
{
    ui8_adc_offset_done_flag = 0;
    uint32_t ui32_ph1_offset_temp = 0;
    uint32_t ui32_ph2_offset_temp = 0;
    uint32_t ui32_ph3_offset_temp = 0;
    uint32_t ui32_torque_offset_temp = 0;
    uint32_t ui32_battery_voltage_temp = 0;
    uint32_t ui32_chip_temperature_temp = 0;

    const uint8_t N = 32;

    pADC_Data->ChipTemperatureData.q31_temperature_adc_cumulated = 0;

    for (uint8_t i = 0; i < N; i++)
    {
        while (!ui8_adc_regular_flag){}
        ui32_ph1_offset_temp += adcData[2];
        ui32_ph2_offset_temp += adcData[3];
        ui32_ph3_offset_temp += adcData[4];
        ui32_torque_offset_temp += adcData[TQ_ADC_INDEX];
        ui32_battery_voltage_temp += adcData[0];
        ui32_chip_temperature_temp += adcData[CHIP_TEMP_ADC_INDEX];
        ui8_adc_regular_flag = 0;
    }
    ui16_ph1_offset = ui32_ph1_offset_temp / N;
    ui16_ph2_offset = ui32_ph2_offset_temp / N;
    ui16_ph3_offset = ui32_ph3_offset_temp / N;

    //ui16_torque_offset = ui32_torque_offset_temp / N;

    ui16_torque_offset = TORQUE_ADC_OFFSET;

    i32_battery_voltage_Vx10_cumulated = (ui32_battery_voltage_temp * CAL_BAT_V / (N * 100)) << ui8_battery_voltage_shift;
    i32_battery_voltage_uncorrected_Vx10_cumulated = i32_battery_voltage_Vx10_cumulated;

    pADC_Data->BatteryData.i32_battery_voltage_Vx10 = i32_battery_voltage_Vx10_cumulated >> ui8_battery_voltage_shift;

    pADC_Data->ChipTemperatureData.q31_temperature_adc_cumulated = (ui32_chip_temperature_temp / N) << (pADC_Data->ChipTemperatureData.ui8_shift);
    pADC_Data->ChipTemperatureData.q31_temperature_degrees = 0; // setting it in the slow loop

#ifdef MOTOR_TEMP_SENSOR_INSTALLED
    pADC_Data->MotorTemperatureData.q31_temperature_adc_cumulated = adcData[MOTOR_TEMP_ADC_INDEX] << (pADC_Data->MotorTemperatureData.ui8_shift);
#endif

// comment hochsitzcola 20.05.21
#ifdef DISABLE_DYNAMIC_ADC               // set  injected channel with offsets
    ADC1->JSQR = 0b00100000000000000000; //ADC1 injected reads phase A JL = 0b00, JSQ4 = 0b00100 (decimal 4 = channel 4)
    ADC1->JOFR1 = ui16_ph1_offset;
    ADC2->JSQR = 0b00101000000000000000; //ADC2 injected reads phase B, JSQ4 = 0b00101, decimal 5
    ADC2->JOFR1 = ui16_ph2_offset;
#endif

#if (DISPLAY_TYPE == DISPLAY_TYPE_DEBUG)
    info_printf("LISHUI OPEN SOURCE FOC FIRMWARE\n");
    while(ui8_g_UART_TxCplt_flag == 0) {};
    info_printf("phase current offsets:  %d, %d, %d\n", ui16_ph1_offset, ui16_ph2_offset, ui16_ph3_offset);
    while(ui8_g_UART_TxCplt_flag == 0) {};
    info_printf("torque offset:  %d (%d)\n\n", ui16_torque_offset, (int16_t) ui32_torque_offset_temp/N);
    while(ui8_g_UART_TxCplt_flag == 0) {};
#endif
    
    ui8_adc_offset_done_flag = 1;
}


void process_battery_data(BatteryData_t *pBatteryData, int32_t i32_i_d_mA, int32_t i32_i_q_mA, int32_t u_d, int32_t u_q)
{
    int32_t i32_battery_current_mA_temp = (i32_i_d_mA * u_d + i32_i_q_mA * u_q) / 2365;
    i32_battery_current_mA_cumulated -= i32_battery_current_mA_cumulated >> ui8_battery_current_shift;
    i32_battery_current_mA_cumulated += i32_battery_current_mA_temp;
    pBatteryData->i32_battery_current_mA = i32_battery_current_mA_cumulated >> ui8_battery_current_shift;


    i32_battery_voltage_Vx10_cumulated -= (i32_battery_voltage_Vx10_cumulated >> ui8_battery_voltage_shift);
    // add voltage drop across internal resistance (0.14 ohm)
    i32_battery_voltage_Vx10_cumulated += adcData[0] * CAL_BAT_V / 100   +   i32_battery_current_mA_temp * 14 / 10000;
    pBatteryData->i32_battery_voltage_Vx10 = i32_battery_voltage_Vx10_cumulated >> ui8_battery_voltage_shift;
    //
    i32_battery_voltage_uncorrected_Vx10_cumulated -= (i32_battery_voltage_uncorrected_Vx10_cumulated >> ui8_battery_voltage_shift);
    i32_battery_voltage_uncorrected_Vx10_cumulated += adcData[0] * CAL_BAT_V / 100;
    pBatteryData->i32_battery_voltage_uncorrected_Vx10 = i32_battery_voltage_uncorrected_Vx10_cumulated >> ui8_battery_voltage_shift;

    pBatteryData->i32_battery_power_W_x10 = pBatteryData->i32_battery_voltage_Vx10 * pBatteryData->i32_battery_current_mA / 1000;

    voltage_state_t new_state = pBatteryData->enum_voltage_state;
    switch(pBatteryData->enum_voltage_state)
    {
        case VOLTAGE_STATE_NORMAL:
            if(pBatteryData->i32_battery_voltage_Vx10 < BATTERY_CRITICAL_VOLTAGE) 
                new_state = VOLTAGE_STATE_CRITICAL;
            break;
        case VOLTAGE_STATE_CRITICAL:
            if(pBatteryData->i32_battery_voltage_Vx10 > BATTERY_CRITICAL_VOLTAGE + 20) 
                new_state = VOLTAGE_STATE_NORMAL;
            if(pBatteryData->i32_battery_voltage_Vx10 < BATTERY_UNDER_VOLTAGE) 
                new_state = VOLTAGE_STATE_UNDER_VOLTAGE_ERROR;
            break;
        case VOLTAGE_STATE_UNDER_VOLTAGE_ERROR:
            if(pBatteryData->i32_battery_voltage_Vx10 > BATTERY_UNDER_VOLTAGE + 20) 
                new_state = VOLTAGE_STATE_CRITICAL;
            break;
    }
    pBatteryData->enum_voltage_state = new_state;
}


/**
   * @brief Compute state of charge (soc) from the voltage level with a piecewise linear relationship.
   * @param[in]     i32_voltage_Vx10
   * @param[out]    ui32_soc in percent
   */
static uint32_t get_soc_from_voltage(int32_t i32_voltage_Vx10)
{
    static const int16_t VOLTAGE[4] = {390, 436, 482, 544};
    static const int16_t SOC[4]     = {0,    10,  50, 100};
    uint32_t ui32_soc;

    if(i32_voltage_Vx10 < VOLTAGE[0])
    {
        return 0;
    }
    if(i32_voltage_Vx10 > VOLTAGE[3])
    {
        return 100;
    }

    for(int8_t i = 3; i >= 1; --i)
    {
        if(i32_voltage_Vx10 >= VOLTAGE[i-1])
        {
            ui32_soc = SOC[i-1] + ((SOC[i] - SOC[i-1]) * (i32_voltage_Vx10 - VOLTAGE[i-1])) / (VOLTAGE[i] - VOLTAGE[i-1]);
            break;
        }
    }

    return ui32_soc;
}


/**
   * @brief Compute remaining distance and time given the current velocity and battery power.
   * @param[in]     i32_battery_power_W_x10 (BatteryData)
   * @param[in]     i32_battery_voltage_Vx10 (BatteryData)
   * @param[in]     ui32_speed_kmh_x10
   * @param[out]    ui8_remaining_distance_km (BatteryData)
   * @param[out]    ui16_remaining_time_min (BatteryData)
   */
void compute_remaining_distance(BatteryData_t *pBatteryData, uint32_t ui32_speed_kmh_x10)
{
    // 1100 Wh
    // voltage range: 41-54V  -> 84.6 Wh / V
    static uint32_t ui32_E_distance_kmx100_cumulated;
    static uint32_t ui32_E_time_min_cumulated;
    static const uint8_t ui8_shift = 8;
    static uint8_t ui8_initialize_flag = 1;
    //uint32_t E = 0;
    if(pBatteryData->i32_battery_voltage_Vx10 > 390)
    {
        int32_t i32_W = pBatteryData->i32_battery_power_W_x10;
    
        // linear 
        //uint32_t ui32_E = (pBatteryData->i32_battery_voltage_Vx10 - 410) * 8460;  // remaining Wh x 1000
        
        // piecewise linear
        uint32_t ui32_E = (get_soc_from_voltage(pBatteryData->i32_battery_voltage_Vx10) * 1100 / 100) * 1000;  // remaining  Wh x 1000

        if(ui8_initialize_flag)
        {
            // initialize the togo value with some assumptions
            ui32_E = ui32_E / 4000; // remaining hours x 100 with 400W
            ui32_E = ui32_E * 30;  //  remaining kmh x 100 with 30 kmh
            ui32_E_distance_kmx100_cumulated = (ui32_E << ui8_shift);
            ui8_initialize_flag = 0;
        }
        else if(i32_W > 500) // do not update if no power is used (or if the power is negative)
        {

            ui32_E = ui32_E / i32_W; // E = Wh x 1000 / W x10 = remaining hours x 100
            ui32_E_time_min_cumulated -= (ui32_E_time_min_cumulated >> ui8_shift);
            ui32_E_time_min_cumulated += ui32_E * 60 / 100;
            //
            uint32_t s = (ui32_speed_kmh_x10) / 10;
            if(s < 5) 
            {
                s = 5;
            }
            ui32_E = ui32_E * s; // E = hours x 100 * km / hour = remaining km x 100
        
            ui32_E_distance_kmx100_cumulated -= (ui32_E_distance_kmx100_cumulated >> ui8_shift);
            ui32_E_distance_kmx100_cumulated += ui32_E;
        }

        // bound at 120 km
        if(ui32_E_distance_kmx100_cumulated > (12000 << ui8_shift))
        {
            ui32_E_distance_kmx100_cumulated = (12000 << ui8_shift);
        }
        
        pBatteryData->ui8_remaining_distance_km = (uint8_t) ((ui32_E_distance_kmx100_cumulated >> ui8_shift) / 100);
        pBatteryData->ui16_remaining_time_min = (uint16_t) (ui32_E_time_min_cumulated >> ui8_shift);
    }
    else
    {
        pBatteryData->ui8_remaining_distance_km = 0;
        pBatteryData->ui16_remaining_time_min = 0;
    }
}

void process_chip_temperature(TemperatureData_t* pChipTemperatureData)
{
    pChipTemperatureData->q31_temperature_adc_cumulated -= (pChipTemperatureData->q31_temperature_adc_cumulated >> pChipTemperatureData->ui8_shift);
    pChipTemperatureData->q31_temperature_adc_cumulated += adcData[CHIP_TEMP_ADC_INDEX];

    // data sheet STM32F103x4
    // 5.3.19 Temperature sensor characteristics
    // avg voltage at 25 degrees 1.43 V
    // avg solpe 4.3 mV / degree

    // recover voltage (3.3V  ref voltage, 2^12 = 4096)
    // int32_t v_temp = adcData[CHIP_TEMP_ADC_INDEX] * 3300 >> 12; // voltage in mV
    int32_t v_temp = (pChipTemperatureData->q31_temperature_adc_cumulated >> pChipTemperatureData->ui8_shift) * 3300 >> 12; // voltage in mV
    //
    v_temp = (1430 - v_temp) * 10 / 43 + 25;
    if (v_temp > 0)
        pChipTemperatureData->q31_temperature_degrees = v_temp;
    else
        pChipTemperatureData->q31_temperature_degrees = 0;

    temperature_state_t new_state = pChipTemperatureData->enum_temperature_state;

    // process state transition
    switch(pChipTemperatureData->enum_temperature_state)
    {
        case TEMP_STATE_NOMRAL:
            if(pChipTemperatureData->q31_temperature_degrees > CRITICAL_CHIP_TEMPERATURE) 
                new_state = TEMP_STATE_CRITICAL;
            break;
        case TEMP_STATE_CRITICAL:
            if(pChipTemperatureData->q31_temperature_degrees > CHIP_OVER_TEMPERATURE) 
                new_state = TEMP_STATE_OVER_TEMP_ERROR;
            if(pChipTemperatureData->q31_temperature_degrees < CRITICAL_CHIP_TEMPERATURE - 10) 
                new_state = TEMP_STATE_NOMRAL;
            break;
        case TEMP_STATE_OVER_TEMP_ERROR:
            if(pChipTemperatureData->q31_temperature_degrees < CHIP_OVER_TEMPERATURE - 10)
                new_state = TEMP_STATE_CRITICAL;
            break;
    }
    pChipTemperatureData->enum_temperature_state = new_state;

}

void process_motor_temperature(TemperatureData_t* pMotorTemperatureData)
{
#ifdef MOTOR_TEMP_SENSOR_INSTALLED
    // 
    pMotorTemperatureData->q31_temperature_adc_cumulated -= (pMotorTemperatureData->q31_temperature_adc_cumulated >> pMotorTemperatureData->ui8_shift);
    pMotorTemperatureData->q31_temperature_adc_cumulated += adcData[MOTOR_TEMP_ADC_INDEX];
    //uint16_t raw_data = adcData[MOTOR_TEMP_ADC_INDEX];
    // adc: mapping 5000mV to 1 << 12 = 4096 adc steps
    // scale adc reading to mv
    int32_t v_temp = (pMotorTemperatureData->q31_temperature_adc_cumulated >> pMotorTemperatureData->ui8_shift) * 5000 >> 12;
    // TMP36 : 750 mv @ 25 degrees
    // TMP36 : 10 mV / K slope
    v_temp = 25 + (v_temp - 750) / 10;
    

    if(v_temp > 0)
        pMotorTemperatureData->q31_temperature_degrees = v_temp;
    else
        pMotorTemperatureData->q31_temperature_degrees = 0;
#endif
}