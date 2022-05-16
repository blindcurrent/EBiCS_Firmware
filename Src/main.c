
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"



/* USER CODE BEGIN Includes */

// Lishui BLCD FOC Open Source Firmware
// Board uses IRS2003 half bridge drivers, this need inverted pulses for low-side Mosfets, deadtime is generated in driver
// This firmware bases on the ST user manual UM1052
// It uses the OpenSTM32 workbench (SW4STM32 toolchain)
// Basic peripheral setup was generated with CubeMX

#include "config.h"
//#include "eeprom.h"
#include "display_base.h"
#include "display_debug.h"
#include "pedal_sensor.h"
#include "hall.h"
#include "varia.h"
#include "wheel_speed.h"
#include "adc.h"
#include "phase_current.h"
#include "svpwm.h"
#include "debug.h"
#include "system.h"
#include "motor_control_v2.h"
#include "current_target.h"

#include <arm_math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


static uint32_t ui32_tim3_counter=0;
static uint8_t ui8_slowloop_counter=0;
uint16_t ui16_boost_counter = 0;

uint16_t ui16_ph1_offset=0;
uint16_t ui16_ph2_offset=0;
uint16_t ui16_ph3_offset=0;
uint16_t ui16_torque_offset = 0;

uint8_t PI_flag = 0;

volatile uint8_t ui8_adc_inj_flag=0;
volatile uint8_t ui8_adc_regular_flag=0;

volatile uint8_t ui8_adc_offset_done_flag=0;
volatile uint8_t ui8_external_SPEED_control_flag=0;

volatile uint16_t switchtime[3];
volatile uint16_t adcData[8]; //Buffer for ADC1 Input

//uint16_t VirtAddVarTab[NB_OF_VAR] = {0x01, 0x02, 0x03}; // eeprom

MotorState_t MS;

uint16_t ui16_motor_init_state_timeout = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);


void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();

    //initialize MS struct.
    MS.i8_assist_level = 1;
    MS.ui8_lights = 1;
    MS.ui8_walk_assist = 0;

    MS.ui8_dbg_log_value = 0;
    MS.ui16_dbg_value2 = 0;
    MS.ui16_dbg_value = 20;
    MS.ui8_go = 0;
    MS.ui8_log = 1;

    //
    init_motor_control_v2();
    init_pedal_data(&MS.PedalData);
    init_hall_data(&MS.HallData);
    init_wheel_speed_data(&MS.WheelSpeedData);
    init_adc_data(&MS.ADC_Data);
    //


    //Virtual EEPROM init
    //HAL_FLASH_Unlock();
    //EE_Init();
    //HAL_FLASH_Lock();

    MX_ADC1_Init();
    /* Run the ADC calibration */
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
    {
        /* Calibration Error */
        Error_Handler();
    }
    MX_ADC2_Init();
    /* Run the ADC calibration */
    if (HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK)
    {
        /* Calibration Error */
        Error_Handler();
    }

    /* USER CODE BEGIN 2 */
    SET_BIT(ADC1->CR2, ADC_CR2_JEXTTRIG); //external trigger enable
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
    SET_BIT(ADC2->CR2, ADC_CR2_JEXTTRIG); //external trigger enable
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);

    //HAL_ADC_Start_IT(&hadc1);
    HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)adcData, 7);
    HAL_ADC_Start_IT(&hadc2);
    MX_TIM1_Init(); //Hier die Reihenfolge getauscht!
    MX_TIM2_Init();
    MX_TIM3_Init();

    // Start Timer 1
    if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
    {
        /* Counter Enable Error */
        Error_Handler();
    }

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); // turn on complementary channel
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);

    TIM1->CCR4 = TRIGGER_DEFAULT; //ADC sampling just before timer overflow (just before middle of PWM-Cycle)
                                  //PWM Mode 1: Interrupt at counting down.

    // TIM1->BDTR |= 1L<<15;
    // TIM1->BDTR &= ~(1L<<15); //reset MOE (Main Output Enable) bit to disable PWM output
    // Start Timer 2
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
    {
        /* Counter Enable Error */
        Error_Handler();
    }

    // Start Timer 3

    if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
    {
        /* Counter Enable Error */
        Error_Handler();
    }

    Display_Init(&MS);

    // -------------------------------------- adc calibration

    CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE); //Disable PWM
    HAL_Delay(200); //wait for stable conditions
    calibrate_adc_offset(&MS.ADC_Data);
 
    // --------------------------------------


#if (AUTODETECT == 1)
    autodetect(&MS);
    while (1) { };
#endif

    CLEAR_BIT(TIM1->BDTR, TIM_BDTR_MOE); //Disable PWM
 
    init_watchdog();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    ui8_g_UART_TxCplt_flag = 1;
    //info_printf("enter main loop\n");
    HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_SET); // LED_Pin?
    while (1)
    {
        // --------------------------------------------------
        // PI control
        //

        if (PI_flag || !is_pwm_enabled())
        {
            // only pi control can enable the pwm
            run_motor_control_v2(&MS);
            process_battery_data(&MS.ADC_Data.BatteryData, MS.i32_i_d_mA, MS.i32_i_q_mA, MS.u_d, MS.u_q);
        }
        // --------------------------------------------------
        // UART Comm
        //

        //display message processing
        if (ui8_g_UART_Rx_flag)
        {
            Display_Service(&MS);

            if (MS.ui8_lights)
                HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_SET); // LED_Pin?
            else
                HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_RESET);

            ui8_g_UART_Rx_flag = 0;
        }

        // --------------------------------------------------
        process_pedal_data(&MS.PedalData);

        // --------------------------------------------------
        process_wheel_speed_data(&MS.HallData, &MS.WheelSpeedData);

        if(ui8_adc_regular_flag)
        {
            //process_battery_data(&MS.ADC_Data.BatteryData, MS.i32_i_d_mA, MS.i32_i_q_mA, MS.u_d, MS.u_q);
            ui8_adc_regular_flag = 0;
        }

        //----------------------------------------------------------------------------------------------------------------------------------------------------------
        //slow loop procedere @16Hz
        if (ui32_tim3_counter > 500)
        {

            // --------------------------------------------------
            //
            compute_remaining_distance(&MS.ADC_Data.BatteryData, MS.WheelSpeedData.uint32_SPEED_kmh_x10);
            debug_comm(&MS);

            if (ui16_motor_init_state_timeout > 0)
            {
                --ui16_motor_init_state_timeout;
            }

            if (ui8_slowloop_counter == 0)
            {
                // 1Hz process frequency for battery voltage and chip temperature

                // --------------------------------------------------
                process_chip_temperature(&MS.ADC_Data.ChipTemperatureData);
                process_motor_temperature(&MS.ADC_Data.MotorTemperatureData);

                ui8_slowloop_counter = 16;
                //test_motor_control_v2();
            }
            else
            {
                --ui8_slowloop_counter;
            }

            ui32_tim3_counter = 0;
        }

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */

} // end main while loop

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI; // LSI -> independent watchdog IWDG
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_InjectionConfTypeDef sConfigInjected;
  ADC_ChannelConfTypeDef sConfig;

  // enable temperature sensor -> seems not to be necessary
  //ADC1->CR2 |= ADC_CR2_TSVREFE;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE; //Scan muß für getriggerte Wandlung gesetzt sein
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;// Trigger regular ADC with timer 3 ADC_EXTERNALTRIGCONV_T1_CC1;// // ADC_SOFTWARE_START; //
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
  hadc1.Init.NbrOfDiscConversion = 0;


  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_DUALMODE_REGSIMULT_INJECSIMULT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Injected Channel 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4; // Hier bin ich nicht sicher ob Trigger out oder direkt CC4
  sConfigInjected.AutoInjectedConv = DISABLE; //muß aus sein
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = ui16_ph1_offset;//1900;
  HAL_ADC_Stop(&hadc1); //ADC muß gestoppt sein, damit Triggerquelle gesetzt werden kann.
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure Regular Channel
  */
sConfig.Channel = ADC_CHANNEL_7; //battery voltage
sConfig.Rank = ADC_REGULAR_RANK_1;
sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5 ;//ADC_SAMPLETIME_1CYCLE_5
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
  _Error_Handler(__FILE__, __LINE__);
}


/**Configure Regular Channel
*/
sConfig.Channel = ADC_CHANNEL_3; //Connector SP: throttle input
sConfig.Rank = ADC_REGULAR_RANK_2;
sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5; //ADC_SAMPLETIME_1CYCLE_5
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
_Error_Handler(__FILE__, __LINE__);
}
/**Configure Regular Channel
*/
sConfig.Channel = ADC_CHANNEL_4; //Phase current 1
sConfig.Rank = ADC_REGULAR_RANK_3;
sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
_Error_Handler(__FILE__, __LINE__);
}
/**Configure Regular Channel
*/
sConfig.Channel = ADC_CHANNEL_5; //Phase current 2
sConfig.Rank = ADC_REGULAR_RANK_4;
sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
_Error_Handler(__FILE__, __LINE__);
}
/**Configure Regular Channel
*/
sConfig.Channel = ADC_CHANNEL_6; //Phase current 3
sConfig.Rank = ADC_REGULAR_RANK_5;
sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
_Error_Handler(__FILE__, __LINE__);
}

/**Configure Regular Channel
*/
sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;     //ADC_CHANNEL_8; // connector AD2
sConfig.Rank = ADC_REGULAR_RANK_6;
sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
_Error_Handler(__FILE__, __LINE__);
}

/**Configure Regular Channel
*/
sConfig.Channel = ADC_CHANNEL_9; // connector AD1, temperature or torque input for Controller from PhoebeLiu @ aliexpress
sConfig.Rank = ADC_REGULAR_RANK_7;
sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_239CYCLES_5;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
{
_Error_Handler(__FILE__, __LINE__);
}

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_InjectionConfTypeDef sConfigInjected;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE; //hier auch Scan enable?!
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Injected Channel 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_5;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = ui16_ph2_offset;//	1860;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = _T;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  //sConfigOC.OCMode = TIM_OCMODE_ACTIVE; // war hier ein Bock?!
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = PWM_DEAD_TIME;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 128;                       // counter frequency = 64MHz / 128 = 500 kHz
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = HALL_TIMEOUT;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function 8kHz interrupt frequency for regular adc triggering */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 7813;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = DISPLAY_UART_BAUDRATE;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();



  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  // 27.05.21 - disabled rx cplt interrupt
  // HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  // HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Hall_1_Pin Hall_2_Pin Hall_3_Pin */
  GPIO_InitStruct.Pin = Hall_1_Pin|Hall_2_Pin|Hall_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LIGHT_Pin */
  GPIO_InitStruct.Pin = LIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LIGHT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Brake_Pin */
  GPIO_InitStruct.Pin = Brake_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Brake_GPIO_Port, &GPIO_InitStruct);


  /*Configure GPIO pins : Speed_EXTI5_Pin PAS_EXTI8_Pin */
  GPIO_InitStruct.Pin = Speed_EXTI5_Pin|PAS_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}


/* USER CODE BEGIN 4 */



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint16_t tim3_prescaler = 0;

	if(htim == &htim3) 
	{
		if(ui32_tim3_counter < 32000) ui32_tim3_counter++;
		//
		if(MS.PedalData.uint32_PAS_counter < PAS_TIMEOUT)
		{
			MS.PedalData.uint32_PAS_counter++;
			//if(HAL_GPIO_ReadPin(PAS_GPIO_Port, PAS_Pin)) 
            //{
            //    MS.PedalData.uint32_PAS_HIGH_counter++;
            //}
		}
		//

        ++tim3_prescaler;
        if(tim3_prescaler == 800)
        {
            tim3_prescaler = 0;
            // decrement boost counter at 10 Hz
            if(ui16_boost_counter)
                --ui16_boost_counter;
        }
	}

    if(htim == &htim2)
    {
        // HALL_TIMEOUT
        MS.HallData.ui8_hall_timeout_flag = 1;
        MS.HallData.ui16_timertics = HALL_TIMEOUT;
        MS.HallData.uint32_tics_filtered = HALL_TIMEOUT;
    }
}



// regular ADC callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    static uint32_t ui32_reg_adc_value_filter = 0;

	uint16_t torque_adc = adcData[TQ_ADC_INDEX];

	if(torque_adc > ui16_torque_offset)
	{
		torque_adc = adcData[TQ_ADC_INDEX] - ui16_torque_offset;
		ui32_reg_adc_value_filter -= ui32_reg_adc_value_filter >> 5;
	
		ui32_reg_adc_value_filter += torque_adc;
	
		MS.PedalData.uint16_torque_adc = ui32_reg_adc_value_filter >> 5;
	}
	else
	{
		ui32_reg_adc_value_filter = 0;
        MS.PedalData.uint16_torque_adc = 0;
	}

    ui8_adc_regular_flag=1;

}

//injected ADC

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    int32_t i32_ph1_current=0;
    int32_t i32_ph2_current=0;

	//for oszi-check of used time in FOC procedere
	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

	if(!ui8_adc_offset_done_flag)
	{
	    i32_ph1_current = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
	    i32_ph2_current = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
	    ui8_adc_inj_flag=1;
	}
	else
    {
        sample_phase_currents(&i32_ph1_current, &i32_ph2_current);


        //i16_fast_log_sample_buffer[0] = (int16_t) i32_ph1_current;
        //i16_fast_log_sample_buffer[1] = (int16_t) i32_ph2_current;
        //fast_log_data();

    	uint16_t ui16_tim2_recent = __HAL_TIM_GET_COUNTER(&htim2); // read in timertics since last event
        estimate_rotor_angle(&MS.HallData, ui16_tim2_recent);

        dynamic_phase_current_sampling();

        transform_currents(i32_ph1_current, i32_ph2_current, MS.HallData.q31_rotorposition_absolute, &MS);

        compute_svpwm_switchtime(&MS, MS.HallData.q31_rotorposition_absolute);

        // only update switchtimes if pwm is enabled
        if(is_pwm_enabled())
        {
    	    TIM1->CCR1 =  (uint16_t) switchtime[0];
    	    TIM1->CCR2 =  (uint16_t) switchtime[1];
    	    TIM1->CCR3 =  (uint16_t) switchtime[2];
        }

    

    } // end else
    	
    //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
    //Hall sensor event processing
	if(GPIO_Pin == GPIO_PIN_0 || GPIO_Pin == GPIO_PIN_1 || GPIO_Pin == GPIO_PIN_2)
	{
    	uint16_t ui16_tim2_recent = __HAL_TIM_GET_COUNTER(&htim2); // read in timertics since last hall event

        // 250 tics -> 110 kmh
        if(ui16_tim2_recent > 250) // debounce
        {
            __HAL_TIM_SET_COUNTER(&htim2, 0); //reset tim2 counter
       	    uint8_t ui8_hall_state = GPIOA->IDR & 0b111; //Mask input register with Hall 1 - 3 bits
            process_hall_event(&MS.HallData, ui16_tim2_recent, ui8_hall_state);
        }
	}

	//PAS processing
	if(GPIO_Pin == PAS_EXTI8_Pin)
	{
		MS.PedalData.ui8_PAS_flag = 1;
	}

	//Speed processing
	if(GPIO_Pin == Speed_EXTI5_Pin)
	{
        ui8_external_SPEED_control_flag = 1; //with debounce
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
//#if ( (DISPLAY_TYPE != DISPLAY_TYPE_KM_N5236) && (DISPLAY_TYPE != DISPLAY_TYPE_DEBUG) ) // -> UART_IdleItCallback
//	ui8_UART_flag=1;
//#endif
//

// disabled dma channel 5 interrupts -> see MX_DMA_Init
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	ui8_g_UART_TxCplt_flag=1;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) 
{
    Display_Init(&MS);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
