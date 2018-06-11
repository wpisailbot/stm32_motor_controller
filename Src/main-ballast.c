/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_can.h"

#include "math.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Convenience function for setting the PWM rate */
void setPWMValue(uint32_t value);

/* Convenience functions for the Inclinometer */
void setupInclinometer();
float readHeel();
int16_t readInclinometer(uint16_t reg);
uint16_t probeAddresses();


/* Convenience functions for the magnetic encoder */
void setupMagEncoder();
float readEncoder();
void setEncoderZero(float angle);

/* Convenience function for timer */
uint8_t timeEvent(uint32_t triggerTime);

/* Defines for the board's LEDs */
#define LEDG_PIN GPIO_PIN_1
#define LEDR_PIN GPIO_PIN_0
#define LED_GPIO_PORT GPIOB

/* Defines for the motor driver */
#define MOTINA_PIN GPIO_PIN_7
#define MOTENA_PIN GPIO_PIN_6
#define MOTINB_PIN GPIO_PIN_4
#define MOTENB_PIN GPIO_PIN_5

/* Defines for the Inclinometer */
#define INCL_ADDR (0x14<<1)
#define MANG_ADDR (0x40<<1)

/* Software timing for status and sensor values */
#define SENSOR_UPDATESTEP 50;
#define STATUS_UPDATESTEP 100;
uint32_t sensor_nexttime;
uint32_t status_nexttime;

uint8_t count1 = 0;
uint8_t count2 = 0;

int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();

  /* Set the CAN hardware to continue while code is stopped in debug */
  __HAL_CAN_DBG_FREEZE(&hcan, DISABLE);

  /* Clear the WKUI Bit */
  volatile unsigned int * CAN_MSR = 0x40006404u;
  *(CAN_MSR) = 3080l;


  //Add the Tx data to the handler
  CanTxMsgTypeDef myData;
  hcan.pTxMsg = &myData;

  //Add an RX buffer struct to the handler
  CanRxMsgTypeDef myRxData;
  hcan.pRxMsg = &myRxData;

  sensor_nexttime = 0;
  status_nexttime = 0;

  uint32_t motor_pwm_val = 0;
  uint32_t motor_dir_val = 0;

  // Allow time for the inclinometer to warm up (Will fail otherwise)


  for (uint8_t i=0; i<4; i++) {
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LEDG_PIN);
    HAL_Delay(100);
  }
  HAL_Delay(1000);
  setupInclinometer();

  MX_CAN_Init();

  myData.Data[0]=0xDE; myData.Data[1]=0xAD; myData.Data[2]=0xB0; myData.Data[3]=0x07; myData.Data[4]=0x02;
  myData.ExtId = 0x14FF1234;
  myData.DLC = 5;
  HAL_CAN_Transmit(&hcan, 1);

  HAL_GPIO_WritePin(LED_GPIO_PORT, LEDR_PIN, GPIO_PIN_SET);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_StatusTypeDef status;
  while (1) {
    //Clear the received message ids
    hcan.pRxMsg->StdId = 0;
    hcan.pRxMsg->ExtId = 0;
    //Attempt to receive a new can message, but timeout quickly, because there is a lot on the bus
    status = HAL_CAN_Receive(&hcan, CAN_FIFO0, 1);
    if (status==HAL_OK) {
      //hcan.pRxMsg or hcanpRx1Msg depending on FIFO used
      //Check to make sure that the message is for this motor controller
      //TODO make this ID checking more intuitive
      // Receive the SCAMP command messge
      if((hcan.pRxMsg->ExtId == 0x14FF0215) &&
         (hcan.pRxMsg->StdId == 0x00000000) &&
         (hcan.pRxMsg->DLC >= 3)) {
      //if((hcan.pRxMsg->DLC >= 3)) {
        uint8_t winchval = hcan.pRxMsg->Data[0];
        uint8_t rudderval = hcan.pRxMsg->Data[1];
        uint8_t ballastval = hcan.pRxMsg->Data[2];

        //Convert winchval to useful units

        //if (90-winchval
        /*if (winchval-90 > 0) {
          motor_pwm_val = (90-winchval)*(0x2000/90);
          motor_dir_val = 1;
        } else{
          motor_pwm_val = (winchval-90)*(0x2000/90);
          motor_dir_val = 2;
        }*/
        // period is 0x2000
        if (90-ballastval < 0) {
          motor_pwm_val = (90-ballastval)*(0x2000/90);
          motor_dir_val = 1;
        } else{
          motor_pwm_val = (ballastval-90)*(0x2000/90);
          motor_dir_val = 2;
        }
      }

      // Set Moveable Ballast zero position
      if((hcan.pRxMsg->ExtId == 0x14FFFFF1) &&
         (hcan.pRxMsg->DLC ==0)) {
        // Get the current angle as the reference position
        float curr_ang = readEncoder();
        if (curr_ang >-1000) {
          setEncoderZero(curr_ang);
        }
      
        for (int i=0; i<4; i++) {
          HAL_GPIO_TogglePin(LED_GPIO_PORT, LEDG_PIN);
          HAL_Delay(100);
        }
      }
    }


    //Update the state of the motor
    if (motor_dir_val == 1) {
      //Go Forwards
      HAL_GPIO_WritePin(GPIOB, MOTINA_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, MOTINB_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_GPIO_PORT, LEDG_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_GPIO_PORT, LEDR_PIN, GPIO_PIN_RESET);
    } else if (motor_dir_val == 2) {
      //Go Backwards
      HAL_GPIO_WritePin(GPIOB, MOTINB_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, MOTINA_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_GPIO_PORT, LEDR_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_GPIO_PORT, LEDG_PIN, GPIO_PIN_RESET);
    } else {
      //Brake or Coast
      HAL_GPIO_WritePin(GPIOB, MOTINB_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, MOTINA_PIN, GPIO_PIN_RESET);
      setPWMValue(0);
    }
    setPWMValue(motor_pwm_val);
    
    // Software "timer" for sensor updates
    if (timeEvent(sensor_nexttime)) {
      //
      sensor_nexttime += SENSOR_UPDATESTEP;
      myData.StdId = 0x00;
      myData.IDE = CAN_ID_EXT;
      myData.RTR = CAN_RTR_DATA;
      
      // Get the encoder value
      // 280 ticks/rev
      // approx 7 turns
      int16_t enc_val = __HAL_TIM_GET_COUNTER(&htim3);

      enc_val = enc_val>>4;
      enc_val += 127;
      if(enc_val < 0) {
        enc_val = 0x00;
      } else if (enc_val > 0xFF) {
        enc_val = 0xFF;
      }
      
      myData.ExtId = 0x14FF0115;
      myData.DLC = 3;
      myData.Data[0] = enc_val;
      myData.Data[2] = count1++;
      //HAL_CAN_Transmit(&hcan, 1);

      // Get the inclinometer value
      float inc_val = readHeel();

      // Get the ballast angle
      float bal_ang = readEncoder();
      float bal_angcpy = bal_ang;

      // Send data *10000 in radians
      // Inclinometer first  Not sure what the order of bits is yet
      myData.ExtId = 0x14FF0515;
      myData.DLC = 4;
      // Convert to radians 
      if (bal_ang > 180) {
        bal_ang = bal_ang-360;
      }
      bal_ang = bal_ang*3.14159/180.0*10000;
      // Dumb offset right now to get it centered on 0
      inc_val = -inc_val;
      inc_val = inc_val*3.14159/180.0*10000;
      myData.Data[0] = ((int16_t)inc_val) & 0xFF;
      myData.Data[1] = (((int16_t)inc_val) >> 8) & 0xFF;
      myData.Data[2] = ((int16_t)bal_ang) & 0xFF;
      myData.Data[3] = (((int16_t)bal_ang) >> 8) & 0xFF;
      HAL_CAN_Transmit(&hcan, 1);

      if (bal_angcpy < -900) {
        // Send a CAN message that I2C is resetting
        myData.Data[0]=0xDE; myData.Data[1]=0xAD; myData.Data[2]=0x12; myData.Data[3]=0xC2;
        myData.ExtId = 0x14FF1234;
        myData.DLC = 4;
        HAL_CAN_Transmit(&hcan, 100);
        // Reset I2C
        MX_I2C1_Init();
      }
    }

    uint32_t curr_time = HAL_GetTick();
    myData.ExtId = 0x14FF1235;
    myData.DLC = 4;
    myData.Data[0] = (curr_time>>24)&0xFF;
    myData.Data[1] = (curr_time>>16)&0xFF;
    myData.Data[2] = (curr_time>>8) &0xFF;
    myData.Data[3] = curr_time & 0xFF;
    HAL_CAN_Transmit(&hcan, 100);

    // Software "timer" for status updates
    if (timeEvent(status_nexttime)) {
      status_nexttime += STATUS_UPDATESTEP;
      myData.StdId = 0x00;
      myData.ExtId = 0x14FF0315;
      myData.IDE = CAN_ID_EXT;
      myData.RTR = CAN_RTR_DATA;
      myData.DLC = 6;
      /* Trigger an ADC read */
      uint16_t temp_val;
      uint16_t curr_val;

      //Set the ADC to read from specific channels in setup for group mode
      //Then call Start, and poll for conversion.  GetValue into a buffer, and read for as many times as channels setup.
      //Call start for next polling round.  TODO fix this to be "correct"
      HAL_ADC_Start(&hadc);
      HAL_ADC_PollForConversion(&hadc, 10);
      curr_val = HAL_ADC_GetValue(&hadc);
      temp_val = HAL_ADC_GetValue(&hadc);
      HAL_ADC_Stop(&hadc);
      //Send the Temp sensor value
      myData.Data[1] = temp_val & 0xff;
      myData.Data[0] = (temp_val>>8) & 0xff;
      //Send the Current sensor value
      myData.Data[3] = curr_val & 0xff;
      myData.Data[2] = (curr_val>>8) & 0xff;
      //Send the Motor Disabled status value
      myData.Data[4] = (!HAL_GPIO_ReadPin(GPIOB, MOTENB_PIN)<<1) |
                       (!HAL_GPIO_ReadPin(GPIOB, MOTENA_PIN));
      myData.Data[5] = count2++;
      HAL_CAN_Transmit(&hcan, 100);
    }
  }
}

void setPWMValue(uint32_t value) {
  TIM_OC_InitTypeDef sConfigOC;
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = value;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  //Terrible way to do this, but good enough without modifying the libs for now... TODO
  if (HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
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

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
       TODO Not sure why these settings need to be like this, but it works*/
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  //hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.ScanConvMode = DISABLE;
  //hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.EOCSelection = DISABLE;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  //hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */

  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

  sConfig.Channel = ADC_CHANNEL_0;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  //hcan.Init.Mode = CAN_MODE_LOOPBACK;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_6TQ;  //Time Quanta settings from
  hcan.Init.BS2 = CAN_BS2_1TQ;  //CANOpen recommendations
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  //hcan.Init.NART = ENABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  //Setup a filter to filter out nothing!
  CAN_FilterConfTypeDef filterConf;
  filterConf.FilterIdHigh = 0x0000;
  filterConf.FilterIdLow = 0x0000;
  filterConf.FilterMaskIdHigh = 0x0000;
  filterConf.FilterMaskIdLow = 0x0000;
  filterConf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filterConf.FilterNumber = 0;
  filterConf.FilterMode = CAN_FILTERMODE_IDMASK;
  filterConf.FilterScale = CAN_FILTERSCALE_32BIT;
  filterConf.FilterActivation = ENABLE;
  filterConf.BankNumber = 0;
  if (HAL_CAN_ConfigFilter(&hcan, &filterConf) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  //hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.Timing = 0x1042C3C7; //Timing for 10kHz clock w/ 8Mhz osc
  //hi2c1.Init.Timing = 0x10420F13;  //Timing for 100kHz clock w/ 8MHz osc
  //hi2c1.Init.Timing = 0x00310309;  //Timing for 400kHz clock w/ 8MHz osc
  //hi2c1.Init.Timing = 0x00100306;  //Timing for 500kHz clock w/ 8MHz osc
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /* Start the Encoder counting module */
  //if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL)) {
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1)) {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 0x2000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0x0800;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim16);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB4 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif


void setupInclinometer() {

  uint8_t txbuf[2];
  HAL_StatusTypeDef status;

  //Setup the required factory bits for boot
  //Need to send 0x0F to address 0x08 per the datasheet
  txbuf[0] = 0x0f;
  status = HAL_I2C_Mem_Write(&hi2c1, INCL_ADDR, 0x08, 1, txbuf, 1, 100);

  // Block for now if there is an error
  if (status != HAL_OK) {
    for (uint8_t i=0; i<10; i++) {
      HAL_GPIO_TogglePin(LED_GPIO_PORT, LEDG_PIN);
      HAL_GPIO_TogglePin(LED_GPIO_PORT, LEDR_PIN);
      HAL_Delay(100);
    }
  }
  //Setup the tipover register for 70 degrees
  txbuf[0] = 0x18;
  status = HAL_I2C_Mem_Write(&hi2c1, INCL_ADDR, 0x07, 1, txbuf, 1, 100);

  // Block for now if there is an error
  if (status != HAL_OK) {
    for (uint8_t i=0; i<10; i++) {
      HAL_GPIO_TogglePin(LED_GPIO_PORT, LEDG_PIN);
      HAL_GPIO_TogglePin(LED_GPIO_PORT, LEDR_PIN);
      HAL_Delay(500);
    }
  }

  //Allow the inclinometer time to warmup
  HAL_Delay(500);
}

float readHeel() {
  //Get the X and Y accelerations
  int16_t x_val = readInclinometer(0x00);
  int16_t y_val = readInclinometer(0x02);

  //Convert from radians to degrees
  return atan2(x_val,y_val)*57.32;
}

int16_t readInclinometer(uint16_t reg) {
  uint8_t rxbuf[2];
  HAL_StatusTypeDef status;

  //TODO Add acceleration check from register 0x06
  int16_t val;
  //For some reason, multibyte reads doesn't appear to work...
  //Get the upper 6 bits
  status = HAL_I2C_Mem_Read(&hi2c1, INCL_ADDR, reg+1, 1, rxbuf, 1, 100);
  if (status == HAL_OK) {
    val = rxbuf[0]<<8;
  } else {
    return 0xFFFF;
  }
  //Get the lower 8 bits
  status = HAL_I2C_Mem_Read(&hi2c1, INCL_ADDR, reg, 1, rxbuf, 1, 100);
  if (status == HAL_OK) {
    val |= rxbuf[0];
  } else {
    return 0xFFFF;
  }

  return val<<2;
}


uint16_t probeAddresses() {
  uint8_t txbuf[2];
  HAL_StatusTypeDef status;

  txbuf[0] = 0x0f;

  for (uint16_t i=0; i<=0x7F; i++) {
    status = HAL_I2C_Mem_Write(&hi2c1, (i<<1), 0x08, 1, txbuf, 1, 50);
    if (status == HAL_OK) {
      return i;
    }
  }
  return 0xFFFF;
}

void setupEncoder() {

}

float readEncoder() {
  uint8_t rxbuf[2];
  HAL_StatusTypeDef status;

  uint16_t encoder_val = 0;

  //Get the upper 8 bits of the angle
  status = HAL_I2C_Mem_Read(&hi2c1, MANG_ADDR, 0xFE, 1, rxbuf, 1, 100);
  encoder_val = rxbuf[0]<<6;

  if (status != HAL_OK) {
    if (status == HAL_ERROR) {
      if (hi2c1.ErrorCode == HAL_I2C_ERROR_AF) {
        return -1000;
      } else if (hi2c1.ErrorCode == HAL_I2C_ERROR_BERR) {
        return -1010;
      } else if (hi2c1.ErrorCode == HAL_I2C_ERROR_ARLO) {
        return -1020;
      } else if (hi2c1.ErrorCode == HAL_I2C_ERROR_OVR) {
        return -1080;
      } else if (hi2c1.ErrorCode == HAL_I2C_ERROR_TIMEOUT) {
        return -1200;
      } else if (hi2c1.ErrorCode == HAL_I2C_ERROR_SIZE) {
        return -1400;
      } else {
        return -1600;
      }
    } else if (status == HAL_BUSY) {
      return -2000;
    } else if (status == HAL_TIMEOUT) {
      return -3000;
    }
  }

  //Get the lower 6 bits of the angle
  status = HAL_I2C_Mem_Read(&hi2c1, MANG_ADDR, 0xFF, 1, rxbuf, 1, 100);
  encoder_val |= rxbuf[0];
  if (status != HAL_OK) {
    return -1000;
  }

  return encoder_val / 45.508; //Convert to degrees
}

void setEncoderZero(float angle) {
  //float newZero = angle * 45.508;
  //uint8_t top = ((uint16_t)newZero) >> 6;  //Upper bits of zero pos (not really)
  //uint8_t btm = ((uint16_t)newZero) & 0x003F; //Bottom 6 bits (not really)

  HAL_StatusTypeDef status;

  uint8_t top, btm;
  uint8_t zero = 0;
  status = HAL_I2C_Mem_Write(&hi2c1, MANG_ADDR, 0x16, 1, &zero, 1, 100);
  status = HAL_I2C_Mem_Write(&hi2c1, MANG_ADDR, 0x17, 1, &zero, 1, 100);

  status = HAL_I2C_Mem_Read(&hi2c1, MANG_ADDR, 0xFE, 1, &top, 1, 100);
  status = HAL_I2C_Mem_Read(&hi2c1, MANG_ADDR, 0xFF, 1, &btm, 1, 100);
  status = HAL_I2C_Mem_Write(&hi2c1, MANG_ADDR, 0x16, 1, &top, 1, 100);
  status = HAL_I2C_Mem_Write(&hi2c1, MANG_ADDR, 0x17, 1, &btm, 1, 100);

}

// Gets around the issue of timer overflows
uint8_t timeEvent(uint32_t triggerTime) {
  uint32_t curr_time = HAL_GetTick();
  if (triggerTime <= curr_time) {
    return 1;
  }
  return (triggerTime - curr_time) > ((0xFFFFFFFF/4)*3);
}

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
