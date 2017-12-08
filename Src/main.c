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
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "PID.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define pi      3.141592654
#define MAX_BUF_DATA_RECV   50  
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile int32_t i16_Counter_Left = 0, i16_Counter_Right = 0;
volatile int32_t i16_Counter_Left_smooth = 0, i16_Counter_Right_smooth = 0;
uint8_t ui8_BufLog[50];
uint8_t idx_uart = 0;
uint8_t write_idx = 0, read_idx = 0;
uint16_t u16_avail_byte = 0;

float f_Pulse_Target_Left = 0, f_Pulse_Target_Right = 0, f_Pulse_Target_temp;
float f_Error_Left, f_Error_Right;
float f_PIDResult_Left, f_PIDResult_Right;
float f_PIDScale_Left, f_PIDScale_Right;

float f_radius = 0.03;        //metter
float f_length = 0.2;        //metter   
uint8_t ui8_dir_bot = 0;    //0: Forward 1:backward
uint8_t ui8_dir_cmd = 0;    //0: Forward 1:backward
uint8_t count_reset = 0;

char* cToken;
int8_t idx_read_buf = 0;
uint8_t idx = 0;
char Rx_indx, Rx_data[2], Rx_Buffer[50], Rx_Buffer_uart[200], Transfer_cplt;

float f_tangent_vel_target =0, f_angular_vel_target=0;  //wc and vc
float f_lwheel_omega_target =0, f_rwheel_omega_target=0;    //wl, wr
float f_lwheel_vec_target =0, f_rwheel_vec_target=0;    //vl, vr
float f_lwheel_omega_enc = 0, f_rwheel_omega_enc = 0;
float f_lwheel_vec_enc = 0, f_rwheel_vec_enc = 0; 
float f_tangent_vel_enc =0, f_angular_vel_enc=0;
PID_PARAMETERS PID_ParaMotor_Left = {.Kp = 0.0022, .Kd = 0.0008, .Ki = 0.1,
             .Ts = 0.020, .PID_Saturation = 95, .e_=0, .e__=0, .u_=0};
PID_PARAMETERS PID_ParaMotor_Right = {.Kp = 0.0022, .Kd = 0.0008, .Ki = 0.1,
             .Ts = 0.020, .PID_Saturation = 95, .e_=0, .e__=0, .u_=0};
//PID_PARAMETERS PID_ParaMotor_Right = {.Kp = 0.002, .Kd = 0.008, .Ki = 0.1,
//             .Ts = 0.020, .PID_Saturation = 95, .e_=0, .e__=0, .u_=0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void UART_Log(uint8_t* log)
{
  HAL_UART_Transmit_DMA(&huart2, log, strlen(log));
}

void uart_gets(uint8_t* rec_data)
{
   HAL_UART_Receive_DMA(&huart2, rec_data,1);
}

void uart_puts(uint8_t *str)					//Sends a String to the UART.
{
     uint8_t i= strlen((char*)str);
     HAL_UART_Transmit_DMA(&huart2,str,i);
//     str++;
     
//     while(*str) uart_putc(*str++);			//Advance though string till end
//     return;
}


void string_to_num()
{
//	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);

    char* cToken_temp;
    if (Rx_Buffer[0]=='[')
    {
    cToken_temp = strtok (Rx_Buffer,"[");
    if (cToken_temp != NULL)
    {
    cToken = strtok (cToken_temp, ",");
    f_tangent_vel_target = atof(cToken);
        if (cToken_temp != NULL)
        {
        cToken = strtok(NULL,"E"); 
        f_angular_vel_target = atof(cToken);
        }
    }
    }
}

void to_motor_cmd ()
{

//    f_rwheel_vec_target = (2*f_tangent_vel_target + f_angular_vel_target*f_length)/2;
//    f_lwheel_vec_target = (2*f_tangent_vel_target - f_angular_vel_target*f_length)/2;
    f_rwheel_vec_target = (2*f_tangent_vel_target - f_angular_vel_target*f_length)/2;
    f_lwheel_vec_target = (2*f_tangent_vel_target + f_angular_vel_target*f_length)/2;
    f_rwheel_omega_target = f_rwheel_vec_target/f_radius;
    f_lwheel_omega_target = f_lwheel_vec_target/f_radius;
    f_Pulse_Target_Right =  (f_rwheel_omega_target*3250*4*0.02)/(2*pi);   //Tsample=0.02
    f_Pulse_Target_Left =  (f_lwheel_omega_target*3250*4*0.02)/(2*pi);   //Tsample=0.02
}

void encoder_feedback()
{
    f_rwheel_omega_enc = (i16_Counter_Right/(3250*4*0.02))*2*pi;  //Tsample=0.02s
    f_lwheel_omega_enc = (i16_Counter_Left/(3250*4*0.02))*2*pi;  //Tsample=0.02s
    f_rwheel_vec_enc = f_rwheel_omega_enc*f_radius;
    f_lwheel_vec_enc = f_lwheel_omega_enc*f_radius;  
//    f_angular_vel_enc = (f_rwheel_vec_enc - f_lwheel_vec_enc)/f_length; 
    f_angular_vel_enc = (f_lwheel_vec_enc - f_rwheel_vec_enc)/f_length; 
    f_tangent_vel_enc = (f_rwheel_vec_enc + f_lwheel_vec_enc)/2;   
    
//    sprintf((char*) ui8_BufLog,"[%f,%f]",f_tangent_vel_enc, f_angular_vel_enc);
//    UART_Log(ui8_BufLog);
}

//void uart_receive_data ()
//{
//    uint8_t i;
//    if (Rx_indx==0) {for (i=0;i<50;i++) Rx_Buffer[i]=0;}   //clear Rx_Buffer before receiving new data 
//    if (Rx_data[idx_uart]=='[')    
//    {
//        while (idx_uart<200) //if received data different from ascii 13 (enter)
//        {
//            Rx_Buffer[Rx_indx++]=Rx_data[idx_uart++];    //add data to Rx_Buffer            
//        if (Rx_data[idx_uart]==']')             //if received data = 13
//        {
//            Rx_Buffer[Rx_indx]='E';
//            Rx_indx=0;
//            idx_uart++;
//            Transfer_cplt=1;//transfer complete, data is ready to read
//            string_to_num();
//            //convert
//            to_motor_cmd();     //de duoi phan set PWM
//            break;
//        }
//        if(idx_uart == 200)
//        {
//            idx_uart = 0;
//        }
//        }
//	 }   
//}

void uart_receive_data ()
{
    if (u16_avail_byte>0)
    {
    while (Rx_Buffer_uart[read_idx++] != ']')
    {
        if (Rx_Buffer_uart[read_idx] == '[')
        {
            idx = 0;
            continue;
        }
        if (read_idx != write_idx)
        {
            Rx_Buffer[idx] = Rx_Buffer_uart[read_idx];
            idx++;      //con tro cho mang buffer
        }
        if (read_idx == 200)
        {
            read_idx = 0;
        }
        if (u16_avail_byte)
        {
            u16_avail_byte--;
        }
    }
    }
    string_to_num();
    to_motor_cmd();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == htim3.Instance)
  {
//    uint8_t ui8_dir_bot = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) & __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5);
    i16_Counter_Left = TIM2->CNT;
    i16_Counter_Right = TIM5->CNT;
   
    if (i16_Counter_Left>30000)
    {
        i16_Counter_Left = i16_Counter_Left - 50000;        
    }
    if (i16_Counter_Right>30000)
    {
        i16_Counter_Right = i16_Counter_Right - 50000;        
    }
    __HAL_TIM_SetCounter(&htim2, 0);
    __HAL_TIM_SetCounter(&htim5, 0);
    
    //count reset
    if (count_reset >= 100)
    {
        HAL_GPIO_WritePin (GPIOE, GPIO_PIN_7 | GPIO_PIN_9,1);
    }
    else 
    {
        count_reset++;
    }
    
    //filter
//    i16_Counter_Left_k_1 = i16_Counter_Left_k + (uint32_t)(0.9*(i16_Counter_Left-i16_Counter_Left_k)); 
    i16_Counter_Left_smooth = i16_Counter_Left_smooth + (0.2*(i16_Counter_Left-i16_Counter_Left_smooth));
    i16_Counter_Right_smooth = i16_Counter_Right_smooth + (0.2*(i16_Counter_Right-i16_Counter_Right_smooth));  
    i16_Counter_Left = i16_Counter_Left_smooth;        
    i16_Counter_Right = i16_Counter_Right_smooth;    
    //end filter
    //uart
    uart_receive_data();
    //end uart
    f_Error_Left = f_Pulse_Target_Left - (float)i16_Counter_Left;
    f_PIDResult_Left = pid_process(&PID_ParaMotor_Left, f_Error_Left);
    
    f_Error_Right = f_Pulse_Target_Right - (float)i16_Counter_Right;
    f_PIDResult_Right = pid_process(&PID_ParaMotor_Right, f_Error_Right);

//    f_PIDScale_Left = (f_PIDResult_Left + 100)/2;
//    f_PIDScale_Right = (f_PIDResult_Right + 100)/2;   


    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (int8_t)f_PIDScale_Left);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (int8_t)f_PIDScale_Left);  

    //feedback
    encoder_feedback();    
//    sprintf((char*) ui8_BufLog,"%f \t %d \t %f \t %d \t %d \n\r",f_Pulse_Target_Left, i16_Counter_Left,f_Pulse_Target_Right, i16_Counter_Right,ui8_dir_bot);
//     sprintf((char*) ui8_BufLog,"%f \t %f\n\r",f_tangent_vel_target, f_angular_vel_target); 
//    sprintf((char*) ui8_BufLog,"%f \t %d \t %f \t %d\n\r",f_Pulse_Target_Left, i16_Counter_Left,f_Pulse_Target_Right, i16_Counter_Right);
    //sprintf((char*) ui8_BufLog,"%d \t %d \t %d \t %d\n\r", i16_counter, (int16_t)f_Error_Left, (int16_t)f_PIDResult_Left, (int16_t)f_PIDScale_Left);
//    UART_Log(ui8_BufLog);
     
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==huart2.Instance)
	{
        Rx_Buffer_uart[write_idx] = Rx_data[0];
        write_idx++;
        u16_avail_byte++;
        if (write_idx == 200)
        {
            write_idx = 0;
        }
    }
}


/* USER CODE END 0 */

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
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);
  uart_gets(Rx_data);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//      f_tangent_vel_target =0.4;
//      f_angular_vel_target=0.2;
//    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 65);
//    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 65); 
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

//    HAL_Delay(20);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 50000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 42000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 39;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 50000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE7 PE9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD9 PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
