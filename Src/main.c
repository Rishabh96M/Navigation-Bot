
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Kitchen and back with atan2 flip correction
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
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "stm32f3xx_it.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//communications
uint8_t buffer_rxx[63];
uint8_t mag_buffer[1];


uint8_t f255=0;
uint8_t s255=0;
uint8_t byte1 = 0;
uint8_t byte2 = 0;
uint8_t byte3 = 0;
uint8_t byte4 = 0;
uint8_t byte_counter = 0;

uint8_t fc255=0;
uint8_t sc255=0;
uint16_t c_len=0;
uint16_t coordinate_count;
uint8_t coordinate_buffer[68];
uint8_t coordinate_buffer_test[60];
uint8_t camera_coordinate_buffer[4];

//uint8_t *camera_coordinates, *camera_temp;
//uint8_t camera_coordinate_count = 0;
//uint8_t fcc255 = 0;
//uint8_t cc_len = 0;
//uint8_t ccrecv = 0;

float mag_data;

//via point variables
int coordinates_received = 0;
int imu_data_received = 0;

//square via points
float x_via_points[] = {0.5,  1.0,  2.0,  2.5,  3.5, 2.5, 2.0, 1.0, 0.0};
float y_via_points[] = {0.0, -0.5, -2.0, -2.5, -3.5,-2.5,-2.0,-0.5, 0.0};

uint8_t *coordinates,*temp;
float *x_via_point_ptr,*y_via_point_ptr,*x_via_point_temp,*y_via_point_temp;

//Global coordinates
float Global_X   = 0;
float Global_Y   = 0;
float Global_phi = 0;

float Global_phi_d = 0;
float Global_X_cam = 0;
float Global_Y_cam = 0;
float Global_phi_cam = 0;

float camera_x = 0;
float camera_y = 0;

uint16_t sum_of_array = 0;
uint16_t previous_sum_of_array = 0;

//Camera variables
uint8_t camera_data_available = 0;

//Encoder counters
int32_t EN1_pulse_counter=0;
int32_t EN2_pulse_counter=0;

//time variables
uint32_t tic=0;
uint32_t toc=0;
uint32_t loop_time=1;

//Encoder 1 variables
float EN1_degrees = 0;
float EN1_previous_degrees = 0;
float EN1_velocity = 0;
float EN1_rotations = 0;
float EN1_rotations_prev = 0;
float EN1_rpm = 0;
float EN1_target_rpm = 1;

//Encoder 2 variables
float EN2_degrees = 0;
float EN2_previous_degrees = 0;
float EN2_velocity = 0;
float EN2_rotations = 0;
float EN2_rotations_prev = 0;
float EN2_rpm = 0;
float EN2_target_rpm = 1;

//UART communication buffer
char buffer[20];
char buffer_tx[] = "hello\n\r";

//Navigation Variables
float dl = 0;
float dr = 0;
float dc = 0;
float phi = 0;
float x_pos = 0;
float y_pos = 0;
float x_goal = 0;
float y_goal = 0;
float phi_d = 0;
float tolerance = 0.1; //10 centimeters
float euclidean_distance = 0;
float dt = 0;
float previous_x_pos = 0;
float previous_y_pos = 0;
float previous_phi = 0;
float Vx = 0;
float Vy = 0;
float Vphi = 0;
float Vl = 0;
float Vr = 0;
float V  = 0;
float phi_dot = 0;
float previous_phi_d = 0;

//Control Variables
//Motor 1 parameters
float M1_Kp = 0.01;
float M1_Ki = 0;
float M1_Kd = 0.1;
float M1_error = 0;
float M1_previous_error = 0;
float M1_diff_error = 0;
float M1_int_error = 0;
float M1_dt=0;
float M1_bias = 0;
float M1_update_pwm = 0;
float M1_pwm = 100;
int M1_moving_average[] = {0,0,0,0,0,0};

//Motor 2 parameters
float M2_Kp = 0.01;
float M2_Ki = 0;
float M2_Kd = 0.1;
float M2_error = 0;
float M2_previous_error = 0;
float M2_diff_error = 0;
float M2_int_error = 0;
float M2_dt=0;
float M2_bias = 0;
float M2_update_pwm = 0;
float M2_pwm = 100;
int M2_moving_average[] = {0,0,0,0,0,0};

//BOT orientation  parameters
float BO_Kp = 40;
float BO_Ki = 0;
float BO_Kd = 0;
float BO_error = 0;
float BO_previous_error = 0;
float BO_diff_error = 0;
float BO_int_error = 0;
float BO_dt=0;
float BO_bias = 0;
float BO_update_pwm = 0;
float BO_pwm = 0;

//BOT constants
float wheel_base   = 0.245;
float wheel_radius = 0.0325;
#define PI 3.14159265358979323846
int encoder_direction[2] = {-1,1};
float wheel_velocity = 90; //rpm
float dist, max_rotations;

uint8_t aRxBuffer[1];
uint8_t recv;
uint8_t mode=0;
uint8_t recvBufSize=1;

uint8_t crecv;
uint8_t cRxBuffer[1];
uint8_t cmode=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//void print_values(int);
void update_sensor_data(void);
void update_robot_state(void);
void update_robot_control(void);
void update_euclidean_distance(void);
void get_camera_data_status(void);
void convert_data(void);
void PID_BO(void);
void PID_M1(void);
void PID_M2(void);
void get_coordinates(void);

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  //__HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
  //HAL_UART_Receive_IT(&huart2, coordinates_received, sizeof(coordinates_received));

  HAL_UART_Receive_IT(&huart3, (uint8_t *)cRxBuffer, recvBufSize);
  HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, recvBufSize);

  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,0);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
  /* USER CODE END WHILE */
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,100);
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,100);
	  update_robot_state();
  /* USER CODE BEGIN 3 */
  if(coordinates_received==1 && imu_data_received==1)
	{
	  for(int v=0;v<c_len/4;v++)
	  {
		  x_goal = *(x_via_point_ptr+v);
		  y_goal = *(y_via_point_ptr+v);

//		  x_goal = x_via_points[v];
//		  y_goal = y_via_points[v];

		  if(v==0)
		  {
			  HAL_Delay(35000);
		  }

		  while(1)
			  {
			  	  tic = HAL_GetTick();
			  	  update_sensor_data();
			  	  update_robot_state();
			  	  update_robot_control();
			  	  update_euclidean_distance();

			  	  //get_camera_data_status();
			  	  //if(camera_data_available == 1){convert_data();}
			  	  /*if(camera_data_available==0)
			  	  {
			  		  Global_X = Global_X + Vx*100;
			  		  Global_Y = Global_Y + Vy*100;
			  		  Global_phi = Global_phi + Vphi;
			  	  }*/

	  	  	  	  EN1_previous_degrees = EN1_degrees;
	  	  	  	  EN2_previous_degrees = EN2_degrees;

	  	  	  	  HAL_Delay(1);

	  	  	      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,(int)M1_pwm-3);
	  	  	      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,(int)M2_pwm);

	  	  	  	  loop_time = HAL_GetTick() - tic;
	  	  	  	  if((float)sqrt(pow((double)(y_goal-y_pos),(double)2)+pow((double)(x_goal-x_pos),(double)2))<0.10){break;}
			  }

	  }

	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,100);
	  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,100);

//	  free(x_via_point_ptr);
//	  free(y_via_point_ptr);
//	  free(coordinates);

//	  }
	  //coordinates_received=0;
  }
 }
}
  /* USER CODE END 3 */



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
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

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
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

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
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

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB10 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/*void print_values(choice)
{
	switch(choice)
	{
	//printing dps
	case 1:
		HAL_UART_Transmit(&huart2,buffer_tx,20,100);

	 break;

	//printing rotations
	case 2:
		gcvt((float)x_pos,5,buffer);
		strcat(buffer,"  ");
		HAL_UART_Transmit(&huart2,buffer,20,100);
		buffer[0] = '\0';

		gcvt((float)y_pos,5,buffer);
		strcat(buffer,"  ");
		HAL_UART_Transmit(&huart2,buffer,20,100);
		buffer[0] = '\0';

		gcvt((float)phi,5,buffer);
		strcat(buffer,"\n\r");
		HAL_UART_Transmit(&huart2,buffer,20,100);
		buffer[0] = '\0';
		break;

	//printing Degrees
	case 3:
		gcvt((float)EN1_degrees,3,buffer);
		strcat(buffer,"\n\r");
		HAL_UART_Transmit(&huart2,buffer,20,100);
		buffer[0] = '\0';
		break;

	//printing time elapsed
	case 4:
		gcvt((float)loop_time,3,buffer);
		strcat(buffer,"\n\r");
		HAL_UART_Transmit(&huart2,buffer,20,100);
		buffer[0] = '\0';
		break;

	//print rpm
	case 5:
			gcvt((float)EN1_rpm,3,buffer);
			strcat(buffer,"\n\r");
			HAL_UART_Transmit(&huart2,buffer,20,100);
			buffer[0] = '\0';
			break;

	//print rpm
	case 6:
			gcvt((float)EN2_rpm,3,buffer);
			strcat(buffer,"\n\r");
			HAL_UART_Transmit(&huart2,buffer,20,100);
			buffer[0] = '\0';
			break;
	}
}*/

void update_sensor_data(void)
{
	EN1_degrees   = (float)encoder_direction[0]*((float)EN1_pulse_counter/1600)*360;
	EN1_rotations_prev=EN1_rotations;
    EN1_rotations = (float)encoder_direction[0]*((float)EN1_pulse_counter/1600);
    EN1_velocity  = (float)(EN1_degrees - EN1_previous_degrees);
	EN1_rpm 	  = ((EN1_velocity*1000)/loop_time)/6;

	EN2_degrees   = (float)encoder_direction[1]*((float)EN2_pulse_counter/1600)*360;
	EN2_rotations_prev=EN2_rotations;
	EN2_rotations = (float)encoder_direction[1]*(float)EN2_pulse_counter/1600;
	EN2_velocity  = (float)(EN2_degrees - EN2_previous_degrees);
	EN2_rpm 	  = ((EN2_velocity*1000)/loop_time)/6;
}

void update_robot_state(void)
{
//	//odometry updates
//	dl = EN1_rotations*2*PI*wheel_radius;
//	dr = EN2_rotations*2*PI*wheel_radius;
//	dc = (dl+dr)/2;
//	phi = (dr-dl)/wheel_base;
//	y_pos = dc*sin(phi);
//	x_pos = dc*cos(phi);

	//odometry updates
	dl = (EN1_rotations-EN1_rotations_prev)*2*PI*wheel_radius;
	dr = (EN2_rotations-EN2_rotations_prev)*2*PI*wheel_radius;
	dc = (dl+dr)/2;
	//phi +=(dr-dl)/wheel_base;
	y_pos += dc*sin(phi);
	x_pos += dc*cos(phi);

	//phi = mag_data*(PI/180);

//	Vx = (x_pos - previous_x_pos);
//	Vy = (y_pos - previous_y_pos);
//	Vphi = (phi - previous_phi);
//
//	previous_phi = phi;
//	previous_x_pos = x_pos;
//	previous_y_pos = y_pos;
}

void update_robot_control(void)
{
	previous_phi_d = phi_d;
	phi_d = atan2((double)(y_goal-y_pos),(double)(x_goal-x_pos));

	if(abs(phi_d - previous_phi_d)>=PI)
	{
		if(phi_d < previous_phi_d){phi_d += 2*PI;}
		if(phi_d > previous_phi_d){phi_d -= 2*PI;}
	}

	//Global_phi_d = atan2((double)(y_goal*100-Global_Y),(double)(x_goal*100-Global_X));
	//if(y_goal-y_pos>=0){phi_d = phi_d*(-1);}

	BO_error = phi_d - phi;
	PID_BO();

	if(isnan(BO_update_pwm)){BO_update_pwm=0;}

	//M1_pwm = wheel_velocity + BO_update_pwm;
	//M2_pwm = wheel_velocity - BO_update_pwm;
	//if(M1_pwm>100){M1_pwm=100;}
	//if(M1_pwm<0){M1_pwm=0;}
	//if(M2_pwm>100){M2_pwm=100;}
	//if(M2_pwm<0){M2_pwm=0;}

	//set point velocity
	EN1_target_rpm = wheel_velocity - BO_update_pwm;
	EN2_target_rpm = wheel_velocity + BO_update_pwm;

	//motor rpm testing - all data is in rpm
	//EN1_target_rpm = 90;
	//EN2_target_rpm = 90;

	if(EN1_target_rpm<0){EN1_target_rpm=0;}
	if(EN1_target_rpm>wheel_velocity+30){EN1_target_rpm=wheel_velocity+30;}
	if(EN2_target_rpm<0){EN2_target_rpm=0;}
	if(EN2_target_rpm>wheel_velocity+30){EN2_target_rpm=wheel_velocity+30;}

	M1_error =  abs(EN1_target_rpm) - abs(EN1_rpm);
	M2_error =  abs(EN2_target_rpm) - abs(EN2_rpm);

	PID_M1();
	PID_M2();
}

void update_euclidean_distance(void)
{
 	euclidean_distance=(float)sqrt(pow((double)(y_goal-y_pos),(double)2)+pow((double)(x_goal-x_pos),(double)2));
}

void PID_M1()
{
	//M1_dt = 23;
	//M1_int_error = M1_int_error + (M1_error * M1_dt);
	//(M1_Ki*M1_int_error*M1_dt)  +  + M1_bias;

	M1_diff_error = (M1_error - M1_previous_error)/loop_time;
	M1_update_pwm = (M1_Kp*M1_error) + M1_Kd*(M1_diff_error);

	if(M1_update_pwm>10){M1_update_pwm=10;}
	if(M1_update_pwm<-10){M1_update_pwm=-10;}

	M1_pwm = M1_pwm - M1_update_pwm;
	if(M1_pwm>100){M1_pwm=100;}
	if(M1_pwm<0){M1_pwm=0;}

	M1_previous_error = M1_error;
}

void PID_M2()
{
	//M2_dt = (float)loop_time;
	//M2_int_error = M2_int_error + (M2_error * M2_dt);

	M2_diff_error = (M2_error - M2_previous_error)/loop_time;
	M2_update_pwm = (M2_Kp*M2_error) + (M2_Kd*M2_diff_error);

	//+  (M2_Ki*M2_int_error*M2_dt)  +  + M2_bias;

	if(M2_update_pwm>10){M2_update_pwm=10;}
	if(M2_update_pwm<-10){M2_update_pwm=-10;}

	M2_pwm = M2_pwm - M2_update_pwm;

	if(M2_pwm>100){M2_pwm=100;}
	if(M2_pwm<0){M2_pwm=0;}

	M2_previous_error = M2_error;
}

void PID_BO()
{
	//BO_dt = (float)loop_time;
	//BO_diff_error = (BO_error - BO_previous_error)/BO_dt;
	//BO_int_error = BO_int_error + (BO_error * BO_dt);
	BO_update_pwm = (BO_Kp*BO_error);
	//+  (BO_Ki*BO_int_error*BO_dt)  - (BO_Kd*(BO_diff_error/BO_dt)) + BO_bias;

	//M2_previous_error = M2_error;
}

void compute_moving_average()
{
	for(int k = sizeof(M1_moving_average)/sizeof(M1_moving_average[0])-1; k > 0; k--)
	{
		M1_moving_average[k]=M1_moving_average[k-1];
		M2_moving_average[k]=M2_moving_average[k-1];
	}

	M1_moving_average[0] = M1_pwm;
	M2_moving_average[0] = M2_pwm;

	int M1_SUM = 0;
	int M2_SUM = 0;

	for (int k =0 ; k < sizeof(M2_moving_average)/sizeof(M2_moving_average[0]); k--)
	{
		M1_SUM+=M1_moving_average[k];
		M2_SUM+=M2_moving_average[k];
	}

	M1_pwm = M1_SUM / sizeof(M2_moving_average)/sizeof(M2_moving_average[0]);
	M2_pwm = M2_SUM / sizeof(M2_moving_average)/sizeof(M2_moving_average[0]);
}

void convert_data()
{
	if(buffer_rxx[0]==0 || buffer_rxx[0]==1)
	{
		//camera_data_available=1;
		Global_X = (float)buffer_rxx[1]*10 + ((float)buffer_rxx[2])/10;
		Global_Y = (float)buffer_rxx[4]*10 + ((float)buffer_rxx[5])/10;
		Global_phi = (float)buffer_rxx[7]  + ((float)buffer_rxx[8])/100;

		if(buffer_rxx[0]==1){Global_X   = -1*Global_X;}
		if(buffer_rxx[3]==1){Global_Y   = -1*Global_Y;}
		if(buffer_rxx[6]==1){Global_phi = -1*Global_phi;}

//		Global_X = (Global_X_cam + Global_X)/2;
//		Global_X = (Global_Y_cam + Global_Y)/2;
//		Global_X = (Global_phi_cam + Global_phi)/2;
	}
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	UNUSED(huart);
//	//HAL_UART_Receive_IT(&huart1,buffer_rxx,sizeof(buffer_rxx));
//	//if(mag_buffer>128){mag_buffer-255;}
//	phi = ((float)mag_buffer*360.0)*(PI/180.0);
//	HAL_UART_Receive_IT(&huart1,mag_buffer,sizeof(mag_buffer));
//}

void get_camera_data_status()
{
	//sum_of_array  = sum(buffer_rxx,sizeof(buffer_rxx));
	if(sum_of_array-previous_sum_of_array == 0){camera_data_available  = 0;}
	else{camera_data_available = 1;}
    previous_sum_of_array = sum_of_array;
}

int sum(int arr[], int n)
{
    int sum = 0;
    for (int i = 0; i < n; i++)
    sum += arr[i];
    return sum;
}

void get_mag_orientation()
{
	mag_data = (float)mag_buffer[0] * 360.0;
}

void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */

	HAL_UART_IRQHandler(&huart1,&mode);
	recv=aRxBuffer[0];

	if(mode==1)
		    {
		  	  if(f255==0)//FIRST 255
		  	  {
		  		  if(recv==255)
		  		  {
		  			  f255=1;
		  			  s255=0;
		  			  byte1=255;
		  			  byte2=255;
		  			  byte3=255;
		  			  byte4=255;

		  		  }
		  	  }
		  	  else if(f255==1 && s255==0)//SECOND 255
		  	  {
		  		  if(recv==255)
		  		  {
		  			  s255=1;
		  			  byte1=255;
					  byte2=255;
					  byte3=255;
					  byte4=255;

		  		  }
		  		  else
		  		  {
		  			  f255=0;
		  			  s255=0;
		  			  byte1=255;
					  byte2=255;
					  byte3=255;
					  byte4=255;
		  		  }
		  	  }
		  	  else if(f255==1 && s255==1 && byte1==255)
		  	  {
		  		  if(recv>=0 && recv<=9)
		  		  {
		  			  byte1=recv;
		  			  byte2=255;
		  			  byte3=255;
		  			  byte4=255;
		  		  }
		  		  else
		  		  {
		  			  s255=0;
		  			  f255=0;
		  			  byte1=255;
					  byte2=255;
					  byte3=255;
					  byte4=255;
		  		  }
		  	  }
		  	 else if(f255==1 && s255==1 && byte1<=9 && byte2==255)
		  		{
		  		 if(recv>=0 && recv<=9)
		  			{
		  			  	byte2=recv;
		  			  	byte3=255;
		  			  	byte4=255;
		  			}
		  		else
		  			{
		  		  s255=0;
				  f255=0;
				  byte1=255;
				  byte2=255;
				  byte3=255;
				  byte4=255;
		  			}
		  		}
		  	else if(f255==1 && s255==1 && byte1<=9 && byte2<=9 && byte3==255)
		  		{
		  		if(recv>=0 && recv<=9)
		  			{
		  			byte3=recv;
		  			byte4=255;
		  			}
		  		else
		  			{
		  			s255=0;
		  			f255=0;
		  			byte1=255;
		  			byte2=255;
		  			byte3=255;
		  			byte4=255;
		  			 }
		  		}

		  	else if(f255==1 && s255==1 && byte1<=9 && byte2<=9 && byte3<=9 && byte4==255)
		  		{
		  		if(recv>=0 && recv<=9)
		  			{
		  				byte4=recv;
		  			}
		  		else
		  			 {
		  			  byte4=255;
		  			  byte3=255;
		  			  byte2=255;
		  			  byte1=255;
		  			  s255=0;
		  			  f255=0;
		  			 }

		  		uint16_t recvff = ((uint16_t)byte1*1000)+((uint16_t)byte2*100)+((uint16_t)byte3*10)+((uint16_t)byte4);

		  		float recvf=((float)recvff)/10.0;

		  		if(recvf>180){recvf = recvf-360;}

		  		previous_phi = phi;

		  		phi = recvf*(PI/180);

		  		phi = -1*phi;

		  		if(abs(phi - previous_phi)>=PI)
		  		{
		  			if(phi < previous_phi){phi += 2*PI;}
		  			if(phi > previous_phi){phi -= 2*PI;}
		  		}

		  		imu_data_received = 1;

		  		byte4=255;
		  		byte3=255;
		  		byte2=255;
		  		byte1=255;
		  		s255=0;
		  		f255=0;

		  		}

		    }



	huart1.pRxBuffPtr=aRxBuffer;
	huart1.RxXferCount=recvBufSize;
	if(huart1.RxState==HAL_UART_STATE_READY)
	{
	   HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, recvBufSize);
	}


  /* USER CODE BEGIN USART1_IRQn 1 */
  /* USER CODE END USART1_IRQn 1 */


}
/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2,&mode);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles USART3 global interrupt.
*/
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
	HAL_UART_IRQHandler(&huart3,&cmode);
	crecv=cRxBuffer[0];

	if(cmode==1)
	{
		 if(fc255==0)
			{
			if(crecv==255)
				{
				 fc255=1;
				 }
			}
		 else if(fc255==1 && sc255==0)//SECOND 255
		 	 {
		 	if(crecv==255)
		 	{
		 		sc255=1;
		 		c_len=0;
		 	}
		 	else
		 	{
		 		fc255=0;
		 		sc255=0;
		 		c_len=0;
		 	}
		 	 }
		 else if(fc255==1 && sc255 ==1 && c_len==0)
		 {
			 if(crecv>0)
			 {
				 c_len = crecv;
				 coordinates = (uint8_t *)calloc(c_len,sizeof(uint8_t));
				 temp = coordinates;
			 }
			 else
			 {
				 fc255=0;
				 sc255=0;
				 c_len=0;
			 }
		 }
		 else if(fc255==1 && sc255 ==1 && c_len>0)
		 {
			 //coordinate_buffer[coordinate_count] = crecv;
			 *coordinates = crecv;
			 coordinates++;
			 coordinate_count++;

			 if(coordinate_count==c_len)
			 {
				 coordinates = temp;
				 get_coordinates();
//				 uint8_t ack = 255;
//				 HAL_UART_Transmit(&huart3,&ack,sizeof(ack),100);
//				 fc255 = 0;
//				 sc255 = 0;
//				 c_len = 0;
//				 coordinate_count=0;
			 }
		 }

//		 if(fcc255==0 && fc255==0)
//		 {
//			 if(crecv==244)
//			 	{
//				 fcc255=1;
//			 	}
//		 }
//
//		 else if(fcc255==1 && cc_len == 0)
//		 {
//			 if(crecv>0)
//			 {
//				 cc_len = crecv;
//			 }
//			 else
//			 {
//				 fcc255=0;
//				 cc_len=0;
//			 }
//			 camera_coordinates = (uint8_t *)calloc(cc_len,sizeof(uint8_t));
//			 camera_temp = camera_coordinates;
//		 }
//
//		 else if(fcc255==1 && cc_len > 0)
//		 {
//			 camera_coordinate_buffer[camera_coordinate_count] = crecv;
//			 *camera_coordinates = crecv;
//			 camera_coordinates++;
//			 camera_coordinate_count++;
//		 }


//			if(camera_coordinate_count==cc_len)
//			{
//				camera_coordinates = camera_temp;
//
//				uint16_t X = (*(camera_coordinates+0) <<8 ) | *(camera_coordinates+1) ;
//
//				if(X>32768)
//				{
//					camera_x = ((float)(32768-X))/100;
//				}
//				else
//				{
//					camera_x = ((float)X)/100;
//				}
//
//				uint16_t Y = (*(camera_coordinates+2) << 8) | *(camera_coordinates+3);
//
//				if(Y>32768)
//				{
//					camera_y = ((float)(32768-Y))/100;
//				}
//				else
//				{
//					camera_y = ((float)Y)/100;
//				}
//
//
//			camera_coordinates = camera_temp;
//			fcc255 = 0;
//			cc_len = 0;
//			}

	}
  /* USER CODE BEGIN USART3_IRQn 1 */
	huart3.pRxBuffPtr=aRxBuffer;
	huart3.RxXferCount=recvBufSize;
	if(huart3.RxState==HAL_UART_STATE_READY)
	{
		HAL_UART_Receive_IT(&huart3, (uint8_t *)cRxBuffer, recvBufSize);
	}
  /* USER CODE END USART3_IRQn 1 */
}

//void get_pointer_data()

void get_coordinates(void)
{

	x_via_point_ptr = (float *)calloc(c_len/4,sizeof(float));
	y_via_point_ptr = (float *)calloc(c_len/4,sizeof(float));

	x_via_point_temp = x_via_point_ptr;
	y_via_point_temp = y_via_point_ptr;

	for(int i=0;i<c_len/4;i++)
	{
		uint16_t X = (*(coordinates + (i*4)+0) <<8 ) | *(coordinates + (i*4)+1) ;

	    if(X>32768)
	    {
	    	*x_via_point_ptr = ((float)(32768-X))/100;
	    	x_via_point_ptr++;
	    }
	    else
		{
	    	*x_via_point_ptr = ((float)X)/100;
	    	x_via_point_ptr++;
		}

	    uint16_t Y = (*(coordinates + (i*4)+2) << 8) | *(coordinates + (i*4)+3);

	    if(Y>32768)
	    {
	    	*y_via_point_ptr = ((float)(32768-Y))/100;
	    	y_via_point_ptr++;
	    }
	    else
	    {
	    	*y_via_point_ptr = ((float)Y)/100;
	    	y_via_point_ptr++;
	    }
	}

	coordinates_received = 1;

	coordinates = temp;
	x_via_point_ptr = x_via_point_temp;
	y_via_point_ptr = y_via_point_temp;

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
