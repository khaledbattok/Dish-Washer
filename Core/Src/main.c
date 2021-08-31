/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define buffer_size 100
#define eeprom_read_address 0xA0
#define eeprom_write_address 0xA0
#define flow_time_limit     120 //sec
#define temp_rise_time_limit 10 //min

#define Pred_Time_1 2*60+49
#define Pred_Time_2 2*60+58
#define Pred_Time_3 2*60+27
#define Pred_Time_4 1*60
#define Pred_Time_5 36
#define Pred_Time_6 11
#define NumOfTimers 5
#define FlowPulsesReq 3000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t Pred_Time[6]={Pred_Time_1,Pred_Time_2,Pred_Time_3,Pred_Time_4,Pred_Time_5,Pred_Time_6};
uint8_t SavedInformation[20];
uint16_t temp_arr[1];
int flow_meter_count, door, o_f;
uint8_t rec_data[1];
uint8_t inputSerial[64];
int level, count, sizes, checksum, LM, CounterParser;
uint8_t Data_Sending[64];
int Count_Send;
uint16_t adc_buffer[buffer_size];
float adc_avg_buffer[4];
int adc_count;
float sum_adc;
float adc_temp;
int tim3_flag, tim4_flag;
int level_work;
float temp;
int door_count, o_f_count;
struct {
  int counter_value;
  int counter_Enable;
} counters[NumOfTimers];
int counter, counter_flag, counter_1,counter_flag_1,counter_2,counter_flag_2,fcn_done;
int temp_offset;
struct serial_com_info
{
	int req;
	int grant;
	int status;
	int value;
}test_mode,main_mode,power_off,start_info,selecting_button,pill,Timer,WhaterHardness;
struct Error_st
{
	int opened_door_e1;
	int inlet_water_e2;
	int drain_alarm_e3;
	int sensor_alarm_e4;
	int overflow_e5;
	int water_leackge_e6;
	int heater_alarm_e7;
} Error;
int o_f_flag,o_f_time,o_f_count_flag;
int reset_device_req;
int reset_errors_req;
uint8_t done_flag_1;
uint8_t done_flag_2;
int WashingTime;
int buzzer_flag;
int buzzer_flag_count;
int Seg_Value;
int washing_state,dry_state,mode_done,wait_flag,wait_time;
int limit=20;
int pwr,pwr_1,pwr_2,max_pwr;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void send_to_display(void);
void protocolParser(void);
void sendProtocol(UART_HandleTypeDef *huart, uint8_t command, uint8_t *Input_Data, uint8_t inSize);
float adc_sort(uint16_t *adc_arr, int size);
float adc2temp(float x);
uint8_t test_fcn(int n);
void reset_all(void);
void checking_main_errors(void);
int error_handle_task(void);
void reset_errors(void);
int num_of_error(void);
int get_water_level(void);
int get_over_flow_status(void);
int get_door_status(void);
void convert_data(void);
void init_config(void);
void Calc_Temp(void);
uint8_t TakeTheWater(int FlowMeterPulsesRequired,int TimeOut,int TimerNum,int Error_Checking);
uint8_t TakeTheDispenser(int TimeOut,int TimerNum);
uint8_t DoTheWashing(int TimeOut,int TimerNum);
uint8_t BringTheWaterTemperatureToTheSpecifiedValue(int TempSetPoint,int TimeOut,int TimerNum);
uint8_t DrainTheWater(int TimeOut,int TimerNum);
uint8_t AddSaltWater(int TimeOut,int TimerNum);
uint8_t GenerateBuzzerSound(int NumOfSounds,int TimerNum);
uint8_t Wait(int TimeOut,int TimerNum);
void StartTimer(int TimerNum);
void StopTimer(int TimerNum);
void ResetTimer(int TimerNum);
int GetTimerValue(int TimerNum);
int GetTimerCount(int TimerNum);
void OpenInletValve(void);
void CloseInletValve(void);
void OpenDispenserValve(void);
void CloseDispenserValve(void);
void OpenSaltValve(void);
void CloseSaltValve(void);
void TurnOnTheWashingPump(void);
void TurnOffTheWashingPump(void);
void TurnOnHeater(void);
void TurnOffHeater(void);
void TurnOnTheDrainPump(void);
void TurnOffTheDrainPump(void);
float GetWaterTemp(void);
void TurnOnBuzzer(void);
void TurnOffBuzzer(void);
void ResetTimers(void);
void ProcessMain(int ProgramNum);
void beep(void);
void SetTimeValue(int TimerNum,int SetTimeValue);
void main_program_1(void);
void main_program_2(void);
void main_program_3(void);
void main_program_4(void);
void main_program_5(void);
void main_program_6(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	init_config();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(Error.opened_door_e1 && get_door_status())
		{
			Error.opened_door_e1=0;
			wait_flag=1;
		}
    if(Timer.status && (start_info.status==1))
    {
      wait_flag=2;
      if(Wait(60*60,1))
      {
				Timer.value--;
      }
			if(Timer.value==0)
			{
				Timer.status=0;
        wait_flag=0;
				ResetTimer(3);
			}
    }
		checking_main_errors();
		reset_device_req=error_handle_task();
		if(reset_errors_req)
			reset_errors();
		if(num_of_error()==0)
		{
			if(test_mode.status)
			{
				if(start_info.status==1)
					if(fcn_done==0)
						if(test_fcn(test_mode.value))
							fcn_done=1;
				if(selecting_button.status && selecting_button.grant==0)
				{
					if(test_mode.value!=selecting_button.value)
						fcn_done=0;
					reset_all();
					ResetTimers();
					test_mode.value=selecting_button.value;
					selecting_button.status=0;
				}
			}
			else if(main_mode.status)
			{
				Seg_Value=Pred_Time[main_mode.value-1]-GetTimerValue(3)/60;
				if(start_info.status==1)
				{
					if(get_door_status()==0)
					{
						Error.opened_door_e1=1;
					}
					if(wait_flag==0)
						ProcessMain(main_mode.value);
				}
				else if(start_info.status==0)
				{
					reset_all();
				}
				else if(start_info.status==2)
				{
					reset_all();
					//level_work=0;
				}
				if(selecting_button.status && selecting_button.grant==0)
				{
					Timer.status=0;
					Timer.value=0;
					main_mode.value=selecting_button.value;
					selecting_button.status=0;
					level_work=0;
					mode_done=0;
				}
		}
	}		
  if(CounterParser>0)
  {
    protocolParser();
    convert_data();
    CounterParser--;
  }
  
  if (tim3_flag)
  {
    Calc_Temp();
    tim3_flag = 0;
  }
	if(buzzer_flag)
	{
		beep();
		buzzer_flag=0;
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim1)
  {
		send_to_display();
    HAL_GPIO_TogglePin(heart_GPIO_Port, heart_Pin);

    for(int i=0;i<NumOfTimers;i++)
    {
      if(counters[i].counter_Enable==1)
        counters[i].counter_value++;
      else if(counters[i].counter_Enable==-1 || counters[i].counter_Enable==255)
        counters[i].counter_Enable=0;

    }
		if(buzzer_flag_count<2)
			buzzer_flag_count++;
    counters[2].counter_Enable=(counters[0].counter_Enable==1) || (counters[1].counter_Enable==1);
		if(wait_flag)
			wait_time++;
		if(wait_time>=10 && wait_flag==1)
		{
			wait_flag=0;
			wait_time=0;
		}
  }
  if (htim == &htim3)
  {
		if(o_f_flag)
			o_f_time++;
		o_f_count_flag=1;
		if(o_f_time>=20)
		{
			o_f=o_f_count>8;
			o_f_flag=(o_f)?1:0;
			o_f_time=0;
			o_f_count=0;
		}

    if (door_count < 26)
      door_count++;
    
    if (door_count < 20 )
      door = 1;
    else
      door = 0;
    
    tim3_flag = 1;
  }
	if(htim == &htim4)
	{
		pwr_1++;
		pwr_2+=HAL_GPIO_ReadPin(power_flag_GPIO_Port,power_flag_Pin);
		if(pwr_1>=40)
		{
			pwr_1=0;
			pwr=pwr_2;
			pwr_2=0;
			if(pwr>=35)
			{
				if(main_mode.status==1)
				{
					SavedInformation[0]=main_mode.status;
					SavedInformation[1]=main_mode.value;
					SavedInformation[2]=start_info.status;
					SavedInformation[3]=level_work;
					SavedInformation[4]=counters[2].counter_value;
					SavedInformation[5]=counters[2].counter_value>>8;
					SavedInformation[6]=counters[0].counter_value;
					SavedInformation[7]=counters[0].counter_value>>8;
					SavedInformation[8]=flow_meter_count;
					SavedInformation[9]=flow_meter_count>>8;
					SavedInformation[10]=WhaterHardness.status;
					HAL_I2C_Mem_Write(&hi2c1,eeprom_write_address,8,I2C_MEMADD_SIZE_16BIT,SavedInformation,11,HAL_MAX_DELAY);
				}
			}
		}
	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2)
    flow_meter_count++;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    inputSerial[count] = rec_data[0];
    if (inputSerial[0] == 0x5A && level == 0)
    {
      count++;
      if (count > 1)
        level = 1;
    }
    else if (inputSerial[0] != 0x5A && level == 0)
    {
      count = 0;
      level = 0;
    }
    else if (level == 1 && inputSerial[1] == 0xA5)
    {
      count++;
      if (count > 2)
        level = 2;
    }
    else if (level == 1 && inputSerial[1] != 0xA5)
    {
      count = 0;
      level = 0;
    }
    else if (level == 2)
    {
      sizes = inputSerial[3] + 5;
      if (sizes < 32 && sizes > 0)
      {
        count++;
        if (count > 3)
          level = 3;
      }
      else
      {
        count = 0;
        level = 0;
        sizes = 0;
      }
    }
    else if (level == 3)
      count++;
    if (count == sizes && level == 3)
    {
      checksum = 0;
      for (LM = 2; LM < (sizes - 1); LM++)
        checksum += inputSerial[LM];
      if (checksum == inputSerial[sizes - 1])
      {
        CounterParser++;
      }
      else
      {
        for (LM = 0; LM < sizes; LM++)
          inputSerial[LM] = 0;
      }
      count = 0;
      level = 0;
      sizes = 0;
    }
    HAL_UART_Receive_IT(&huart1, rec_data, 1);
  }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == door_Pin)
  {
    door_count = 0;
  }
  if (GPIO_Pin == o_f_Pin)
  {
		if(o_f_count_flag)
			o_f_count++;
		o_f_count_flag=0;
		o_f_flag=1;
  }
}
void sendProtocol(UART_HandleTypeDef *huart, uint8_t command, uint8_t *Input_Data, uint8_t inSize)
{
  uint8_t Send_Serial[64], m, size;
  size = inSize + 5;
  Send_Serial[0] = 0x5A;
  Send_Serial[1] = 0xA5;
  Send_Serial[2] = command;
  Send_Serial[3] = inSize;
  Send_Serial[4 + inSize] = Send_Serial[2] + Send_Serial[3];
  for (m = 0; m < inSize; m++)
  {
    Send_Serial[4 + m] = Input_Data[m];
    Send_Serial[4 + inSize] += Input_Data[m];
  }
  HAL_UART_Transmit(huart, Send_Serial, size, 200);
}
void send_to_display(void)
{
	Data_Sending[Count_Send++] = test_mode.grant;
	Data_Sending[Count_Send++] = test_mode.status;
	Data_Sending[Count_Send++] = test_mode.value;
	Data_Sending[Count_Send++] = main_mode.grant;
	Data_Sending[Count_Send++] = main_mode.status;
	Data_Sending[Count_Send++] = main_mode.value;
	Data_Sending[Count_Send++] = power_off.grant;
	Data_Sending[Count_Send++] = power_off.status;
	Data_Sending[Count_Send++] = start_info.grant;
	Data_Sending[Count_Send++] = start_info.status;
	Data_Sending[Count_Send++] = selecting_button.grant;
	Data_Sending[Count_Send++] = selecting_button.status;
	Data_Sending[Count_Send++] = num_of_error();
	Data_Sending[Count_Send++] = Seg_Value;
  Data_Sending[Count_Send++] = HAL_GPIO_ReadPin(rins_GPIO_Port, rins_Pin);
  Data_Sending[Count_Send++] = HAL_GPIO_ReadPin(salt_GPIO_Port, salt_Pin);
	Data_Sending[Count_Send++] = washing_state;
	Data_Sending[Count_Send++] = dry_state;	
  Data_Sending[Count_Send++] = mode_done;
  Data_Sending[Count_Send++] = Timer.grant;
  Data_Sending[Count_Send++] = Timer.status;
  Data_Sending[Count_Send++] = Timer.value;
  Data_Sending[Count_Send++] = WhaterHardness.grant;
  Data_Sending[Count_Send++] = WhaterHardness.status;
  Data_Sending[Count_Send++] = temp;
	
  sendProtocol(&huart1, 0, Data_Sending, Count_Send);
  Count_Send = 0;
  test_mode.grant=0;
	main_mode.grant=0;
	power_off.grant=0;
	start_info.grant=0;
	selecting_button.grant=0;
  Timer.grant=0;
  WhaterHardness.grant=0;
}
void protocolParser(void)
{
	int index=4;
	test_mode.req=inputSerial[index++];
	main_mode.req=inputSerial[index++];
	power_off.req=inputSerial[index++];
	power_off.value=inputSerial[index++];
	start_info.req=inputSerial[index++];
	start_info.value=inputSerial[index++];
	selecting_button.req=inputSerial[index++];
	selecting_button.value=inputSerial[index++];
	pill.req=inputSerial[index++];
  Timer.req=inputSerial[index++];
  WhaterHardness.req=inputSerial[index++];
  WhaterHardness.value=inputSerial[index++];
}
float adc_sort(uint16_t *adc_arr, int size)
{
  int arr_i, arr_j;
  for (arr_i = 0; arr_i < size - 1; arr_i++)
  {
    for (arr_j = 0; arr_j < size - arr_i - 1; arr_j++)
    {
      if (adc_arr[arr_j] > adc_arr[arr_j + 1])
      {
        adc_arr[arr_j] = adc_arr[arr_j] + adc_arr[arr_j + 1];
        adc_arr[arr_j + 1] = adc_arr[arr_j] - adc_arr[arr_j + 1];
        adc_arr[arr_j] = adc_arr[arr_j] - adc_arr[arr_j + 1];
      }
    }
  }
  return (adc_arr[size / 2 - 2] + adc_arr[size / 2 - 1] + adc_arr[size / 2 - 0] + adc_arr[size / 2 + 1]) * 0.25;
}
float adc2temp(float x)
{
  return 0.0163 * x + 7.8059 + temp_offset;
}
uint8_t test_fcn(int n)
{
	if(get_door_status()==0)
	{
		Error.opened_door_e1=1;
	}
	else
		Error.opened_door_e1=0;
	if(num_of_error()==0)
	{
    switch (n)
    {
    case 9:
      return TakeTheWater(FlowPulsesReq,flow_time_limit,1,1);
      break;
    case 8:
      return TakeTheDispenser(3,1);
      break;
    case 7:
      return DoTheWashing(30,1);
      break;
    case 6:
      return BringTheWaterTemperatureToTheSpecifiedValue(60,temp_rise_time_limit,1);
      break;
    case 5:
      return DrainTheWater(30,1);
      break;
    case 4:
      return TakeTheWater(FlowPulsesReq,flow_time_limit,1,1);
      break;
    case 3:
      return DoTheWashing(10,1);
      break;
    case 2:
      return AddSaltWater(90,1);
      break;
    case 1:
      return DrainTheWater(30,1);
      break;
    case 0:
    return GenerateBuzzerSound(2,1);
      break;
    default:
      return 0;
      break;
    }
  }
}
void reset_all(void)
{
  CloseInletValve();
  CloseDispenserValve();
  TurnOffTheDrainPump();
  TurnOffTheWashingPump();
  CloseSaltValve();
	StopTimer(1);
  StopTimer(2);
}
void ResetTimers(void)
{
	ResetTimer(1);
	ResetTimer(2);
}
void checking_main_errors(void)
{
	if(get_door_status() && get_over_flow_status()==1){
		Error.overflow_e5=1;
	}
	
	if(temp>80)
	{
		Error.sensor_alarm_e4=1;
	}
	
}
int error_handle_task(void)
{
	int ans=0;
	if(Error.opened_door_e1 || Error.inlet_water_e2 || Error.drain_alarm_e3 || Error.overflow_e5 || Error.sensor_alarm_e4 || Error.water_leackge_e6 || Error.heater_alarm_e7)
	{
		beep();
		reset_all();
		ans=1;
	}
	return (Error.overflow_e5 || Error.sensor_alarm_e4);
}
int num_of_error(void)
{
	if(Error.overflow_e5)
		return 5;
	if(Error.sensor_alarm_e4)
		return 4;
	if(Error.opened_door_e1)
		return 1;
	if(Error.inlet_water_e2)
		return 2;
	if(Error.drain_alarm_e3)
		return 3;
	if(Error.water_leackge_e6)
		return 6;
	if(Error.heater_alarm_e7)
		return 7;
	return 0;
}
void reset_errors(void)
{
	Error.drain_alarm_e3=0;
	Error.heater_alarm_e7=0;
	Error.inlet_water_e2=0;
	Error.water_leackge_e6=0;
	reset_errors_req=0;
}
int get_water_level(void)
{
	return (!HAL_GPIO_ReadPin(w_l_GPIO_Port,w_l_Pin));
}
int get_over_flow_status(void)
{
	return o_f;
}
int get_door_status(void)
{
	return door;
}
void convert_data(void)
{
	if(test_mode.req)
	{
		if(buzzer_flag_count==2)
		{
			buzzer_flag=1;
			buzzer_flag_count=0;
		}
		test_mode.status=1;
		test_mode.grant=1;
		test_mode.req=0;
	}
	if(main_mode.req)
	{
		if(buzzer_flag_count==2)
		{
			buzzer_flag=2;
			buzzer_flag_count=0;
		}
		main_mode.status=1;
		main_mode.grant=1;
		main_mode.req=0;
	}
	if(power_off.req)
	{
		if(buzzer_flag_count==2)
		{
			buzzer_flag=3;
			buzzer_flag_count=0;
		}
		power_off.status=power_off.value;
		power_off.grant=1;
		power_off.req=0;
		if(power_off.status)
		{
			test_mode.status=0;
			main_mode.status=0;
			start_info.status=0;
      Timer.status=0;
      Timer.value=0;
			washing_state=0;
			main_mode.value=2;
      level_work=0;
			reset_all();
			reset_errors();
			Error.opened_door_e1=0;
      ResetTimer(3);
		}
	}
	if(start_info.req)
	{
		if(buzzer_flag_count==2)
		{
			buzzer_flag=4;
			buzzer_flag_count=0;
		}
		start_info.status=start_info.value;
		start_info.grant=1;
		start_info.req=0;
		reset_errors();	
	}
	if(selecting_button.req)
	{
		if(buzzer_flag_count==2)
		{
			buzzer_flag=5;
			buzzer_flag_count=0;
		}
		selecting_button.status=1;
		selecting_button.grant=1;
		selecting_button.req=0;
	}
  if(Timer.req)
  {
    if(buzzer_flag_count==2)
		{
			buzzer_flag=6;
			buzzer_flag_count=0;
      if(Timer.value<24)
      {
        Timer.status=1;
        Timer.value++;
      }
      else
      {
        Timer.status=0;
        Timer.value=0;
      }
		}
    Timer.grant=1;
    Timer.req=0;
  }
  if(WhaterHardness.req)
  {
    if(buzzer_flag_count==2)
		{
			buzzer_flag=7;
			buzzer_flag_count=0;
      WhaterHardness.status=WhaterHardness.value;
			SavedInformation[10]=WhaterHardness.status;
			HAL_I2C_Mem_Write(&hi2c1,eeprom_write_address,8,I2C_MEMADD_SIZE_16BIT,SavedInformation,11,HAL_MAX_DELAY);
    }
    WhaterHardness.grant=1;
    WhaterHardness.req=0;
  }
}
void init_config(void)
{
	HAL_I2C_Mem_Read(&hi2c1,eeprom_read_address,8,I2C_MEMADD_SIZE_16BIT,SavedInformation,20,HAL_MAX_DELAY);
	if(SavedInformation[0]==1)
	{
		main_mode.status          =  SavedInformation[0];
		main_mode.value           =  SavedInformation[1];
		start_info.status         =  SavedInformation[2];
		level_work                =  SavedInformation[3];
		counters[2].counter_value =  SavedInformation[4]+SavedInformation[5]*256;
		counters[0].counter_value =  SavedInformation[6]+SavedInformation[7]*256;
    flow_meter_count          =  SavedInformation[8]+SavedInformation[9]*256;
		for(int i=0;i<20;i++)
		{
			SavedInformation[i]=0;
		}
		HAL_Delay(10);
		HAL_I2C_Mem_Write(&hi2c1,eeprom_write_address,8,I2C_MEMADD_SIZE_16BIT,SavedInformation,20,HAL_MAX_DELAY);
	}
	else
	{
		main_mode.value=2;
		power_off.status=1;
		start_info.status=0;
	}
	if(SavedInformation[10]>=1 && SavedInformation[10]<=8)
		WhaterHardness.status=SavedInformation[10];
	else
		WhaterHardness.status=5;
	power_off.grant=1;
	start_info.grant=1;
	selecting_button.grant=1;
	Timer.grant=1;
	Timer.status=0;
	test_mode.grant=1;
	main_mode.grant=1;
	test_mode.value=9;
	HAL_ADC_Start(&hadc1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim1);
	HAL_UART_Receive_IT(&huart1, rec_data, 1);
	beep();
}
void Calc_Temp(void)
{
	sum_adc += HAL_ADC_GetValue(&hadc1);
	adc_count++;
	if (adc_count >= buffer_size)
	{
		adc_temp = sum_adc * 0.01;
		temp = adc2temp(adc_temp);
		adc_count = 0;
		sum_adc = 0;
	}
}
void ProcessMain(int ProgramNum)
{
  switch(ProgramNum)
    {
      case 1:
        main_program_1();
      break;
      case 2:
        main_program_2();
      break;
      case 3:
        main_program_3();
      break;
      case 4:
        main_program_4();
      break;
      case 5:
				main_program_5();
      break;
      case 6:
        main_program_6();
      break;
    }
}
void main_program_1(void)
{
  washing_state=(level_work!=-1) &&  (level_work<21) ;
  dry_state=(level_work>=21) && (level_work<35);
  if(level_work==0)
  {
    washing_state=1;
    StartTimer(3);
    if(Wait(10,1))
    {
      level_work=1;
    }
  }
  else if(level_work==1)
  {
    if(DrainTheWater(30,1))
    {
      level_work=2;
    }
  }
  else if(level_work==2)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,1,0))
    {
      case 1:
        level_work=3;
        break;
      case 2:
        level_work=-2;
        break;
    }
  }
  else if(level_work==-2)
  {
    if(DrainTheWater(30,4))
    {
      level_work=-3;
    }    
  }
  else if(level_work==-3)
  {
    if(Wait(3*60,4))
    {
      level_work=-4;
    }
  }
  else if(level_work==-4)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,4,1))
    {
      case 1:
        level_work=3;
        break;
      case 2:
        break;
    }
  }
  else if(level_work==3)
  {
    if(DoTheWashing(10,1))
    {
      level_work=4;
      WashingTime=14*60-GetTimerValue(3);
    }
  }
  else if(level_work==4)
  {
    if(WashingTime==0)
    {
      WashingTime=14*60-GetTimerValue(3);
      ResetTimer(1);
    }
    if(BringTheWaterTemperatureToTheSpecifiedValue(40,WashingTime,1))
    {
      level_work=5;
      SetTimeValue(3,14*60);
      WashingTime=0;
    }
  }
  else if(level_work==5)
  {
    if(DoTheWashing(5*60,1))
    {
      level_work=6;
    }
  }
  else if(level_work==6)
  {
    if(DrainTheWater(30,1))
    {
      level_work=7;
    }
  }
  else if(level_work==7)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,1,0))
    {
      case 1:
        level_work=8;
        break;
      case 2:
        level_work=-7;
        break;
    }
  }
  else if(level_work==-7)
  {
    if(DrainTheWater(30,4))
    {
      level_work=-8;
    }    
  }
  else if(level_work==-8)
  {
    if(Wait(3*60,4))
    {
      level_work=-9;
    }
  }
  else if(level_work==-9)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,4,1))
    {
      case 1:
        level_work=8;
        break;
      case 2:
        break;
    }
  }
  else if(level_work==8)
  {
    if(DoTheWashing(10,1))
    {
      level_work=9;
      WashingTime=20*60-(GetTimerValue(3)-(5+14)*60);
    }
  }
  else if(level_work==9)
  {
    if(WashingTime==0)
    {
      WashingTime=20*60-(GetTimerValue(3)-(5+14)*60);
      SetTimeValue(2,GetTimerValue(1));
      ResetTimer(1);
    }
    if(done_flag_1==0)
      done_flag_1=BringTheWaterTemperatureToTheSpecifiedValue(60,WashingTime,1);
    if(done_flag_2==0)
      done_flag_2=TakeTheDispenser(90,2);
    if(done_flag_1 && done_flag_2)
    {
      level_work=10;
      SetTimeValue(3,(14+5+20)*60);
      done_flag_2=0;
      done_flag_1=0;
      WashingTime=0;
    }
  }
  else if(level_work==10)
  {
    if(DoTheWashing(35*60,1))
    {
      level_work=11;
    }
  }
 else if(level_work==11)
  {
    if(DrainTheWater(30,1))
    {
      level_work=12;
    }
  }
  else if(level_work==12)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,1,0))
    {
      case 1:
        level_work=13;
        WashingTime=7*60-(GetTimerValue(3)-(35+20+5+14)*60);
        break;
      case 2:
        level_work=-12;
        break;
    }
  }
  else if(level_work==-12)
  {
    if(DrainTheWater(30,4))
    {
      level_work=-13;
    }    
  }
  else if(level_work==-13)
  {
    if(Wait(3*60,4))
    {
      level_work=-14;
    }
  }
  else if(level_work==-14)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,4,1))
    {
      case 1:
        level_work=13;
        WashingTime=7*60-(GetTimerValue(3)-(35+20+5+14)*60);
        break;
      case 2:
        break;
    }
  }
  else if(level_work==13)
  {
    if(WashingTime==0)
    {
      ResetTimer(1);
      WashingTime=7*60-(GetTimerValue(3)-(35+20+5+14)*60);
    }
    if(DoTheWashing(WashingTime,1))
    {
      level_work=14;
      WashingTime=0;
    }
  }
  else if(level_work==14)
  {
    if(DrainTheWater(30,1))
    {
      level_work=15;
    }
  }
  else if(level_work==15)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,1,0))
    {
      case 1:
        level_work=16;
        WashingTime=7*60-(GetTimerValue(3)-(7+35+20+5+14)*60);
        break;
      case 2:
        level_work=-15;
        break;
    }
  }
  else if(level_work==-15)
  {
    if(DrainTheWater(30,4))
    {
      level_work=-16;
    }    
  }
  else if(level_work==-16)
  {
    if(Wait(3*60,4))
    {
      level_work=-17;
    }
  }
  else if(level_work==-17)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,4,1))
    {
      case 1:
        level_work=16;
        WashingTime=7*60-(GetTimerValue(3)-(7+35+20+5+14)*60);
        break;
      case 2:
        break;
    }
  }
  else if(level_work==16)
  {
    if(WashingTime==0)
    {
      ResetTimer(1);
      WashingTime=7*60-(GetTimerValue(3)-(7+35+20+5+14)*60);
    }
    if(DoTheWashing(WashingTime,1))
    {
      level_work=17;
      WashingTime=0;
    }
  }
  else if(level_work==17)
  {
    if(DrainTheWater(30,1))
    {
      level_work=18;
    }
  }
  else if(level_work==18)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,1,0))
    {
      case 1:
        level_work=19;
        break;
      case 2:
        level_work=-18;
        break;
    }
  }
  else if(level_work==-18)
  {
    if(DrainTheWater(30,4))
    {
      level_work=-19;
    }    
  }
  else if(level_work==-19)
  {
    if(Wait(3*60,4))
    {
      level_work=-20;
    }
  }
  else if(level_work==-20)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,4,1))
    {
      case 1:
        level_work=19;
        break;
      case 2:
        break;
    }
  }
  else if(level_work==19)
  {
    if(DoTheWashing(10,1))
    {
      level_work=20;
      WashingTime=23*60-(GetTimerValue(3)-(7+7+35+20+5+14)*60);
    }
  }
  else if(level_work==20)
  {
    if(WashingTime==0)
    {
      ResetTimer(1);
      WashingTime=23*60-(GetTimerValue(3)-(7+7+35+20+5+14)*60);
    }
    if(BringTheWaterTemperatureToTheSpecifiedValue(65,WashingTime,1))
    {
      level_work=21;
      SetTimeValue(3,(23+7+7+35+20+5+14)*60);
      WashingTime=0;
    }
  }
  else if(level_work==21)
  {
    if(DoTheWashing(1*60,1))
    {
      level_work=22;
      washing_state=0;
      dry_state=1;
    }
  }
  else if(level_work==22)
  {
    if(Wait(30,1))
    {
      level_work=23;
    }
  }
  else if(level_work==23)
  {
    if(DrainTheWater(30,1))
    {
      level_work=24;
    }    
  }
  else if(level_work==24)
  {
    if(Wait(2*60+30,1))
    {
      level_work=25;
    }
  }
  else if(level_work==25)
  {
    if(DrainTheWater(30,1))
    {
      level_work=26;
    }    
  }
  else if(level_work==26)
  {
    if(Wait(11*60+30,1))
    {
      level_work=27;
    }
  }
  else if(level_work==27)
  {
    if(DrainTheWater(30,1))
    {
      level_work=28;
    }    
  }
  else if(level_work==28)
  {
    if(Wait(20*60+30,1))
    {
      level_work=29;
    }
  }
  else if(level_work==29)
  {
    if(DrainTheWater(30,1))
    {
      level_work=30;
    }    
  }
  else if(level_work==30)
  {
    if(Wait(19*60+30,1))
    {
      level_work=31;
    }
  }
  else if(level_work==31)
  {
    if(DrainTheWater(30,1))
    {
      level_work=32;
    }    
  }
    else if(level_work==32)
  {
    if(Wait(1*60,1))
    {
      level_work=33;
    }
  }
  else if(level_work==33)
  {
    if(DrainTheWater(60,1))
    {
      level_work=34;
    }    
  }
  else if(level_work==34)
  {
    mode_done=1;
    if(GenerateBuzzerSound(4,1))
    {
      dry_state=0;
      level_work=-1;
      start_info.status=0;
			ResetTimer(3);
    }
  }
}
void main_program_2(void)
{
  washing_state=(level_work!=-1) &&  (level_work<13) ;
  dry_state=(level_work>=13) && (level_work<26);
  if(level_work==0)
  {
    washing_state=1;
    StartTimer(3);
    if(Wait(10,1))
    {
      level_work=1;
    }
  }
  else if(level_work==1)
  {
    if(DrainTheWater(30,1))
    {
      level_work=2;
    }
  }
  else if(level_work==2)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,1,0))
    {
      case 1:
        level_work=3;
	      WashingTime=26*60-GetTimerValue(3);
        break;
      case 2:
        level_work=-2;
        break;
    }
  }
    else if(level_work==-2)
    {
      if(DrainTheWater(30,4))
      {
        level_work=-3;
      }    
    }
    else if(level_work==-3)
    {
      if(Wait(3*60,4))
      {
        level_work=-4;
      }
    }
    else if(level_work==-4)
    {
      switch(TakeTheWater(FlowPulsesReq,flow_time_limit,4,1))
      {
        case 1:
          level_work=3;
	        WashingTime=26*60-GetTimerValue(3);
          break;
        case 2:
          break;
      }
    }
  else if(level_work==3)
  {
    if(WashingTime==0)
    {
      WashingTime=26*60-GetTimerValue(3);
      ResetTimer(1);
    }
    if(DoTheWashing(WashingTime,1))
    {
      level_work=4;
      WashingTime=0;
    }
  }
  else if(level_work==4)
  {
    if(DrainTheWater(30,1))
    {
      level_work=5;
    }
  }
  else if(level_work==5)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,1,0))
    {
      case 1:
        level_work=6;
        break;
      case 2:
        level_work=-5;
        break;
    }
  }
    else if(level_work==-5)
    {
      if(DrainTheWater(30,4))
      {
        level_work=-6;
      }    
    }
    else if(level_work==-6)
    {
      if(Wait(3*60,4))
      {
        level_work=-7;
      }
    }
    else if(level_work==-7)
    {
      switch(TakeTheWater(FlowPulsesReq,flow_time_limit,4,1))
      {
        case 1:
          level_work=6;
          break;
        case 2:
          break;
      }
  }
  else if(level_work==6)
  {
    if(DoTheWashing(10,1))
    {
      level_work=7;
      WashingTime=18*60-(GetTimerValue(3)-26*60);
    }
  }
  else if(level_work==7)
  {
    if(WashingTime==0)
    {
      WashingTime=18*60-(GetTimerValue(3)-26*60);
      SetTimeValue(2,GetTimerValue(1));
      ResetTimer(1);
    }
    if(done_flag_1==0)
      done_flag_1=BringTheWaterTemperatureToTheSpecifiedValue(45,WashingTime,1);
    if(done_flag_2==0)
      done_flag_2=TakeTheDispenser(90,2);
    if(done_flag_1 && done_flag_2)
    {
      level_work=8;
      SetTimeValue(3,(18+26)*60);
      done_flag_2=0;
      done_flag_1=0;
      WashingTime=0;
    }
  }
  else if(level_work==8)
  {
    if(DoTheWashing(60*60,1))
    {
      level_work=9;
    }
  }
   else if(level_work==9)
  {
    if(DrainTheWater(30,1))
    {
      level_work=10;
    }
  }


  else if(level_work==10)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,1,0))
    {
      case 1:
        level_work=11;
        break;
      case 2:
        level_work=-10;
        break;
    }
  }
    else if(level_work==-10)
    {
      if(DrainTheWater(30,4))
      {
        level_work=-11;
      }    
    }
    else if(level_work==-11)
    {
      if(Wait(3*60,4))
      {
        level_work=-12;
      }
    }
    else if(level_work==-12)
    {
      switch(TakeTheWater(FlowPulsesReq,flow_time_limit,4,1))
      {
        case 1:
          level_work=11;
          break;
        case 2:
          break;
      }
  }
  else if(level_work==11)
  {
    if(DoTheWashing(10,1))
    {
      level_work=12;
      WashingTime=16*60-(GetTimerValue(3)-(60+18+26)*60);
    }
  }
  else if(level_work==12)
  {
    if(WashingTime==0)
    {
      WashingTime=16*60-(GetTimerValue(3)-(60+18+26)*60);
      ResetTimer(1);
    }
    if(BringTheWaterTemperatureToTheSpecifiedValue(60,WashingTime,1))
    {
      level_work=13;
      SetTimeValue(3,(16+60+18+26)*60);
      WashingTime=0;
    }
  }
  else if(level_work==13)
  {
    if(DoTheWashing(1*60,1))
    {
      level_work=14;
      washing_state=0;
      dry_state=1;
    }
  }
  else if(level_work==14)
  {
    if(DrainTheWater(30,1))
    {
      level_work=15;
    }    
  }
  else if(level_work==15)
  {
    if(Wait(2*60+30,1))
    {
      level_work=16;
    }
  }
  else if(level_work==16)
  {
    if(DrainTheWater(30,1))
    {
      level_work=17;
    }    
  }
  else if(level_work==17)
  {
    if(Wait(11*60+30,1))
    {
      level_work=18;
    }
  }
  else if(level_work==18)
  {
    if(DrainTheWater(30,1))
    {
      level_work=19;
    }    
  }
  else if(level_work==19)
  {
    if(Wait(20*60+30,1))
    {
      level_work=20;
    }
  }
  else if(level_work==20)
  {
    if(DrainTheWater(30,1))
    {
      level_work=21;
    }    
  }
  else if(level_work==21)
  {
    if(Wait(19*60+30,1))
    {
      level_work=22;
    }
  }
  else if(level_work==22)
  {
    if(DrainTheWater(30,1))
    {
      level_work=23;
    }    
  }
  else if(level_work==23)
  {
    if(Wait(1*60,1))
    {
      level_work=24;
    }
  }
  else if(level_work==24)
  {
    if(DrainTheWater(60,1))
    {
      level_work=25;
    }    
  }
  else if(level_work==25)
  {
    mode_done=1;
    if(GenerateBuzzerSound(4,1))
    {
      dry_state=0;
      level_work=-1;
      start_info.status=0;
			ResetTimer(3);
    }
  }
}
void main_program_3(void)
{
  washing_state=(level_work!=-1) &&  (level_work<17) ;
  dry_state=(level_work>=17) && (level_work<25);
  if(level_work==0)
  {
    washing_state=1;
    StartTimer(3);
    if(Wait(10,1))
    {
      level_work=1;
    }
  }
  else if(level_work==1)
  {
    if(DrainTheWater(30,1))
    {
      level_work=2;
    }
  }
  else if(level_work==2)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,1,0))
    {
      case 1:
        level_work=3;
         WashingTime=11*60-GetTimerValue(3);
        break;
      case 2:
        level_work=-2;
        break;
    }
  }
    else if(level_work==-2)
    {
      if(DrainTheWater(30,4))
      {
        level_work=-3;
      }    
    }
    else if(level_work==-3)
    {
      if(Wait(3*60,4))
      {
        level_work=-4;
      }
    }
    else if(level_work==-4)
    {
      switch(TakeTheWater(FlowPulsesReq,flow_time_limit,4,1))
      {
        case 1:
          level_work=3;
          WashingTime=11*60-GetTimerValue(3);
          break;
        case 2:
          break;
      }
    }
  else if(level_work==3)
  {
    if(WashingTime==0)
    {
      WashingTime=11*60-GetTimerValue(3);
      ResetTimer(1);
    }
    if(DoTheWashing(WashingTime,1))
    {
      level_work=4;
      WashingTime=0;
    }
  }
  else if(level_work==4)
  {
    if(DrainTheWater(30,1))
    {
      level_work=5;
    }
  }
  else if(level_work==5)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,1,0))
    {
      case 1:
        level_work=6;
        break;
      case 2:
        level_work=-5;
        break;
    }
  }
    else if(level_work==-5)
    {
      if(DrainTheWater(30,4))
      {
        level_work=-6;
      }    
    }
    else if(level_work==-6)
    {
      if(Wait(3*60,4))
      {
        level_work=-7;
      }
    }
    else if(level_work==-7)
    {
      switch(TakeTheWater(FlowPulsesReq,flow_time_limit,4,1))
      {
        case 1:
          level_work=6;
          break;
        case 2:
          break;
      }
    }
  else if(level_work==6)
  {
    if(DoTheWashing(10,1))
    {
      level_work=7;
      WashingTime=17*60-(GetTimerValue(3)-11*60);
    }
  }
  else if(level_work==7)
  {
    if(WashingTime==0)
    {
      WashingTime=17*60-(GetTimerValue(3)-11*60);
      SetTimeValue(2,GetTimerValue(1));
      ResetTimer(1);
    }
    if(done_flag_1==0)
      done_flag_1=BringTheWaterTemperatureToTheSpecifiedValue(45,WashingTime,1);
    if(done_flag_2==0)
      done_flag_2=TakeTheDispenser(90,2);
    if(done_flag_1 && done_flag_2)
    {
      level_work=8;
      SetTimeValue(3,(17+11)*60);
      done_flag_2=0;
      done_flag_1=0;
      WashingTime=0;
    }
  }
  else if(level_work==8)
  {
    if(DoTheWashing(40*60,1))
    {
      level_work=9;
    }
  }
  else if(level_work==9)
  {
    if(DrainTheWater(30,1))
    {
      level_work=10;
    }
  }
  else if(level_work==10)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,1,0))
    {
      case 1:
        level_work=12;
	      WashingTime=4*60-(GetTimerValue(3)-(40+17+11)*60);
        break;
      case 2:
        level_work=-10;
        break;
    }
  }
    else if(level_work==-10)
    {
      if(DrainTheWater(30,4))
      {
        level_work=-11;
      }    
    }
    else if(level_work==-11)
    {
      if(Wait(3*60,4))
      {
        level_work=-12;
      }
    }
    else if(level_work==-12)
    {
      switch(TakeTheWater(FlowPulsesReq,flow_time_limit,4,1))
      {
        case 1:
          level_work=12;
	        WashingTime=4*60-(GetTimerValue(3)-(40+17+11)*60);
          break;
        case 2:
          break;
      }
  }
  else if(level_work==12)
  {
    if(WashingTime==0)
    {
      WashingTime=4*60-(GetTimerValue(3)-(40+17+11)*60);
      ResetTimer(1);
    }
    if(DoTheWashing(WashingTime,1))
    {
      level_work=13;
      WashingTime=0;
    }
  }
  else if(level_work==13)
  {
    if(DrainTheWater(30,1))
    {
      level_work=14;
    }
  }
  else if(level_work==14)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,1,0))
    {
      case 1:
        level_work=15;
        break;
      case 2:
        level_work=-14;
        break;
    }
  }
    else if(level_work==-14)
    {
      if(DrainTheWater(30,4))
      {
        level_work=-15;
      }    
    }
    else if(level_work==-15)
    {
      if(Wait(3*60,4))
      {
        level_work=-16;
      }
    }
    else if(level_work==-16)
    {
      switch(TakeTheWater(FlowPulsesReq,flow_time_limit,4,1))
      {
        case 1:
          level_work=15;
          break;
        case 2:
          break;
      }
  }
  else if(level_work==15)
  {
    if(DoTheWashing(10,1))
    {
      level_work=16;
      WashingTime=16*60-(GetTimerValue(3)-(4+40+17+11)*60);
    }
  }
  else if(level_work==16)
  {
    if(WashingTime==0)
    {
      WashingTime=16*60-(GetTimerValue(3)-(4+40+17+11)*60);
      ResetTimer(1);
    }
    if(BringTheWaterTemperatureToTheSpecifiedValue(55,WashingTime,1))
    {
      level_work=17;
      SetTimeValue(3,(16+4+40+17+11)*60);
      WashingTime=0;
    }
  }
  else if(level_work==17)
  {
    if(DoTheWashing(1*60,1))
    {
      level_work=18;
      washing_state=0;
      dry_state=1;
    }
  }
  else if(level_work==18)
  {
    if(Wait(2*60+30,1))
    {
      level_work=19;
    }
  }
  else if(level_work==19)
  {
    if(DrainTheWater(30,1))
    {
      level_work=20;
    }    
  }
  else if(level_work==20)
  {
    if(Wait(32*60+30,1))
    {
      level_work=21;
    }
  }
  else if(level_work==21)
  {
    if(DrainTheWater(30,1))
    {
      level_work=22;
    }    
  }
    else if(level_work==22)
  {
    if(Wait(21*60,1))
    {
      level_work=23;
    }
  }
  else if(level_work==23)
  {
    if(DrainTheWater(60,1))
    {
      level_work=24;
    }    
  }
  else if(level_work==24)
  {
    mode_done=1;
    if(GenerateBuzzerSound(4,1))
    {
      dry_state=0;
      level_work=-1;
      start_info.status=0;
			ResetTimer(3);
    }
  }

}
void main_program_4(void)
{ 
  washing_state=(level_work!=-1) &&  (level_work<15) ;
  dry_state=(level_work>=15) && (level_work<19);
  if(level_work==0)
  {
    StartTimer(3);
    if(Wait(10,1))
    {
      level_work=1;
    }
  }
  else if(level_work==1)
  {
    if(DrainTheWater(30,1))
    {
      level_work=2;
    }
  }
  else if(level_work==2)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,1,0))
    {
      case 1:
        level_work=3;
        break;
      case 2:
        level_work=-2;
        break;
    }
  }
    else if(level_work==-2)
    {
      if(DrainTheWater(30,4))
      {
        level_work=-3;
      }    
    }
    else if(level_work==-3)
    {
      if(Wait(3*60,4))
      {
        level_work=-4;
      }
    }
    else if(level_work==-4)
    {
      switch(TakeTheWater(FlowPulsesReq,flow_time_limit,4,1))
      {
        case 1:
          level_work=3;
          break;
        case 2:
          break;
      }
    }
  else if(level_work==3)
  {
    if(DoTheWashing(10,1))
    {
      level_work=4;
      WashingTime=18*60-GetTimerValue(3);
    }
  }
  else if(level_work==4)
  {
    if(WashingTime==0)
    {
      WashingTime=18*60-GetTimerValue(3);
      ResetTimer(1);
    }
    if(BringTheWaterTemperatureToTheSpecifiedValue(50,WashingTime,1))
    {
      level_work=5;
      SetTimeValue(3,18*60);
      WashingTime=0;
    }
  }
  else if(level_work==5)
  {
    if(DoTheWashing(5*60,1))
    {
      level_work=6;
    }
  }
  else if(level_work==6)
  {
    if(DrainTheWater(30,1))
    {
      level_work=7;
    }
  }
  else if(level_work==7)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,1,0))
    {
      case 1:
        level_work=8;
        break;
      case 2:
        level_work=-7;
        break;
    }
  }
    else if(level_work==-7)
    {
      if(DrainTheWater(30,4))
      {
        level_work=-8;
      }    
    }
    else if(level_work==-8)
    {
      if(Wait(3*60,4))
      {
        level_work=-9;
      }
    }
    else if(level_work==-9)
    {
      switch(TakeTheWater(FlowPulsesReq,flow_time_limit,4,1))
      {
        case 1:
          level_work=8;
          break;
        case 2:
          break;
      }
  }

  else if(level_work==8)
  {
    if(DoTheWashing(10,1))
    {
      level_work=9;
      WashingTime=10*60-(GetTimerValue(3)-23*60);
    }
  }
  else if(level_work==9)
  {
    if(WashingTime==0)
    {
      WashingTime=10*60-(GetTimerValue(3)-23*60);
      SetTimeValue(2,GetTimerValue(1));
      ResetTimer(1);
    }
    if(done_flag_1==0)
      done_flag_1=BringTheWaterTemperatureToTheSpecifiedValue(40,WashingTime,1);
    if(done_flag_2==0)
      done_flag_2=TakeTheDispenser(90,2);
    if(done_flag_1 && done_flag_2)
    {
      level_work=10;
      SetTimeValue(3,(18+5+10)*60);
      done_flag_2=0;
      done_flag_1=0;
      WashingTime=0;
    }
  }
  else if(level_work==10)
  {
    if(DoTheWashing(5*60,1))
    {
      level_work=11;
    }
  }
  else if(level_work==11)
  {
    if(DrainTheWater(30,1))
    {
      level_work=12;
    }
  }
  else if(level_work==12)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,1,0))
    {
      case 1:
        level_work=13;
        break;
      case 2:
        level_work=-12;
        break;
    }
  }
    else if(level_work==-12)
    {
      if(DrainTheWater(30,4))
      {
        level_work=-13;
      }    
    }
    else if(level_work==-13)
    {
      if(Wait(3*60,4))
      {
        level_work=-14;
      }
    }
    else if(level_work==-14)
    {
      switch(TakeTheWater(FlowPulsesReq,flow_time_limit,4,1))
      {
        case 1:
          level_work=13;
          break;
        case 2:
          break;
      }
  }

  else if(level_work==13)
  {
    if(DoTheWashing(10,1))
    {
      level_work=14;
      WashingTime=15*60-(GetTimerValue(3)-38*60);
    }
  }
  else if(level_work==14)
  {
    if(WashingTime==0)
    {
      WashingTime=15*60-(GetTimerValue(3)-38*60);
      ResetTimer(1);
    }
    if(BringTheWaterTemperatureToTheSpecifiedValue(60,WashingTime,1))
    {
      level_work=15;
      SetTimeValue(3,(15+5+10+5+18)*60);
      WashingTime=0;
    }
  }
  else if(level_work==15)
  {
    if(DoTheWashing(1*60,1))
    {
      level_work=16;
    }
  }
  else if(level_work==16)
  {
    if(Wait(5*60,1))
    {
      level_work=17;
    }
  }
  else if(level_work==17)
  {
    if(DrainTheWater(60,1))
    {
      level_work=18;
    }    
  }
  else if(level_work==18)
  {
    mode_done=1;
    if(GenerateBuzzerSound(4,1))
    {
      dry_state=0;
      level_work=-1;
      start_info.status=0;
			ResetTimer(3);
    }
  }
  
  }
void main_program_5(void)
{
  washing_state=(level_work!=-1) &&  (level_work<13) ;
  if(level_work==0)
    {
      StartTimer(3);
      if(Wait(10,1))
      {
        level_work=1;
      }
    }
    else if(level_work==1)
    {
      if(DrainTheWater(30,1))
      {
        level_work=2;
      }
    }
    else if(level_work==2)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,1,0))
    {
      case 1:
        level_work=3;
        break;
      case 2:
        level_work=-2;
        break;
    }
  }
    else if(level_work==-2)
    {
      if(DrainTheWater(30,4))
      {
        level_work=-3;
      }    
    }
    else if(level_work==-3)
    {
      if(Wait(3*60,4))
      {
        level_work=-4;
      }
    }
    else if(level_work==-4)
    {
      switch(TakeTheWater(FlowPulsesReq,flow_time_limit,4,1))
      {
        case 1:
          level_work=3;
          break;
        case 2:
          break;
      }
    }
    else if(level_work==3)
    {
      if(DoTheWashing(10,1))
      {
        level_work=4;
        WashingTime=14*60-GetTimerValue(3);
      }
    }
    else if(level_work==4)
    {
      if(WashingTime==0)
      {
        WashingTime=14*60-GetTimerValue(3);
        SetTimeValue(2,GetTimerValue(1));
        ResetTimer(1);
      }
      if(done_flag_1==0)
        done_flag_1=BringTheWaterTemperatureToTheSpecifiedValue(40,WashingTime,1);
      if(done_flag_2==0)
        done_flag_2=TakeTheDispenser(90,2);
      if(done_flag_1 && done_flag_2)
      {
        level_work=5;
        SetTimeValue(3,14*60);
        done_flag_2=0;
        done_flag_1=0;
        WashingTime=0;
      }
    }
    else if(level_work==5)
    {
      if(DoTheWashing(8*60,1))
      {
        level_work=6;
      }
    }
    else if(level_work==6)
    {
      if(DrainTheWater(30,1))
      {
        level_work=7;
      }
    }
    else if(level_work==7)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,1,0))
    {
      case 1:
        level_work=8;
        break;
      case 2:
        level_work=-7;
        break;
    }
  }
    else if(level_work==-7)
    {
      if(DrainTheWater(30,4))
      {
        level_work=-8;
      }    
    }
    else if(level_work==-8)
    {
      if(Wait(3*60,4))
      {
        level_work=-9;
      }
    }
    else if(level_work==-9)
    {
      switch(TakeTheWater(FlowPulsesReq,flow_time_limit,4,1))
      {
        case 1:
          level_work=8;
          break;
        case 2:
          break;
      }
    }
    else if(level_work==8)
    {
      if(DoTheWashing(10,1))
      {
        level_work=9;
        WashingTime=11*60-(GetTimerValue(3)-22*60);
      }
    }
    else if(level_work==9)
    {
      if(WashingTime==0)
      {
        WashingTime=11*60-(GetTimerValue(3)-22*60);
        ResetTimer(1);
      }
      if(BringTheWaterTemperatureToTheSpecifiedValue(45,WashingTime,1))
      {
        level_work=10;
        SetTimeValue(3,(14+8+11)*60);
      }
    }
    else if(level_work==10)
    {
      if(DoTheWashing(2*60,1))
      {
        level_work=11;
      }
    }
    else if(level_work==11)
    {
      if(DrainTheWater(60,1))
      {
        level_work=12;
      }
    }
    else if(level_work==12)
    {
 
    mode_done=1;     
    if(GenerateBuzzerSound(4,1))
      {
        washing_state=0;
        level_work=-1;
        start_info.status=0;
			  ResetTimer(3); 
      }
  }
}
void main_program_6(void)
{
  washing_state=(level_work!=-1) &&  (level_work<6) ;
  if(level_work==0)
  {
    StartTimer(3);
    if(Wait(10,1))
    {
      level_work=1;
    }
  }
  else if(level_work==1)
  {
    if(DrainTheWater(30,1))
    {
      level_work=2;
    }
  }
  else if(level_work==2)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,1,0))
    {
      case 1:
        level_work=3;
        WashingTime=Pred_Time_6*60-GetTimerValue(3)-60;
        break;
      case 2:
        level_work=-2;
        break;
    }
  }
  else if(level_work==-2)
  {
    if(DrainTheWater(30,4))
    {
      level_work=-3;
    }    
  }
  else if(level_work==-3)
  {
    if(Wait(3*60,4))
    {
      level_work=-4;
    }
  }
  else if(level_work==-4)
  {
    switch(TakeTheWater(FlowPulsesReq,flow_time_limit,4,1))
    {
      case 1:
        level_work=3;
        WashingTime=Pred_Time_6*60-GetTimerValue(3)-60;
        break;
      case 2:
        break;
    }
  }  
  else if(level_work==3)
  {
		if(WashingTime==0)
    {
			WashingTime=Pred_Time_6*60-GetTimerValue(3)-60;
      ResetTimer(1);
    }
    if(DoTheWashing(WashingTime,1))
    {
      level_work=4;
			WashingTime=0;
    }
  }
  else if(level_work==4)
  {
    if(DrainTheWater(60,1))
    {
      level_work=5;
    }
  }
  else if(level_work==5)
  {
		mode_done=1;
    if(GenerateBuzzerSound(4,1))
    {
      washing_state=0;
      level_work=-1;
      start_info.status=0;
			ResetTimer(3);
    }
  }
}
uint8_t TakeTheWater(int FlowMeterPulsesRequired,int TimeOut,int TimerNum,int Error_Checking)
{
  if(get_door_status() && !get_over_flow_status())
  {
    if((flow_meter_count<FlowMeterPulsesRequired) || get_water_level()==0)
    {
      StartTimer(TimerNum);
      OpenInletValve();
      if(GetTimerValue(TimerNum)>=TimeOut)
      {
        if(Error_Checking)
          Error.inlet_water_e2=1;
				CloseInletValve();reset_all();				
        ResetTimer(TimerNum);
        return 2;
      }
      return 0;
    }
    else if((flow_meter_count>=FlowMeterPulsesRequired) && get_water_level()==1)
    {
      ResetTimer(TimerNum);
      CloseInletValve();reset_all();
      return 1;
    }
    return 0;
  }
  else
  {
    StopTimer(TimerNum);
    reset_all();
  }
  return 0;
}
uint8_t TakeTheDispenser(int TimeOut,int TimerNum)
{
  if(get_door_status() && !get_over_flow_status())
  {
    if(GetTimerValue(TimerNum)<TimeOut)
    {
      StartTimer(TimerNum);
      OpenDispenserValve();
      return 0;
    }
    else
    {
      ResetTimer(TimerNum);
      CloseDispenserValve();reset_all();
      return 1;
    }
  }
  else
  {
    StopTimer(TimerNum);
    reset_all();
  }
  return 0;
}
uint8_t DoTheWashing(int TimeOut,int TimerNum)
{
  if(get_door_status() && !get_over_flow_status())
  {
    if(get_water_level() || 1)
    {
      StartTimer(TimerNum);
      TurnOnTheWashingPump();
      CloseInletValve();
      if(GetTimerValue(TimerNum)>=TimeOut)
      {
        ResetTimer(TimerNum);
        TurnOffTheWashingPump();reset_all();
        return 1;
      }
    }
    else
    {
      StopTimer(TimerNum);
      OpenInletValve();
    }
  }
  else
    reset_all();
  return 0;
}
uint8_t BringTheWaterTemperatureToTheSpecifiedValue(int TempSetPoint,int TimeOut,int TimerNum)
{
  if(get_door_status() && !get_over_flow_status())
  {
    if(get_water_level() || 1)
    {
      StartTimer(TimerNum);
      CloseInletValve();
      TurnOnTheWashingPump();
      TurnOnHeater();
      if(GetWaterTemp()>=TempSetPoint)
      {
        ResetTimer(TimerNum);
        TurnOffTheWashingPump();
        TurnOffHeater();reset_all();
        return 1;
      }
      else if(GetTimerValue(TimerNum)>=TimeOut)
      {
        Error.heater_alarm_e7=1;
      }
    }
    else
    {
      StopTimer(TimerNum);
      TurnOffHeater();
      OpenInletValve();
    }
  }
  else
  {
    reset_all();
  }
  return 0;
}
uint8_t DrainTheWater(int TimeOut,int TimerNum)
{
  if(get_door_status() && !get_over_flow_status())
  {
    StartTimer(TimerNum);
    if(GetTimerValue(TimerNum)<TimeOut)
    {
      TurnOnTheDrainPump();
    }
    else
    {
      if(!get_water_level())
      {
        TurnOffTheDrainPump();reset_all();
        flow_meter_count=0;
				ResetTimer(TimerNum);
        return 1;
      }
      else
      {
        Error.drain_alarm_e3=1;
        reset_all();
      }
    }
  }
  else
    reset_all();
  return 0;
}
uint8_t AddSaltWater(int TimeOut,int TimerNum)
{
  if(get_door_status() && !get_over_flow_status())
  {
    if(GetTimerValue(TimerNum)<TimeOut)
    {
      StartTimer(TimerNum);
      OpenSaltValve();
    }
    else
    {
      ResetTimer(TimerNum);
      CloseSaltValve();reset_all();
			return 1;
    }
  }
  else
    reset_all();
  return 0;
}
uint8_t GenerateBuzzerSound(int NumOfSounds,int TimerNum)
{
  uint8_t timer_count=GetTimerCount(TimerNum);
  if(timer_count<NumOfSounds*2)
  {
    StartTimer(TimerNum);
    if(timer_count%2==0)
      TurnOffBuzzer();
    else
      TurnOnBuzzer();
  }
  else
  {
    ResetTimer(TimerNum);
    TurnOffBuzzer();
    reset_all();
    return 1;
  }
  return 0;
}
uint8_t Wait(int TimeOut,int TimerNum)
{
  if(get_door_status() && !get_over_flow_status())
  {
    if(GetTimerValue(TimerNum)<TimeOut)
    {
      StartTimer(TimerNum);
    }
    else if(GetTimerValue(TimerNum)>=TimeOut)
    {
      ResetTimer(TimerNum);reset_all();
			return 1;
    }
  }
  else
    reset_all();
  return 0;
}
void StartTimer(int TimerNum)
{
  if(counters[TimerNum-1].counter_Enable==0)
    counters[TimerNum-1].counter_Enable=1;
}
void StopTimer(int TimerNum)
{
  if(counters[TimerNum-1].counter_Enable==1)
    counters[TimerNum-1].counter_Enable=0;
}
void ResetTimer(int TimerNum)
{
  counters[TimerNum-1].counter_Enable=-1;
  counters[TimerNum-1].counter_value=0;
}
int GetTimerValue(int TimerNum)
{
  return  counters[TimerNum-1].counter_value/2;
}
int GetTimerCount(int TimerNum)
{
  return  counters[TimerNum-1].counter_value;
}
float GetWaterTemp(void)
{
  return temp;
}
void OpenInletValve(void)
{
  HAL_GPIO_WritePin(iv_GPIO_Port,iv_Pin,GPIO_PIN_SET);
}
void CloseInletValve(void)
{
  HAL_GPIO_WritePin(iv_GPIO_Port,iv_Pin,GPIO_PIN_RESET);
}
void OpenDispenserValve(void)
{
  HAL_GPIO_WritePin(dv_GPIO_Port,dv_Pin,GPIO_PIN_SET);
}
void CloseDispenserValve(void)
{
  HAL_GPIO_WritePin(dv_GPIO_Port,dv_Pin,GPIO_PIN_RESET);
}
void OpenSaltValve(void)
{
  HAL_GPIO_WritePin(sv_GPIO_Port,sv_Pin,GPIO_PIN_SET);
}
void CloseSaltValve(void)
{
  HAL_GPIO_WritePin(sv_GPIO_Port,sv_Pin,GPIO_PIN_RESET);
}
void TurnOnTheWashingPump(void)
{
  HAL_GPIO_WritePin(wp_GPIO_Port,wp_Pin,GPIO_PIN_SET);
}
void TurnOffTheWashingPump(void)
{
  HAL_GPIO_WritePin(wp_GPIO_Port,wp_Pin,GPIO_PIN_RESET);
}
void TurnOnHeater(void)
{
  HAL_GPIO_WritePin(heat_relay_GPIO_Port,heat_relay_Pin,GPIO_PIN_SET);
}
void TurnOffHeater(void)
{
  HAL_GPIO_WritePin(heat_relay_GPIO_Port,heat_relay_Pin,GPIO_PIN_RESET);
}
void TurnOnTheDrainPump(void)
{
  HAL_GPIO_WritePin(dp_GPIO_Port,dp_Pin,GPIO_PIN_SET);
}
void TurnOffTheDrainPump(void)
{
  HAL_GPIO_WritePin(dp_GPIO_Port,dp_Pin,GPIO_PIN_RESET);
}
void TurnOnBuzzer(void)
{
  HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin,GPIO_PIN_SET);
}
void TurnOffBuzzer(void)
{
  HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin,GPIO_PIN_RESET);
}
void beep(void)
{
	TurnOnBuzzer();
	HAL_Delay(250);
	TurnOffBuzzer();
	HAL_Delay(250);
}
void SetTimeValue(int TimerNum,int SetTimeValue)
{
  counters[TimerNum-1].counter_value=SetTimeValue*2;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
