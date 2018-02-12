/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
//#include "string.h"
#include "sms.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

osThreadId MainHandle;
osThreadId SensorsHandle;
osThreadId SMSHandle;
osThreadId AdminHandle;
osThreadId pickHandle;
osMessageQId toAlarmHandle;
osMessageQId toMainHandle;
osMessageQId toSensorsHandle;
osMessageQId toSMSHandle;
osSemaphoreId toTickHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
	uint8_t state =0;
	uint32_t sensors = 1;																							// переменная, полученая из модуля sensors. содержит биты состояний и значение температуры двигателя
	uint32_t sensors_mask = 0xFFFFFFFF;															  // mask for off some sensors
	uint16_t to_sensor_module = 0x8000;																			// переменная для передачи команд в задачу sensors
	uint8_t counter_of_alarm=0;																				// счетчик количества срабатываний
	uint8_t counter_of_starts=0;																			// счетчик количества попыток старта двигателя
	uint32_t timer_Guard_But_Door;																		// время ожидания закрытия двери
	uint32_t timer_Open_But_Door;																			// время ожидания открытия двери
	uint32_t timer_of_alarm;																					// время сколько орать
	uint32_t timer_time_of_start_engine;															// время работы стартера
	uint32_t timer_of_start_engine_rest;															// время отдыха после стартера
	uint32_t timer_of_engine_work;
	uint8_t flag_autostart_allowed=0;																	// разрешен ли автозапуск
	uint32_t delay=0;																									// переменная для задержки. после отправки команд в задачу sensors ее используем
//sms variable
	uint16_t recieve_word;																						//переменная для помещения в очередь смс
	uint8_t RxByte; 																									// буфер для приема 
	char admin_number[]="9091286400";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void Task_Main(void const * argument);
void Task_Sensors(void const * argument);
void Task_SMS(void const * argument);
void Task_Admin(void const * argument);
void Task_pick(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of toTick */
  osSemaphoreDef(toTick);
  toTickHandle = osSemaphoreCreate(osSemaphore(toTick), 4);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of Main */
  osThreadDef(Main, Task_Main, osPriorityNormal, 0, 128);
  MainHandle = osThreadCreate(osThread(Main), NULL);

  /* definition and creation of Sensors */
  osThreadDef(Sensors, Task_Sensors, osPriorityAboveNormal, 0, 128);
  SensorsHandle = osThreadCreate(osThread(Sensors), NULL);

  /* definition and creation of SMS */
  osThreadDef(SMS, Task_SMS, osPriorityHigh, 0, 128);
  SMSHandle = osThreadCreate(osThread(SMS), NULL);

  /* definition and creation of Admin */
  osThreadDef(Admin, Task_Admin, osPriorityBelowNormal, 0, 128);
  AdminHandle = osThreadCreate(osThread(Admin), NULL);

  /* definition and creation of pick */
  osThreadDef(pick, Task_pick, osPriorityIdle, 0, 128);
  pickHandle = osThreadCreate(osThread(pick), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of toAlarm */
  osMessageQDef(toAlarm, 4, uint8_t);
  toAlarmHandle = osMessageCreate(osMessageQ(toAlarm), NULL);

  /* definition and creation of toMain */
  osMessageQDef(toMain, 1, uint32_t);
  toMainHandle = osMessageCreate(osMessageQ(toMain), NULL);

  /* definition and creation of toSensors */
  osMessageQDef(toSensors, 1, uint16_t);
  toSensorsHandle = osMessageCreate(osMessageQ(toSensors), NULL);

  /* definition and creation of toSMS */
  osMessageQDef(toSMS, 4, uint16_t);
  toSMSHandle = osMessageCreate(osMessageQ(toSMS), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0);
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, door_Pin|glass_Pin|eng_Pin|starter_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(eng_add_GPIO_Port, eng_add_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : door_Pin glass_Pin eng_Pin starter_Pin */
  GPIO_InitStruct.Pin = door_Pin|glass_Pin|eng_Pin|starter_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : eng_add_Pin */
  GPIO_InitStruct.Pin = eng_add_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(eng_add_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : begin_autostart_Pin shock_hi_Pin */
  GPIO_InitStruct.Pin = begin_autostart_Pin|shock_hi_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : engine_work_Pin handbrake_Pin headlamp_Pin engnition_Pin 
                           door_open_Pin button_close_Pin */
  GPIO_InitStruct.Pin = engine_work_Pin|handbrake_Pin|headlamp_Pin|engnition_Pin 
                          |door_open_Pin|button_close_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	recieve_word=RxByte+0x100;
	portBASE_TYPE * pxHigherPriorityTaskWoken;
	xQueueSendFromISR(toSMSHandle, &recieve_word, pxHigherPriorityTaskWoken);
	HAL_UART_Receive_IT(&huart2,&RxByte,1);
};
/* USER CODE END 4 */

/* Task_Main function */
void Task_Main(void const * argument)
{

  /* USER CODE BEGIN 5 */
	//temp
	uint32_t GLOBAL_timer_Guard_But_Door=60000; //1мин
	uint32_t GLOBAL_timer_Open_But_Door=60000; //1мин
	uint32_t GLOBAL_timer_of_alarm = 120000;
	uint32_t GLOBAL_timer_of_engine_work=300000;
	uint8_t GLOBAL_flag_autostart_allowed=1;
	uint32_t GLOBAL_time_of_start_engine=10000;//10sec 
	uint32_t  GLOBAL_time_of_start_engine_rest=30000;//30sec
	uint8_t GLOBAL_max_of_try_start=3;
	uint8_t GLOBAL_counter_of_alarm=3;
	uint8_t GLOBAL_in_temperature=100;
  
	//temp
	
	
	
	
 // uint8_t state = 0; 																								// переменная состояния
/*	uint32_t sensors = 1;																							// переменная, полученая из модуля sensors. содержит биты состояний и значение температуры двигателя
	uint32_t sensors_mask = 0xFFFFFFFF;															  // mask for off some sensors
	uint16_t to_sensor_module = 0;																			// переменная для передачи команд в задачу sensors
	uint8_t counter_of_alarm=0;																				// счетчик количества срабатываний
	uint8_t counter_of_starts=0;																			// счетчик количества попыток старта двигателя
	uint32_t timer_Guard_But_Door;																		// время ожидания закрытия двери
	uint32_t timer_Open_But_Door;																			// время ожидания открытия двери
	uint32_t timer_of_alarm;																					// время сколько орать
	uint32_t timer_time_of_start_engine;															// время работы стартера
	uint32_t timer_of_start_engine_rest;															// время отдыха после стартера
	uint32_t timer_of_engine_work;
	uint8_t flag_autostart_allowed=0;																	// разрешен ли автозапуск
	uint32_t delay=0;																									// переменная для задержки. после отправки команд в задачу sensors ее используем
*/	
	#define pick(times) for (uint8_t i=0; i<times; i++) xSemaphoreGive( toTickHandle)
	
	#define IN_button_close					 	(sensors & 0x80000000)					// старший бит 
	#define IN_door_open 							(sensors & 0x40000000)
	#define IN_eng										(sensors & 0x20000000)
	#define IN_headlights							(sensors & 0x10000000)
	#define IN_flag_autostart_allowed	(sensors & 0x08000000)
	#define IN_handbrake							(sensors & 0x04000000)
	#define IN_engine_work 						(sensors & 0x02000000)
	#define IN_shock_hi								(sensors & 0x01000000)
	#define IN_shock_low							(sensors & 0x00800000)
	#define IN_begin_autostart				(sensors & 0x00400000)
	#define IN_backdoor_open					(sensors & 0x00200000)
	#define IN_temperature						(sensors & 0x000000FF)					// 1 байт для значения температуры
	
	#define OUT_door_to_close					(to_sensor_module|=0x0001)				//закрыть двери
	#define OUT_door_to_open					(to_sensor_module|=0x0002)				//
	#define OUT_glass_to_close				(to_sensor_module|=0x0004)				//закрыть стекло
	#define OUT_button_close					(to_sensor_module|=0x0008)				//перевести кнопку в значение "закрыто"
	#define OUT_button_open						(to_sensor_module|=0x0010)			//перевести кнопку в значение "открыто"
	#define OUT_eng_main_on						(to_sensor_module|=0x0020)				//
	#define OUT_eng_main_off					(to_sensor_module|=0x0040)				//
	#define OUT_eng_additional_on			(to_sensor_module|=0x0080)				//
	#define OUT_eng_additional_off		(to_sensor_module|=0x0100)				//
	#define OUT_starter_on						(to_sensor_module|=0x0200)				//
	#define OUT_starter_off						(to_sensor_module|=0x0400)				//
	
	#define state_open 									0
	#define state_guard 								1
	#define state_alarm 								2
	#define state_guardButDoor 					3
	#define state_openButDoor 					4
	#define state_autostart_ready 			5
	#define state_autostart_startengine 6
	#define state_autostart_work 				7
							
	/* Infinite loop */
  for(;;)
  {
	/*?????*/  sensors &= sensors_mask; 																					// выключили некоторые сенсоры по маске	
		
	switch (state)
    {	
			case state_open:// состояние open 
				sensors_mask=0xFFFFFFFF;																		//если была сработка и поставлена маска не смотреть дверь, от ее здесь надо обнулить
				if ((IN_button_close>0)&(IN_door_open==0)&(IN_eng==0))			//  нажали на кнопку закрыть. если кнопк==1, дверь закрыта, зажигания нет, 
					{	
						if (IN_headlights==0) pick(1); else pick(3);			 		// 	если фары (пикаем разик) иначе (пикаем 3, отправляем смс?)
						OUT_door_to_close;
						OUT_glass_to_close;					//	двери на закрытие,стекло закрыть,
						delay=300; 
					state=state_guard;
					};												//	ждем 0,3 сек и прыгаем в состояние guard							
				if ((IN_button_close!=0)&(IN_door_open!=0)&(IN_eng==0))	//  нажали кнопку закрыть. кнопк==1, дверь ОТКРЫТА, зажигания нет.
					{	if (IN_headlights!=0) pick(3);												//	если фары==1 то (пикаем 3,смс,) 
						OUT_glass_to_close; 
						delay=300;
						timer_Guard_But_Door = GLOBAL_timer_Guard_But_Door;//2min			//	закрываем окно ждем 0.3 и в состояние Guard?BUtDoor, таймер guardButDoor=2мин
						delay=300;
						state=state_guardButDoor; };
				if ((IN_button_close!=0)&(IN_eng!=0))									//  кнопк==1, зажигание есть, то ошибка -
					{	pick(4);
					OUT_button_open;	};												//	пикаем 4, состояние не меняем, состояние кнопки изменить на 0 и в sensors module его
				if ((GLOBAL_flag_autostart_allowed!=0)&(IN_handbrake!=0)&(IN_engine_work!=0))//  если вобще он разрешен, то смотрим  ручник==1, заведен==1, то
					{ OUT_eng_main_on;
					OUT_eng_additional_on;
					state=state_autostart_ready; };// включаем зажигание_оут=1 и зажигание_оут_доп=1 и в состояние AutoStartReady
				counter_of_alarm=0;					//счетчик срабатывания = 0
				break;
					
    	case state_guard:// состояние guard
				if (IN_button_close==0) //  кнопка==0,
					{	pick(2);
						OUT_door_to_open;
						delay=300;
						timer_Open_But_Door = GLOBAL_timer_Open_But_Door;//1min;//пикаем 2, двери на открытие, ждем 0,3 сек и , таймер openButDoor=Хмин
						state = state_openButDoor;}; //прыгаем в состояние open_butDoor
				if ((IN_door_open!=0)|(IN_shock_hi!=0)|(IN_eng!=0))
					{	counter_of_alarm++;
						timer_of_alarm=GLOBAL_timer_of_alarm ;  
						state=state_alarm ;
					};//  дверь ОТКРЫТА или зажигание==1 или ударHI==1, то счетчик срабатывания ++, timealarm=время для ора и в состояние Alarm
				if (IN_shock_low!=0) pick(4);//	 ударLOW==1 то пикпикх4
				if ((IN_begin_autostart!=0)&(flag_autostart_allowed!=0)) //  begin_autostart==1,смотрим разрешение на автозапуск(ставится из autostartready) ==1
					{	OUT_eng_main_on;
						timer_time_of_start_engine=GLOBAL_time_of_start_engine/*(10sek)*/; 
						timer_of_start_engine_rest=GLOBAL_time_of_start_engine_rest/*(30sek)*/;	//, то зажигание_оут=1, таймер времени завода =10сек, таймер отдыха=30 сек
						counter_of_starts=GLOBAL_max_of_try_start/*3raza*/;
						delay=1000;
						state=state_autostart_startengine;//кол-во попыток=3, ждем 1сек, переход в состояние autostart_startengine
					};
				break;
					
			case state_alarm: // alarm. отсылаем каждый раз очередь в задачу alarmtask. пока счетчик не обнулится
				//пусть пищит по одному.пик-пик-пик. при сработке просто пики непрерывные. скорее всего ее вобще отключат чтоб не орал
				if ((timer_of_alarm>0)&(counter_of_alarm<GLOBAL_counter_of_alarm/*3раз*/))// timeaalarm>0 и счетчик сработки менее порога, то
					{	pick(1); timer_of_alarm--;	};	//кинуть в очередь в alarmtask еще одну еденичку для ора, timealarm--
				if (timer_of_alarm==0) // если timealarm ==0; то
					{	if (IN_door_open!=0) sensors_mask=0xBFDFFFFF;//если сюда пришли от двери( дверь==открыта) то в маску пишем не смотреть дверь
						counter_of_alarm++;
						state=state_guard;	};	// потом увеличиваем счетчик срабатывания++(обычно до трех) и переходим в состояние guard
				if (IN_button_close==0) //кнопка==0,то
					{  timer_Open_But_Door=GLOBAL_timer_Open_But_Door ;
					state=state_openButDoor ; }; //таймер openButDoor=30сек, состояние open?butDoors.(при переходе в другое состояние двери не открываются!!!)
				break;
					
					
			case state_guardButDoor:// состояние guard,butDoor 
				if (timer_Guard_But_Door == 0)
					{/*SMS(door)*/; pick(4);
						timer_Guard_But_Door = INT32_MAX; // смс о незакрытой двери, пик 4, таймер=макс для неопределения его ниже.  таймер оттикал положенные минуты, надо предупредить пользователя о незакрытой двери
						sensors_mask=0xBFDFFFFF;
						state=state_guard;};// выключаем маской двери и в режим охраны
				if (timer_Guard_But_Door < INT32_MAX) timer_Guard_But_Door--;    //таймер тикает или нет, одного предупреждения достаточно.
				if ((IN_door_open==0) & (IN_eng==0))//  дверь закрыта,зажигания нет,
					{	pick(1); OUT_door_to_close;delay=500;  state=state_guard; }; //пикаем разик, двери на закрытие, ждем 0,3 сек и прыгаем в состояние guard 
					// надо либо сразу в очередь кидать или задержка бесполезна!!!!!!!
					// дверь наконец- то закрыли. надо смотреть на зажигание, чтоб никто не сидел внутри, или ключ в заведенной машине не оставил 
				if (IN_eng !=0 ) pick(1);//  зажигание==1,то пик1. //включили зажигание на подохранной машиной. надо попищать по одному пику пока не выключат зажигание
				break;
			
			case state_openButDoor: // состояние openButDoor. нажали на открытие, но двери не открыли
				if (IN_door_open!=0) state=state_open; //дверь открыта, переходим в состояние open
				timer_Open_But_Door--; //тикаем таймером					
				if (timer_Open_But_Door==0) {OUT_button_close; state=state_open; };//таймер дотикал до нуля, пикаем 1, отправить в модуль sensors значение кнопки =1 (виртуальная кнопка закрыто) и переходим в режим открыто- оттуда автоматом упадет в guard 
				break;
			
			case state_autostart_ready:// состояние autostartready
				if (IN_handbrake==0) {OUT_eng_main_off; OUT_eng_additional_off; state=state_open;}; //если ручник отпустили, то зажигание_оут=0 и зажигание_оут_доп=0, и переходим обратно в open
				if (IN_button_close!=0) { OUT_eng_main_off; OUT_eng_additional_off;// если нажали кнопку закрыть то зажигание_оут=0 и зажигание_оут_доп=0, 
																	if (IN_door_open) flag_autostart_allowed=0; else flag_autostart_allowed=1; state=state_open; delay=10000;};//флаг разрешения автозапуска зависит от состояния двери(дверь открыта-нет флага) и прыгаем в open? оттудаавтоматом упадем в guard
				break;
			
			case 6:// состояние autostart_startengine
				if (timer_time_of_start_engine>0) {OUT_starter_on; timer_time_of_start_engine--;};			//если таймер_времени_завода >0 то стартер=1, таймер--
				if (timer_time_of_start_engine==0) {OUT_starter_off; timer_of_start_engine_rest--; };			//если таймер_времени_завода ==0 то стартер=0, таймер отдыха--
				if ((timer_time_of_start_engine==0)&(timer_of_start_engine_rest==0))
					{	counter_of_starts--;
						timer_time_of_start_engine=GLOBAL_time_of_start_engine/*(10sek)*/; timer_of_start_engine_rest=GLOBAL_time_of_start_engine_rest/*(30sek)*/;	
					};				//если оба таймер_времени_завода==0 и таймер отдыха==0 то количество_попыток--,выставляем таймеры как из guarda/ таймер времени завода =10сек, таймер отдыха=30 сек,
				if (counter_of_starts==0) 
					{	/*SMS("can't start'");*/ OUT_eng_main_off; flag_autostart_allowed=0; state=1;
					};//если кол-во_попыток==0, то отправить_смс"не могу завестись" , зажигание_оут=0, флаг разрешения завода=0(иначе по пределу температуры высадит весь акк) и возвращаемся в guard
				if (IN_button_close==0) 
					{	OUT_starter_off; OUT_eng_main_off; OUT_eng_additional_off; pick(2) ; OUT_door_to_open; timer_Open_But_Door = GLOBAL_timer_Open_But_Door; state=4;
					}; // если кнопка=о ,то стартер=0, зажигание_оут=0 и зажигание_оут_доп=0,пикаем 2, двери на открытие, ждем 0,3 сек и прыгаем в состояние open_butDoor, таймер openButDoor=Хмин
				if (IN_engine_work !=0) 
					{	OUT_starter_off; OUT_eng_additional_on; timer_of_engine_work = GLOBAL_timer_of_engine_work; state = 7;
					};// если завод==1(по тахометру или напр.-обрабатывается в sensors) то стартер=0 , то ждем 5 сек, зажигание_оут_доп =1 , таймер_прогрева=Ч минут и переходим в состояние autostart_work 
				break;
			
			case 7:// autostart_work. завелись и просто греемся
				if (IN_button_close==0)
					{	OUT_eng_main_off; OUT_eng_additional_off; pick(2); OUT_door_to_open; timer_Open_But_Door = GLOBAL_timer_Open_But_Door; state=4;
					};// если кнопка=о или дверь=0 ,то  зажигание_оут=0 и зажигание_оут_доп=0,пикаем 2, двери на открытие, ждем 0,3 сек и прыгаем в состояние open_butDoor, таймер openButDoor=Хмин
				if (timer_of_engine_work>0) timer_of_engine_work-- ;// если таймер_прогрева>0 то таймер_прогрева--, 			
				if ((timer_of_engine_work==0)|(IN_temperature>GLOBAL_in_temperature)) 
					{	OUT_eng_main_off; OUT_eng_additional_off; pick(1); state = 1;
					};// если таймер_прогрева==0 или температура движка дошла до целевой, то зажигание_оут=0 и зажигание_оут_доп=0,пикаем 1, и в состояние guard
				break;	
      };// end swich
		
	if (IN_door_open!=0) flag_autostart_allowed=0; //если открыли дверь из любого режима, то флаг разрешения автозапуска=0
	//to_sensor_module|=(state<<12);		// отправляем значение state для отладки
	if (delay>0) osDelay(delay); delay=0;
	
	xQueueSend( toSensorsHandle, &to_sensor_module,portMAX_DELAY);// отправляем команды в задачу  sensors
	to_sensor_module=0x8000; // после отсылки обнуляем команды . Тут только флаг, что команда пришла из main	
	xQueueReceive( toMainHandle, &sensors,portMAX_DELAY);  				  //see queue for update sensors value			
	
  }
 
  /* USER CODE END 5 */ 
}

/* Task_Sensors function */
void Task_Sensors(void const * argument)
{
  /* USER CODE BEGIN Task_Sensors */
	uint32_t sensors_to_main = 0x00000000; // отправка значений ног в задачу main 
	uint16_t commands = 0; // команды, полученные из main
	uint8_t phisical_button, prev_phisical_button, logical_button;
	portBASE_TYPE recieve_result;
  
  /* Infinite loop */
  for(;;)
  {	
		recieve_result = xQueueReceive ( toSensorsHandle, &commands,0);
		
		sensors_to_main = 0x00000000;
		phisical_button=HAL_GPIO_ReadPin( button_close_GPIO_Port, button_close_Pin);  // считываем значение на ноге во временную переменную
		if (prev_phisical_button^phisical_button) logical_button=phisical_button;// если значение изменилось, то пишем ее значение в логическую
		prev_phisical_button=phisical_button;// сохраняем для сравнения
		if ((commands)&(0x0008)) logical_button=1;
		if ((commands)&(0x0010)) logical_button=0;
		//сюда вставить код для обработки смс
		if (logical_button) sensors_to_main+=0x80000000;// готов к отправке в main
		
		if (HAL_GPIO_ReadPin( door_open_GPIO_Port, door_open_Pin)) 							sensors_to_main+=0x40000000;// else sensors_to_main&=0xBFFFFFFF;	//устанавливаем или убираем бит кнопки
		if (HAL_GPIO_ReadPin( engnition_GPIO_Port, engnition_Pin)) 							sensors_to_main+=0x20000000;// else sensors_to_main&=0xDFFFFFFF;	//устанавливаем или убираем бит кнопки
		if (HAL_GPIO_ReadPin( headlamp_GPIO_Port, headlamp_Pin)) 								sensors_to_main+=0x10000000;// else sensors_to_main&=0xEFFFFFFF;	//устанавливаем или убираем бит кнопки
		//if (HAL_GPIO_ReadPin( headlamp_GPIO_Port, headlamp_Pin)) 								sensors_to_main|=0x08000000; else sensors_to_main&=0xEFFFFFFF;	//flag autostart allowed - from sms
		if (HAL_GPIO_ReadPin( handbrake_GPIO_Port, handbrake_Pin)) 							sensors_to_main+=0x04000000;// else sensors_to_main&=0xFBFFFFFF;	//устанавливаем или убираем бит кнопки
		if (HAL_GPIO_ReadPin( engine_work_GPIO_Port, engine_work_Pin)) 					sensors_to_main+=0x02000000;// else sensors_to_main&=0xFDFFFFFF;	//устанавливаем или убираем бит кнопки
		if (HAL_GPIO_ReadPin( shock_hi_GPIO_Port, shock_hi_Pin)) 								sensors_to_main+=0x01000000;// else sensors_to_main&=0xFEFFFFFF;	//устанавливаем или убираем бит кнопки
		//if (HAL_GPIO_ReadPin( shock_hi_GPIO_Port, shock_hi_Pin)) 								sensors_to_main|=0x00800000;// else sensors_to_main&=0xFEFFFFFF;	//shock low
		if (HAL_GPIO_ReadPin( begin_autostart_GPIO_Port, begin_autostart_Pin))  sensors_to_main+=0x00400000;// else sensors_to_main&=0xFFBFFFFF;	//устанавливаем или убираем бит кнопки
		//if (HAL_GPIO_ReadPin( begin_autostart_GPIO_Port, begin_autostart_Pin))  sensors_to_main|=0x00400000;// else sensors_to_main&=0xFFBFFFFF;	//backdoor
		// temperature
		//не все кнопки тут
		
		
		
		if (commands&0x0001) HAL_GPIO_WritePin(door_GPIO_Port,door_Pin,1) ;
	
	
		if (commands&0x0002) HAL_GPIO_WritePin(door_GPIO_Port,door_Pin,0) ;
		
		if (commands&0x0004) HAL_GPIO_TogglePin(glass_GPIO_Port,glass_Pin) ; //OUT_glass_to_close
		
	/*	if (commands&0x0008) HAL_GPIO_WritePin(door_GPIO_Port,door_open_Pin,1) ;//OUT_button_close
		
		if (commands&0x0010) HAL_GPIO_WritePin(door_GPIO_Port,door_open_Pin,1) ;//OUT_button_open
	*/
		if (commands&0x0020) HAL_GPIO_WritePin(eng_GPIO_Port,eng_Pin,1) ;//OUT_eng_main_on
		
		if (commands&0x0040) HAL_GPIO_WritePin(eng_GPIO_Port,eng_Pin,0) ;//OUT_eng_main_off
	
		if (commands&0x0080) HAL_GPIO_WritePin(eng_add_GPIO_Port,eng_add_Pin,1) ;//OUT_eng_additional_on
	
		if (commands&0x0100) HAL_GPIO_WritePin(eng_add_GPIO_Port,eng_add_Pin,0) ;//OUT_eng_additional_off
		
		if (commands&0x0200) HAL_GPIO_WritePin(starter_GPIO_Port,starter_Pin,1) ;//OUT_starter_on
		
		if (commands&0x0400) HAL_GPIO_WritePin(starter_GPIO_Port,starter_Pin,0) ;//OUT_starter_off
		
		
		

			
		
		if (commands&0x8000) xQueueSend( toMainHandle, &sensors_to_main, portMAX_DELAY );
		commands=0;
		osDelay(1);
		
  }
  /* USER CODE END Task_Sensors */
}

/* Task_SMS function */
void Task_SMS(void const * argument)
{
  /* USER CODE BEGIN Task_SMS */
	uint16_t delay_read_sms=2000;																			// проверяем смс каждые ... мсек
	osDelay(2000);
	HAL_UART_Receive_IT(&huart2,&RxByte,1);
  Neoway_init();
  /* Infinite loop */
  for(;;)
  {
		if (delay_read_sms--==0) {	Read_SMS();	delay_read_sms=2000;	};
		MonitoringRing();
		osDelay(1);
  }
  /* USER CODE END Task_SMS */
}

/* Task_Admin function */
void Task_Admin(void const * argument)
{
  /* USER CODE BEGIN Task_Admin */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task_Admin */
}

/* Task_pick function */
void Task_pick(void const * argument)
{
  /* USER CODE BEGIN Task_pick */
  /* Infinite loop */
  for(;;)
  {	
		xSemaphoreTake( toTickHandle, portMAX_DELAY);
		HAL_GPIO_WritePin( LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		osDelay(500);
		HAL_GPIO_WritePin( LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		osDelay(500);	
  }
  /* USER CODE END Task_pick */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

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
