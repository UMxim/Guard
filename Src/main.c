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

//sms variable
	uint16_t recieveWord;																						//���������� ��� ��������� � ������� ���
	uint8_t RxByte; 																									// ����� ��� ������ 
	char admin_number[]="9091286400";
// define command to sensor module	
	#define dOutDoorToClose					1				
	#define dOutDoorToOpen					2				
	#define dOutGlassToClose				3
	#define dOutButtonToClose				4			
	#define dOutButtonToOpen				5		
	#define dOutEngMainToOn					6			
	#define dOutEngMainToOff				7		
	#define dOutEngAdditionalToOn		8			
	#define dOutEngAdditionalToOff	9			
	#define dOutStarterToOn					10			
	#define dOutStarterToOff				11	
	#define dOutUpdateSensors				12
// define number to sms
	#define dSmsDoorOpen						0x0200
	#define dSmsShockHi							0x0300
	#define dSmsShockLow						0x0400
	#define dSmsEngOn								0x0500
	#define dSmsDoorNotClose				0x0600
	#define dSmsCantStartEngine			0x0700
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
	recieveWord=RxByte+0x100;
	portBASE_TYPE * pxHigherPriorityTaskWoken;
	xQueueSendFromISR(toSMSHandle, &recieveWord, pxHigherPriorityTaskWoken);
	HAL_UART_Receive_IT(&huart2,&RxByte,1);
};
/* USER CODE END 4 */

/* Task_Main function */
void Task_Main(void const * argument)
{

  /* USER CODE BEGIN 5 */
	uint8_t state =0;
	uint32_t sensors = 1;																							// ����������, ��������� �� ������ sensors. �������� ���� ��������� � �������� ����������� ���������
	uint32_t sensorsMask = 0xFFFFFFFF;															  // mask for off some sensors
	
	uint8_t counterOfAlarm=0;																				// ������� ���������� ������������
	uint8_t counterOfStarts=0;																			// ������� ���������� ������� ������ ���������
	
	uint32_t timerGuardButDoor;																		// ����� �������� �������� �����
	uint32_t timerOpenButDoor;																			// ����� �������� �������� �����
	uint32_t timerOfAlarm;																					// ����� ������� �����
	uint32_t timerOfStartEngine;															// ����� ������ ��������
	uint32_t timerOfStartEngineRest;															// ����� ������ ����� ��������
	uint32_t timerOfEngineWork;
	
	uint8_t flagAutostartAllowed=0;																	// �������� �� ����������
	
	//temp
	uint32_t	globalTimerGuardButDoor=60000; //1���
	uint32_t 	globalTimerOpenButDoor=60000; //1���
	uint32_t  globalTimerOfAlarm = 120000;
	uint32_t  globalTimerOfEngineWork=300000;
	uint8_t	  globalFlagAutostartAllowed=1;
	uint32_t  globalTimeOfStartEngine=10000;//10sec 
	uint32_t	globalTimeOfStartEngineRest=30000;//30sec
	uint8_t	  globalMaxOfTryStart=3;
	uint8_t 	globalCounterOfAlarm=3;
	uint8_t   globalInTemperature=100;
		
	#define dPick(times)  for (uint8_t i=0; i<times; i++) xSemaphoreGive( toTickHandle) ;
	#define dSendCommand(command) { uint8_t temp=command;  xQueueSend( toSensorsHandle, &temp, portMAX_DELAY) ;};// ���������� ������� � ������  sensors)
	#define dSendSms(command)		{ uint16_t temp=command;  xQueueSend(toSMSHandle,&temp, 1);};
	
// ���������� ����������, ���������� �� sensors �� ���� ���������
	#define dIsButtonClose					 	(sensors & 0x80000000)					// ������� ��� 
	#define dIsDoorOpen 							(sensors & 0x40000000)
	#define dIsEngOn									(sensors & 0x20000000)
	#define dIsHeadlightsOn						(sensors & 0x10000000)
	#define dIsFlagAutostartAllowed		(sensors & 0x08000000)
	#define dIsHandbrakeOn						(sensors & 0x04000000)
	#define dIsEngineWork 						(sensors & 0x02000000)
	#define dIsShockHi								(sensors & 0x01000000)
	#define dIsShockLow								(sensors & 0x00800000)
	#define dIsBeginAutostart					(sensors & 0x00400000)
	#define dIsBackdoorOpen						(sensors & 0x00200000)
	#define dInTemperature						(sensors & 0x000000FF)					// 1 ���� ��� �������� �����������
	
	#define dStateOpen 									0
	#define dStateGuard 								1
	#define dStateAlarm 								2
	#define dStateGuardButDoorOpen			3
	#define dStateOpenButDoorClose	 		4
	#define dStateAutostartReady	 			5
	#define dStateAutostartStartEngine	6
	#define dStateAutostartWork 				7
	
	
							
	/* Infinite loop */
  for(;;)
  {
	
	dSendCommand(dOutUpdateSensors);
	xQueueReceive( toMainHandle, &sensors,portMAX_DELAY);  				  //see queue for update sensors value				
	sensors &= sensorsMask; 																					// ��������� ��������� ������� �� �����	
	
	switch (state)
    {	
			case dStateOpen	:																																											// ��������� open 
				sensorsMask=0xFFFFFFFF;																			//���� ���� �������� � ���������� ����� �� �������� �����, �� �� ����� ���� ��������
				if ( (dIsButtonClose!=0) & (dIsDoorOpen==0) & (dIsEngOn==0) )																				//  ������ �� ������ �������. ���� �����==1, ����� �������, ��������� ���, 
					{	
						if (dIsHeadlightsOn!=0) dPick(3);			 																											// 	���� ����� ���� ������
						dPick(1);
						dSendCommand(dOutDoorToClose);
						dSendCommand(dOutGlassToClose);				
						osDelay(300); 
						state=dStateGuard;
					};												//	���� 0,3 ��� � ������� � ��������� guard							
					
				if ( (dIsButtonClose!=0) & (dIsDoorOpen!=0) & (dIsEngOn==0))	//  ������ ������ �������. �����==1, ����� �������, ��������� ���.
					{	if (dIsHeadlightsOn!=0) dPick(3);												//	���� ����==1 �� (������ 3,���,) 
						dSendCommand(dOutGlassToClose);  
						osDelay(300);
						timerGuardButDoor = globalTimerGuardButDoor;//2min			//	��������� ���� ���� 0.3 � � ��������� Guard?BUtDoor, ������ guardButDoor=2���
						state=dStateGuardButDoorOpen ; 
					};
					
				if ( (dIsButtonClose!=0) & (dIsEngOn!=0) )									//  �����==1, ��������� ����, �� ������ -
					{	dPick(4);
						dSendCommand(dOutButtonToOpen);
					};												//	������ 4, ��������� �� ������, ��������� ������ �������� �� 0 � � sensors module ���
					
				if ( (globalFlagAutostartAllowed!=0) & (dIsHandbrakeOn!=0) & (dIsEngOn!=0))//  ���� ����� �� ��������, �� �������  ������==1, �������==1, ��
					{ 
						dSendCommand(dOutEngMainToOn);
						dSendCommand(dOutEngAdditionalToOn);
						state=dStateAutostartReady;
					};// �������� ���������_���=1 � ���������_���_���=1 � � ��������� AutoStartReady
					
				counterOfAlarm = 0;					//������� ������������ = 0
				break;
					
    	case dStateGuard:// ��������� guard
				if ( dIsButtonClose == 0 ) //  ������==0,
					{	dPick(2);
						dSendCommand(dOutDoorToClose);
						osDelay(300);
						timerOpenButDoor = globalTimerOpenButDoor;//1min;//������ 2, ����� �� ��������, ���� 0,3 ��� � , ������ openButDoor=����
						state = dStateOpenButDoorClose;
					}; //������� � ��������� open_butDoor
					
				if ( (dIsDoorOpen!=0) | (dIsShockHi!=0) | (dIsEngOn!=0) )
					{	
						counterOfAlarm++;
						timerOfAlarm=globalTimerOfAlarm ; 
						if (dIsDoorOpen!=0) dSendSms(dSmsDoorOpen);
						if (dIsShockHi!=0)  dSendSms(dSmsShockHi);
						if (dIsEngOn!=0)	  dSendSms(dSmsEngOn);
						// start rec!!!!
						state=dStateAlarm ;
					};//  ����� ������� ��� ���������==1 ��� ����HI==1, �� ������� ������������ ++, timealarm=����� ��� ��� � � ��������� Alarm
					
				if (dIsShockLow!=0) 
					{
						dPick(4);//	 ����LOW==1 �� �������4
						dSendSms(dSmsShockLow);
					};
					
				if ( (dIsBeginAutostart!=0) & (flagAutostartAllowed!=0) ) //  begin_autostart==1,������� ���������� �� ����������(�������� �� autostartready) ==1
					{	
						dSendCommand(dOutEngMainToOn);
						timerOfStartEngine=globalTimeOfStartEngine/*(10sek)*/; 
						timerOfStartEngineRest=globalTimeOfStartEngineRest/*(30sek)*/;	//, �� ���������_���=1, ������ ������� ������ =10���, ������ ������=30 ���
						counterOfStarts=globalMaxOfTryStart/*3raza*/;
						osDelay(1000);
						state=dStateAutostartStartEngine;//���-�� �������=3, ���� 1���, ������� � ��������� autostart_startengine
					};
				break;
					
			case dStateAlarm: // alarm. �������� ������ ��� ������� � ������ alarmtask. ���� ������� �� ���������
				//����� ����� �� ������.���-���-���. ��� �������� ������ ���� �����������. ������ ����� �� ����� �������� ���� �� ����
				if ( (timerOfAlarm>0) & (counterOfAlarm<globalCounterOfAlarm/*3���*/) )// timeaalarm>0 � ������� �������� ����� ������, ��
					{	
						dPick(1); 
						timerOfAlarm--;
					};	//������ � ������� � alarmtask ��� ���� �������� ��� ���, timealarm--
					
				if (timerOfAlarm==0) // ���� timealarm ==0; ��
					{
						if (dIsDoorOpen!=0) sensorsMask=0xBFDFFFFF;//���� ���� ������ �� �����( �����==�������) �� � ����� ����� �� �������� �����
						counterOfAlarm++;
						state=dStateGuard;
					};	// ����� ����������� ������� ������������++(������ �� ����) � ��������� � ��������� guard
					
				if (dIsButtonClose==0) //������==0,��
					{
						timerOpenButDoor=globalTimerOpenButDoor;
						state=dStateOpenButDoorClose;
					}; //������ openButDoor=30���, ��������� open?butDoors.(��� �������� � ������ ��������� ����� �� �����������!!!)
					//���������� � ���!!!!!!
				break;
					
					
			case dStateGuardButDoorOpen:// ��������� guard,butDoor 
				
				if (timerGuardButDoor == 0)
					{
						/*SMS(door)*/; dPick(4);
						dSendSms(dSmsDoorNotClose);
						timerGuardButDoor = INT32_MAX; // ��� � ���������� �����, ��� 4, ������=���� ��� ������������� ��� ����.  ������ ������� ���������� ������, ���� ������������ ������������ � ���������� �����
						sensorsMask=0xBFDFFFFF;
						state=dStateGuard;
					};// ��������� ������ ����� � � ����� ������
					
				if (timerGuardButDoor < INT32_MAX) timerGuardButDoor--;    //������ ������ ��� ���, ������ �������������� ����������.
					
				if ((dIsDoorOpen==0) & (dIsEngOn==0))//  ����� �������,��������� ���,
					{	
						dPick(1); 
						dSendCommand(dOutDoorToClose);
						osDelay(500);
						state=dStateGuard;
					}; //������ �����, ����� �� ��������, ���� 0,3 ��� � ������� � ��������� guard 
					// ����� �������- �� �������. ���� �������� �� ���������, ���� ����� �� ����� ������, ��� ���� � ���������� ������ �� ������� 
				if (dIsEngOn !=0 ) dPick(1);//  ���������==1,�� ���1. //�������� ��������� �� ����������� �������. ���� �������� �� ������ ���� ���� �� �������� ���������
				break;
			
			case dStateOpenButDoorClose: // ��������� openButDoor. ������ �� ��������, �� ����� �� �������
				
				if (dIsDoorOpen!=0) state=dStateOpen; //����� �������, ��������� � ��������� open
			
				timerOpenButDoor--; //������ ��������					
				
				if (timerOpenButDoor==0) 
					{
						dSendCommand(dOutButtonToClose);
						osDelay(100);
						state=dStateOpen; 
					};//������ ������� �� ����, ������ 1, ��������� � ������ sensors �������� ������ =1 (����������� ������ �������) � ��������� � ����� �������- ������ ��������� ������ � guard 
					
				break;
			
			case dStateAutostartReady:// ��������� autostartready
				
				if (dIsHandbrakeOn==0) 
					{
						dSendCommand(dOutEngMainToOff);
						dSendCommand(dOutEngAdditionalToOff);
						state=dStateOpen;
					}; //���� ������ ���������, �� ���������_���=0 � ���������_���_���=0, � ��������� ������� � open
					
				if (dIsButtonClose!=0) 
					{
						dSendCommand(dOutEngMainToOff);
						dSendCommand(dOutEngAdditionalToOff);// ���� ������ ������ ������� �� ���������_���=0 � ���������_���_���=0, 
						if (dIsDoorOpen) flagAutostartAllowed=0; else flagAutostartAllowed=1;
						state=dStateOpen;
						osDelay(100);
					};//���� ���������� ����������� ������� �� ��������� �����(����� �������-��� �����) � ������� � open? ��������������� ������ � guard
				break;
			
			case dStateAutostartStartEngine:// ��������� autostart_startengine
				
				if (timerOfStartEngine>0) 
					{
						dSendCommand(dOutStarterToOn);
						timerOfStartEngine--;
					};			//���� ������_�������_������ >0 �� �������=1, ������--
					
				if (timerOfStartEngine==0) 
					{
						dSendCommand(dOutStarterToOff); 
						timerOfStartEngineRest--; 
					};			//���� ������_�������_������ ==0 �� �������=0, ������ ������--
					
				if ( (timerOfStartEngine==0) & (timerOfStartEngineRest==0) )
					{	
						counterOfStarts--;
						timerOfStartEngine=globalTimeOfStartEngine /*(10sek)*/; 
						timerOfStartEngineRest=globalTimeOfStartEngineRest/*(30sek)*/;	
					};				//���� ��� ������_�������_������==0 � ������ ������==0 �� ����������_�������--,���������� ������� ��� �� guarda/ ������ ������� ������ =10���, ������ ������=30 ���,
					
				if (counterOfStarts==0) 
					{	/*SMS("can't start'");*/ 
						dSendSms(dSmsCantStartEngine);
						dSendCommand(dOutEngMainToOff);
						//???dSendCommand(dOutEngAdditionalToOff);
						flagAutostartAllowed=0; 
						state=dStateGuard;
					};//���� ���-��_�������==0, �� ���������_���"�� ���� ���������" , ���������_���=0, ���� ���������� ������=0(����� �� ������� ����������� ������� ���� ���) � ������������ � guard
					
				if (dIsButtonClose==0) 
					{
						dSendCommand(dOutStarterToOff);
						dSendCommand(dOutEngMainToOff);
						dSendCommand(dOutEngAdditionalToOff) ;
						dPick(2) ;
						dSendCommand(dOutDoorToOpen);	
						timerOpenButDoor = globalTimerOpenButDoor;
						state=dStateGuardButDoorOpen ;
					}; // ���� ������=� ,�� �������=0, ���������_���=0 � ���������_���_���=0,������ 2, ����� �� ��������, ���� 0,3 ��� � ������� � ��������� open_butDoor, ������ openButDoor=����
					
				if (dIsEngineWork !=0) 
					{	
						dSendCommand(dOutStarterToOff);
						dSendCommand(dOutEngAdditionalToOn);
						timerOfEngineWork = globalTimerOfEngineWork ; 
						state = dStateAutostartWork;
					};// ���� �����==1(�� ��������� ��� ����.-�������������� � sensors) �� �������=0 , �� ���� 5 ���, ���������_���_��� =1 , ������_��������=� ����� � ��������� � ��������� autostart_work 
				break;
			
			case 7:// autostart_work. �������� � ������ �������
				if (dIsButtonClose==0)
					{	dSendCommand(dOutEngMainToOff); dSendCommand(dOutEngAdditionalToOff); dPick(2); dSendCommand(dOutDoorToOpen); timerOpenButDoor = globalTimerOpenButDoor; state=4;
					};// ���� ������=� ��� �����=0 ,��  ���������_���=0 � ���������_���_���=0,������ 2, ����� �� ��������, ���� 0,3 ��� � ������� � ��������� open_butDoor, ������ openButDoor=����
					
				if (timerOfEngineWork>0) timerOfEngineWork-- ;// ���� ������_��������>0 �� ������_��������--, 			
					
				if ( (timerOfEngineWork==0) | (dInTemperature>globalInTemperature) ) 
					{	
						dSendCommand(dOutEngMainToOff);
						dSendCommand(dOutEngAdditionalToOff); 
						dPick(1); 
						state = dStateGuard;
					};// ���� ������_��������==0 ��� ����������� ������ ����� �� �������, �� ���������_���=0 � ���������_���_���=0,������ 1, � � ��������� guard
				break;	
      };// end swich
		
	if (dIsDoorOpen!=0) flagAutostartAllowed=0; //���� ������� ����� �� ������ ������, �� ���� ���������� �����������=0
	
  }
 
  /* USER CODE END 5 */ 
}

/* Task_Sensors function */
void Task_Sensors(void const * argument)
{
  /* USER CODE BEGIN Task_Sensors */
	uint32_t sensorsToMain = 0x00000000; // �������� �������� ��� � ������ main 
	uint8_t commands = 0; // �������, ���������� �� main
	uint8_t phisicalButton=0, prevPhisicalButton=0, logicalButton=0;
	
  
  /* Infinite loop */
  for(;;)
  {	
		sensorsToMain = 0x00000000;
		phisicalButton=HAL_GPIO_ReadPin( button_close_GPIO_Port, button_close_Pin);  // ��������� �������� �� ���� �� ��������� ����������
		if (prevPhisicalButton^phisicalButton) logicalButton=phisicalButton;// ���� �������� ����������, �� ����� �� �������� � ����������
		prevPhisicalButton=phisicalButton;// ��������� ��� ���������
		//if ((commands)&(0x0008)) logicalButton=1;
		//if ((commands)&(0x0010)) logicalButton=0;
		//���� �������� ��� ��� ��������� ���
		if (logicalButton) sensorsToMain+=0x80000000;// ����� � �������� � main
			
		if (HAL_GPIO_ReadPin( door_open_GPIO_Port, door_open_Pin)) 							sensorsToMain+=0x40000000;// else sensorsToMain&=0xBFFFFFFF;	//������������� ��� ������� ��� ������
		if (HAL_GPIO_ReadPin( engnition_GPIO_Port, engnition_Pin)) 							sensorsToMain+=0x20000000;// else sensorsToMain&=0xDFFFFFFF;	//������������� ��� ������� ��� ������
		if (HAL_GPIO_ReadPin( headlamp_GPIO_Port, headlamp_Pin)) 								sensorsToMain+=0x10000000;// else sensorsToMain&=0xEFFFFFFF;	//������������� ��� ������� ��� ������
		if (HAL_GPIO_ReadPin( handbrake_GPIO_Port, handbrake_Pin)) 							sensorsToMain+=0x04000000;// else sensorsToMain&=0xFBFFFFFF;	//������������� ��� ������� ��� ������
		if (HAL_GPIO_ReadPin( engine_work_GPIO_Port, engine_work_Pin)) 					sensorsToMain+=0x02000000;// else sensorsToMain&=0xFDFFFFFF;	//������������� ��� ������� ��� ������
		if (HAL_GPIO_ReadPin( shock_hi_GPIO_Port, shock_hi_Pin)) 								sensorsToMain+=0x01000000;// else sensorsToMain&=0xFEFFFFFF;	//������������� ��� ������� ��� ������
		if (HAL_GPIO_ReadPin( begin_autostart_GPIO_Port, begin_autostart_Pin))  sensorsToMain+=0x00400000;// else sensorsToMain&=0xFFBFFFFF;	//������������� ��� ������� ��� ������
		
			
		xQueueReceive ( toSensorsHandle, &commands,0);
		switch (commands)
    {
		 	case dOutDoorToClose:
    		break;
    	case	dOutDoorToOpen:
    		break;
			case	dOutGlassToClose:
    		break;
			case	dOutButtonToClose:	
    		break;
    	case	dOutButtonToOpen:
    		break;
    	case	dOutEngMainToOn:
    		break;
    	case	dOutEngMainToOff:
    		break;
    	case	dOutEngAdditionalToOn:
    		break;
    	case	dOutEngAdditionalToOff:
    		break;
    	case	dOutStarterToOn:
    		break;
    	case	dOutStarterToOff:
    		break;
    	case	dOutUpdateSensors:
				xQueueSend( toMainHandle, &sensorsToMain, portMAX_DELAY );
    		break;
    };
		
		commands=0;
		osDelay(1);
		
  }
  /* USER CODE END Task_Sensors */
}

/* Task_SMS function */
void Task_SMS(void const * argument)
{
  /* USER CODE BEGIN Task_SMS */
	uint16_t delay_read_sms=2000;																			// ��������� ��� ������ ... ����
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
