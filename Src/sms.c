#include "sms.h"
#include "string.h"
#include "main.h"
#include "cmsis_os.h"

extern UART_HandleTypeDef huart2;
extern osMessageQId toSMSHandle;
char RxBuffer[200];
uint8_t ID=0;

//------------------------------------------------------------------------
uint8_t RecieveData (void)
{
	uint16_t recieve_word;
	uint8_t RxCounter=0;
	
	if (xQueuePeek(toSMSHandle,&recieve_word,0)==pdPASS)
		while (xQueueReceive(toSMSHandle,&recieve_word,10)==pdPASS) 
		{
			ID=( recieve_word>>8);
			uint8_t DATA= (uint8_t) (recieve_word&0xFF);
		  if (ID==1) RxBuffer[RxCounter++]=DATA ;			
		};
	RxBuffer[RxCounter]=0;
	
	return RxCounter;		
};
//--------------------------------------------------------------------------
uint8_t toNeoway(char* text, uint16_t delay_time)
{
	char* Void_str="";
	char* OK="\r\nOK\r\n";//OK="\r\nOK\r\n";
	char* ERROR="\r\nERROR\r\n";
	char* sms_ready="\r\n> ";
	
	HAL_UART_Transmit_IT(&huart2,(uint8_t *) text, strlen(text));

	while (delay_time--) 
	{
		osDelay(1);
		if (RecieveData()!=0) break;
	};
	if (strcmp(RxBuffer,Void_str)==0) return 0;
	if (strcmp(RxBuffer,OK)==0) return 1;
	if (strcmp(RxBuffer,ERROR)==0) return 2;
	if (strcmp(RxBuffer,sms_ready)==0) return 3;
	if (strstr(RxBuffer,"OK")!=NULL) return 4;
};
//--------------------------------------------------------------------------
void Neoway_init(void)
{	
	while (toNeoway("ATE0\r",100)!=1) /*HAL_UART_Transmit_IT(&huart2,"notOK\r",6)*/; 																						//	выключить эхо
	while (toNeoway("AT+enpwrsave=0\r",100)!=1) /*HAL_UART_Transmit_IT(&huart2,"notOK\r",6*/;																	//	выключить спящий режим
	while (toNeoway("AT+CLIP=1\r",100)!=1)/* HAL_UART_Transmit_IT(&huart2,"notOK\r",6)*/;																				//	АОН включен
	while (toNeoway("AT+CMGF=1\r",100)!=1)/* HAL_UART_Transmit_IT(&huart2,"notOK\r",6*/;																				//	текстовый формат смс
	while (toNeoway("AT+CSCS=\"GSM\"\r",100)!=1)/*HAL_UART_Transmit_IT(&huart2,"notOK\r",6)*/;																	// формат ASCII 
};
//----------------------------------------------------------------------------
void Send_SMS (char *number, char *text)
{
	char buffer[24]="AT+CMGS=\"+7";
	char sym1A=0x1A;
	strcat(buffer,number);
	strcat(buffer,"\"\r");
	while (toNeoway(buffer,1000)!=3); HAL_UART_Transmit_IT(&huart2,"not >\r",6);																	// формат ASCII toNeoway(100);
	strcpy(buffer,text);
	strcat(buffer,&sym1A); 
	if (toNeoway(buffer,1000)==4 );// HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, 1);
};
//-------------------------------------------------------------------------------
void Read_SMS(void)
{
	if (toNeoway("AT+CMGR=1\r",1000)!=2)
		{
			if (strstr(RxBuffer,admin_number)!=NULL) 
			{
				if (strstr(RxBuffer,"Open")!=NULL)	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
				if (strstr(RxBuffer,"Sms")!=NULL)	Send_SMS(admin_number," I recive SMS"); ;
				if (strstr(RxBuffer,"Call")!=NULL)	Call(admin_number,20000);
			};
			while (toNeoway("AT+CMGD=1,4\r",1000)!=1);	
		};
	
};
//---------------------------------------------------------------------------------
void Call(char *number, uint16_t time)
{
	char buffer[17]="ATD+7";
	strcat(buffer,number);
	strcat(buffer,";\r");
	toNeoway(buffer,100);
	osDelay(time);
	toNeoway("ATH\r",100);
	osDelay(1000);
};
//---------------------------------------------------------------------------------
void MonitoringRing(void)
{
	if (RecieveData()!=0) 
		if ((strstr(RxBuffer,"RING")!=NULL)&(strstr(RxBuffer,admin_number)!=NULL)) HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin,1);
	switch (ID)
	{
		case 2:			//позвонить админу
			Call(admin_number,15000);
			ID=0;
			break;
		case 3:				//отправить смс
			ID=0;
			break;
	};
	
};














