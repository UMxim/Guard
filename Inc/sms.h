#include "stm32f0xx_hal.h"

extern char admin_number[];

//void Send_SMS (char *number, char *text);
void Read_SMS(void);
void Call(char *, uint16_t);
void MonitoringRing(void);

uint8_t RecieveData (void);
void Neoway_init(void);

