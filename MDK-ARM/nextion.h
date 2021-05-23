
#ifndef FILE_H
#define FILE_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "fatfs.h"

#define CAN_FRAME_COUNT 4

#define CAN_ADR_ZAPI0 0x300
#define CAN_ADR_ZAPI1 0x310

#define CAN_ADR_BMS0 	0x400
#define CAN_ADR_BMS1	0x410
#define CAN_ADR_BMS2	0x420

#define CAN_ADR_PDM 	0x500

#define CAN_ADR_SENS0	0x600
#define CAN_ADR_SENS1	0x610
#define CAN_ADR_SENS2 0x620
#define CAN_ADR_SENS3 0x630
#define CAN_ADR_SENS4 0x640

#define NX_BUTTON_SD 0x01

extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;
extern char k[3];

typedef struct {
	
	uint8_t speed;						//
	uint16_t rpm;
	uint8_t power; 						//
	uint8_t amps;	
	uint8_t bat_percent;			//
	uint8_t bat_voltage;
	uint8_t engine_temp;			//
	uint8_t controller_temp;
	uint8_t bat_temps[6];			//
	
} nextion_uart_t;

typedef struct {
	
	uint8_t Iq;
	uint8_t Id;
	uint8_t Uq;
	uint8_t Ud;
	
} zapi_par_t;

typedef struct {

	FATFS myFatFS;
	FIL myFile;
	UINT myBytes;
	char SD_nazwapliku[7];
	bool init;
	
} sd_card_t;

typedef enum {
	IDLE,
	NEXTION_P,
	NEXTION_R,
	NEXTION_SD	
} dash_state_t;

typedef enum {
	PAGE0,
	PAGE1,
	PAGE2,
	PAGE3	
} dash_page_t;

void ProcessData_P(nextion_uart_t* nx_val, uint8_t (*CAN_ramka)[CAN_FRAME_COUNT][8]);
void ProcessData_R(nextion_uart_t* nx_val, uint8_t (*CAN_ramka)[CAN_FRAME_COUNT][8]);
void ProcessData_SD(nextion_uart_t* nx_val, uint8_t (*CAN_ramka)[CAN_FRAME_COUNT][8]);
void ProcessData_All(nextion_uart_t* nx_val, uint8_t (*CAN_ramka)[CAN_FRAME_COUNT][8]);

void AddToBuffor_P(char* buf_nxt, nextion_uart_t* nx_val, volatile bool* do_wysyl);
void AddToBuffor_R(char* buf_nxt, nextion_uart_t* nx_val, volatile bool* do_wysyl);
void AddToBuffor_SD(char* buf_nxt, nextion_uart_t* nx_val, volatile bool* do_wysyl);

void IdleRun(void);
void Nextion_SendValue( char* buf_nxt, volatile bool* do_wysyl, volatile bool* free);
void Nextion_SDRun(sd_card_t* sd_card, char* buf_nxt, volatile bool* do_wysyl);

void Process_uart(dash_state_t* dash_state,  dash_page_t* dash_page, uint8_t* Uart2_buf, sd_card_t* sd_card);

void SDInit(sd_card_t* sd_card);
void SDkoniec(sd_card_t* sd_card);
void LED0_Off();
void LED0_On();

void CanGetMsgData(uint8_t* aData);


#endif