
#ifndef FILE_H
#define FILE_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "fatfs.h"

#define CAN_FRAME_COUNT 8

#define CAN_ADR_ZAPI0 0x388
#define CAN_ADR_ZAPI1 0x288
#define CAN_ADR_ZAPI2 0x588

#define CAN_ADR_BMS0 	0x400
#define CAN_ADR_BMS1	0x410
#define CAN_ADR_BMS2	0x420

#define CAN_ADR_PDM 	0x700

#define CAN_ADR_SENS0	0x600  //pot1 rear
#define CAN_ADR_SENS1	0x610  //pot2 front
#define CAN_ADR_SENS2 0x620
#define CAN_ADR_SENS3 0x630
#define CAN_ADR_SENS4 0x640

#define NX_BUTTON_SD 	0x01

#define Vol_nom				117
#define Vol_presc			(1000/Vol_nom)
#define PRZELOZENIE		4.615
#define PROMIEN_KOLA	0.30

extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;
extern char k[3];

typedef struct {
	uint16_t min;
	uint16_t max;
	uint16_t avg;	
	
} susp_t;

typedef struct {
	uint8_t status_field;
	uint8_t temps[5];
	uint16_t imd_resistance;

} pdm_t;

typedef struct {
	uint8_t zapi;
	uint8_t pdm;
	uint8_t sensory;
} heartbeat_t;

typedef struct {
	
	uint8_t speed;						//
	uint16_t rpm;
	uint8_t power; 						//
	uint16_t amps;
	uint8_t bat_percent;			//
	uint8_t bat_voltage;
	uint8_t engine_temp;			//
	uint8_t controller_temp;
	susp_t susp_front;			//
	susp_t susp_rear;
	pdm_t pdm_val;
	bool contactor;
	uint32_t can_count;
	
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
	bool flaga;
	bool sd_add_buf;
	
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

//void ProcessData_P(nextion_uart_t* nx_val, uint8_t (*CAN_ramka)[CAN_FRAME_COUNT][8]);
//void ProcessData_R(nextion_uart_t* nx_val, uint8_t (*CAN_ramka)[CAN_FRAME_COUNT][8]);
//void ProcessData_SD(nextion_uart_t* nx_val, uint8_t (*CAN_ramka)[CAN_FRAME_COUNT][8]);
void ProcessData_All(nextion_uart_t* nx_val, uint8_t (*CAN_ramka)[CAN_FRAME_COUNT][8]);

void AddToBuffor_P(char* buf_nxt, nextion_uart_t* nx_val, volatile bool* do_wysyl);
void AddToBuffor_R(char* buf_nxt, nextion_uart_t* nx_val, volatile bool* do_wysyl, uint8_t (*CAN_ramka)[CAN_FRAME_COUNT][8]);
void AddToBuffor_SD(char* buf_sd, nextion_uart_t* nx_val, volatile bool* do_wysyl);

void IdleRun(void);
void Nextion_SendValue( char* buf_nxt, volatile bool* do_wysyl, volatile bool* free);
void SDZapis(sd_card_t* sd_card, char* buf_zap, char* buf_usun, volatile bool* do_wysyl);

void Process_uart(dash_state_t* dash_state,  dash_page_t* dash_page, uint8_t* Uart2_buf, sd_card_t* sd_card);

void SDInit(sd_card_t* sd_card);
void SDkoniec(sd_card_t* sd_card);
void LED0_Off();
void LED0_On();

void CanGetMsgData(uint8_t* aData);


#endif