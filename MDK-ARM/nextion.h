
#ifndef FILE_H
#define FILE_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "fatfs.h"

#define CAN_FRAME_COUNT 8

#define CAN_ADR_ZAPI0 0x388
#define CAN_ADR_ZAPI1 0x288
#define CAN_ADR_ZAPI2 0x588

#define CAN_ADR_SENS0	0x600  //pot1 rear
#define CAN_ADR_SENS1	0x610  //pot2 front
#define CAN_ADR_SENS2 0x620	 //yaw pich roll
#define CAN_ADR_SENS3 0x630  // acell x y z
#define CAN_ADR_SENS4 0x640
#define CAN_ADR_PDM 	0x700  //pdm->temps

#define NX_BUTTON_SD 	0x01

#define	ZAPI_BAT_VOLTAGE	0x02
#define	ZAPI_CONTR_TEMP		0x11
#define	ZAPI_MOT_TEMP			0x12
#define	ZAPI_ID_CURR			0x09
#define	ZAPI_IQ_CURR			0x0A
#define	ZAPI_DC_CURR			0x18
#define	ZAPI_TORQUE				0x0E

#define Vol_nom				117
#define Vol_presc			(1000/Vol_nom)
#define PRZELOZENIE		4.615
#define PROMIEN_KOLA	0.30
#define MAX_RPM				70

extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;
extern char k[3];

typedef struct {
	int16_t min;
	int16_t max;
	int16_t avg;	
	
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

typedef struct{
	int16_t yaw;
	int16_t pitch;
	int16_t roll;

	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	
} imu_t;

typedef struct {
	uint8_t speed;						//
	uint16_t rpm;
	uint8_t power; 						//
	uint8_t bat_percent;			//
	uint8_t bat_voltage;
	uint8_t engine_temp;			//
	uint8_t controller_temp;
	
	int16_t id_curr;
	int16_t iq_curr;
	uint16_t DC_curr;
	uint16_t torque;
	
	susp_t susp_front;			//
	susp_t susp_rear;
	pdm_t pdm_val;
	imu_t imu_data;
	
	bool contactor;
	uint32_t can_count;
	
	uint16_t SDO_req;
	uint16_t SDO_ans;
	
} nextion_uart_t;

typedef struct {

	FATFS myFatFS;
	FIL myFile;
	UINT myBytes;
	char SD_nazwapliku[7];
	bool init;
	bool flaga;							//identyfikator zapisu do buf_sd1 lub buf_sd2
	bool sd_add_buf;				//flaga ustawiana co 5Hz dodajaca linie do buf_sdx
	
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

void ProcessData_All(nextion_uart_t* nx_val, uint8_t (*CAN_ramka)[CAN_FRAME_COUNT][8]);

void AddToBuffor_P(char* buf_nxt, nextion_uart_t* nx_val, bool* do_wysyl);
void AddToBuffor_R(char* buf_nxt, nextion_uart_t* nx_val, bool* do_wysyl);
void AddToBuffor_SD(char* buf_sd, nextion_uart_t* nx_val, bool* do_wysyl);

void IdleRun(void);
void Nextion_SendValue( char* buf_nxt, bool* do_wysyl, bool* free);
void SDZapis(sd_card_t* sd_card, char* buf_zap, char* buf_usun, bool* do_wysyl);

void Process_uart(dash_state_t* dash_state,  dash_page_t* dash_page, uint8_t* Uart2_buf, sd_card_t* sd_card);

void SDInit(sd_card_t* sd_card);
void SDkoniec(sd_card_t* sd_card);
void LED0_Off(void);
void LED0_On(void);

void CanGetMsgData(uint8_t* aData);


#endif
