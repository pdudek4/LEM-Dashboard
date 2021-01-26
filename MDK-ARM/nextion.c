#include "nextion.h"
#include <string.h>

/*===============TO DO===============================================
-dodaj struktury GPS, AKCEL i do buforów nxt wraz z ich rozmiarem i do wysylki ktory bedzie przeliczany w processdata
-dodaj odczyt zasilania z CANa od plytki baterii i sterowanie dioda na zlaczu nextiona
====================================================================*/
char k[3] = {0xff, 0xff, 0xff};

void ProcessData_P(nextion_uart_t* nx_val, uint8_t (*CAN_ramka)[CAN_FRAME_COUNT][8])
{
	//algorytm przeliczenia danych z uint8_t do wartosci parametrow
		nx_val->speed++;// = (*CAN_ramka)[0][0];
		nx_val->rpm = (*CAN_ramka)[0][1];
		nx_val->power = (*CAN_ramka)[0][2];
		nx_val->amps = (*CAN_ramka)[0][3];
}

void ProcessData_R(nextion_uart_t* nx_val, uint8_t (*CAN_ramka)[CAN_FRAME_COUNT][8])
{
		nx_val->bat_percent++;// = (*CAN_ramka)[0][4];
		nx_val->bat_voltage = (*CAN_ramka)[0][5];
		nx_val->engine_temp = (*CAN_ramka)[0][6];
		nx_val->controller_temp = (*CAN_ramka)[0][7];

		nx_val->bat_temps[0] = (*CAN_ramka)[1][0];
		nx_val->bat_temps[1] = (*CAN_ramka)[1][1];
		nx_val->bat_temps[2] = (*CAN_ramka)[1][2];
		nx_val->bat_temps[3] = (*CAN_ramka)[1][3];
		nx_val->bat_temps[4] = (*CAN_ramka)[1][4];
		nx_val->bat_temps[5] = (*CAN_ramka)[1][5];
}

void ProcessData_SD(nextion_uart_t* nx_val, uint8_t (*CAN_ramka)[CAN_FRAME_COUNT][8])
{	
		nx_val->bat_percent++;// = (*CAN_ramka)[0][4];
		nx_val->bat_voltage = (*CAN_ramka)[0][5];
		nx_val->engine_temp = (*CAN_ramka)[0][6];
		nx_val->controller_temp = (*CAN_ramka)[0][7];

		nx_val->bat_temps[0] = (*CAN_ramka)[1][0];
		nx_val->bat_temps[1] = (*CAN_ramka)[1][1];
		nx_val->bat_temps[2] = (*CAN_ramka)[1][2];
		nx_val->bat_temps[3] = (*CAN_ramka)[1][3];
		nx_val->bat_temps[4] = (*CAN_ramka)[1][4];
		nx_val->bat_temps[5] = (*CAN_ramka)[1][5];
}

void AddToBuffor_P(char* buf_nxt, nextion_uart_t* nx_val, volatile bool* do_wysyl)
{
	char value_c[16];
	char parameter[8] = "n0.val=";
	
	sprintf(value_c, "%s%d%c%c%c", parameter, nx_val->speed, 0xff, 0xff, 0xff);
  strcpy(buf_nxt, value_c);
	
	parameter[1]++;
	sprintf(value_c, "%s%d%c%c%c", parameter, nx_val->rpm, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	parameter[1]++;
	sprintf(value_c, "%s%d%c%c%c", parameter, nx_val->power, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	parameter[1]++;
	sprintf(value_c, "%s%d%c%c%c", parameter, nx_val->amps, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	*do_wysyl = true;
}

void AddToBuffor_R(char* buf_nxt, nextion_uart_t* nx_val, volatile bool* do_wysyl)
{
	char value_c[16];
	uint8_t nr = 4;
	
	sprintf(value_c, "n%d.val=%d%c%c%c", nr, nx_val->bat_percent, 0xff, 0xff, 0xff);
  strcpy(buf_nxt, value_c);
		
	nr++; //n5.val=
	sprintf(value_c, "n%d.val=%d%c%c%c", nr, nx_val->bat_voltage, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	nr++; //n6.val=
	sprintf(value_c, "n%d.val=%d%c%c%c", nr, nx_val->engine_temp, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	nr++; //n7.val=
	sprintf(value_c, "n%d.val=%d%c%c%c", nr, nx_val->controller_temp, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);

	nr++; //n8.val=
	sprintf(value_c, "n%d.val=%d%c%c%c", nr, nx_val->bat_temps[0], 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	nr++; //n9.val=
	sprintf(value_c, "n%d.val=%d%c%c%c", nr, nx_val->bat_temps[1], 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	nr++; //n10.val=
	sprintf(value_c, "n%d.val=%d%c%c%c", nr, nx_val->bat_temps[2], 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	nr++; //n11.val=
	sprintf(value_c, "n%d.val=%d%c%c%c", nr, nx_val->bat_temps[3], 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	nr++; //n12.val=
	sprintf(value_c, "n%d.val=%d%c%c%c", nr, nx_val->bat_temps[4], 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	nr++; //n13.val=
	sprintf(value_c, "n%d.val=%d%c%c%c", nr, nx_val->bat_temps[5], 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);

	*do_wysyl = true;
}

void AddToBuffor_SD(char* buf_nxt, nextion_uart_t* nx_val, volatile bool* do_wysyl)
{
	//dodaj parametry zapisywane na SD	
	//----------------1  2  3  4  5  6  7  8  9  10 11 12 13 14 15
	sprintf(buf_nxt, "%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d\r\n",
	/*			1 				   				2 										3										4												5					*/ 			
	nx_val->bat_percent, nx_val->bat_voltage, nx_val->engine_temp, nx_val->controller_temp, nx_val->bat_temps[0],
	/*			6 				   				7 										8										9												10					*/
	nx_val->bat_percent, nx_val->bat_voltage, nx_val->engine_temp, nx_val->controller_temp, nx_val->bat_temps[0],
	/*			11 				   				12 										13									14											15					*/
	nx_val->bat_percent, nx_val->bat_voltage, nx_val->engine_temp, nx_val->controller_temp, nx_val->bat_temps[0]);

	*do_wysyl = true;
}


void IdleRun(void)
{
	HAL_UART_Transmit_IT(&huart2, "IDLE\r\n", 6);	//DEBUG
	HAL_Delay(2000);
	
}

void Nextion_SendValue(char* buf_nxt, volatile bool* do_wysyl, volatile bool* free)
{
	uint16_t size_nxt;
	
		if((true == *do_wysyl) && (true == *free)){
			size_nxt = strlen(buf_nxt);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)buf_nxt, size_nxt);
			*free = false;
			*do_wysyl = false;
		}
	
}

void Nextion_SDRun(sd_card_t* sd_card, char* buf_nxt, volatile bool* do_wysyl)
{
	if(true == sd_card->init){
		SDInit(sd_card);	
		sd_card->init = false;
	}
	
	if(true == *do_wysyl){
		f_lseek(&(sd_card->myFile), f_size(&(sd_card->myFile)));
		f_write(&(sd_card->myFile), buf_nxt, strlen(buf_nxt), &(sd_card->myBytes));
		f_sync(&(sd_card->myFile)); //zapis fat zapobiega utracie danych

						HAL_UART_Transmit_IT(&huart2, "SD_Run\r\n", 8);
		*do_wysyl = false;
	}
}

void Process_uart(dash_state_t* dash_state, uint8_t* Uart2_buf, sd_card_t* sd_card)
{	
	if(NEXTION_SD == *dash_state){
		SDkoniec(sd_card);
	}
	
		switch(Uart2_buf[2]){
		case NX_BUTTON_IDLE:
			*dash_state = IDLE;
			break;
		case NX_BUTTON_P:
			*dash_state = NEXTION_P;
			HAL_TIM_Base_Stop_IT(&htim2);
			HAL_TIM_Base_Stop_IT(&htim3);
			break;
		case NX_BUTTON_R:
			*dash_state = NEXTION_R;
			HAL_TIM_Base_Stop_IT(&htim2);
			__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
			HAL_TIM_Base_Start_IT(&htim3);
			break;
		case NX_BUTTON_SD:
			if(NEXTION_SD == *dash_state){
				*dash_state = IDLE; // docelowo NEXTION_P
			}
  		else{
				HAL_TIM_Base_Stop_IT(&htim1);
				HAL_TIM_Base_Stop_IT(&htim3);
				sd_card->init = true;
	  		*dash_state = NEXTION_SD;
				__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
				HAL_TIM_Base_Start_IT(&htim2);
			}
			break;
		default: 
			*dash_state = IDLE;
		  break;
	}
}


void SDInit(sd_card_t* sd_card)
{
	if(f_mount(&(sd_card->myFatFS), SDPath, 1) == FR_OK){
		//wyslanie pierwszej ikonki na LCD
		HAL_UART_Transmit(&huart2, "vis p0,1", 8, 150);
		HAL_UART_Transmit(&huart2, (uint8_t*)k, 3, 100);
		if(f_open(&(sd_card->myFile), sd_card->SD_nazwapliku, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK){
			//jezeli udalo sie utworzyc plik na sd to wyslij druga ikonke na LCD
			HAL_UART_Transmit(&huart2, "vis p1,1", 8, 150);
			HAL_UART_Transmit(&huart2, (uint8_t*)k, 3, 100);
		}
		else{
			HAL_UART_Transmit(&huart2, "vis p0,0", 8, 150); //usuniecie pierwszej ikonki z lcd
			HAL_UART_Transmit(&huart2, (uint8_t*)k, 3, 100);
		}
	}
	else{
		HAL_UART_Transmit(&huart2, "vis p1,0", 8, 150);		//usuniecie pierwszej ikonki z lcd
		HAL_UART_Transmit(&huart2, (uint8_t*)k, 3, 100);
	}
	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
	HAL_TIM_Base_Start_IT(&htim1);
							HAL_UART_Transmit(&huart2, "SD_Init\r\n", 9, 200);
}

void SDkoniec(sd_card_t* sd_card)
{
	HAL_TIM_Base_Stop_IT(&htim2);
	char nowa_nazwa[7];
  static int nr_pliku = 1;

	f_close(&(sd_card->myFile));
	//usuniecie ikonek z LCD
	HAL_UART_Transmit(&huart2, "vis p0,0", 8, 150);
	HAL_UART_Transmit(&huart2, (uint8_t*)k, 3, 100);
	HAL_UART_Transmit(&huart2, "vis p1,0", 8, 150);
	HAL_UART_Transmit(&huart2, (uint8_t*)k, 3, 100);
	//inkrementacja nazwy (nr) pliku
	nr_pliku++;
  sprintf(nowa_nazwa, "%d.TXT\0", nr_pliku);
  strcpy(sd_card->SD_nazwapliku, nowa_nazwa);
				HAL_UART_Transmit_IT(&huart2, "SD_Koniec\r\n", 11);
}











