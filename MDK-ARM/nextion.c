#include "nextion.h"
#include <string.h>

//#define _DEBUG
/*===============TO DO===============================================
-dodaj struktury GPS, AKCEL i do buforów nxt wraz z ich rozmiarem i do wysylki ktory bedzie przeliczany w processdata
-dodaj odczyt zasilania z CANa od plytki baterii i sterowanie dioda na zlaczu nextiona
-dodaj wyslanie napiec z bmsa (wartosci 'x' float x.yy) processData z nowym timerem o malej f
====================================================================*/
char k[3] = {0xff, 0xff, 0xff};

void ProcessData_P(nextion_uart_t* nx_val, uint8_t (*CAN_ramka)[CAN_FRAME_COUNT][8])
{
	//algorytm przeliczenia danych z uint8_t do wartosci parametrow
		nx_val->speed++;// = (*CAN_ramka)[0][0];
	if(nx_val->speed >200) nx_val->speed = 0;
//		nx_val->rpm++;// = (*CAN_ramka)[0][1];
//		nx_val->power = (*CAN_ramka)[0][2];
//		nx_val->amps = (*CAN_ramka)[0][3];
//		nx_val->bat_percent++;// = (*CAN_ramka)[0][4];

}

void ProcessData_R(nextion_uart_t* nx_val, uint8_t (*CAN_ramka)[CAN_FRAME_COUNT][8])
{
		//nx_val->bat_percent++;// = (*CAN_ramka)[0][4];
	if(nx_val->bat_voltage++ > 90) nx_val->bat_voltage = 0;
		nx_val->bat_voltage++;// = (*CAN_ramka)[0][5];
//		nx_val->engine_temp = (*CAN_ramka)[0][6];
//		nx_val->controller_temp = (*CAN_ramka)[0][7];

//		nx_val->bat_temps[0] = (*CAN_ramka)[1][0];
//		nx_val->bat_temps[1] = (*CAN_ramka)[1][1];
//		nx_val->bat_temps[2] = (*CAN_ramka)[1][2];
//		nx_val->bat_temps[3] = (*CAN_ramka)[1][3];
//		nx_val->bat_temps[4] = (*CAN_ramka)[1][4];
//		nx_val->bat_temps[5] = (*CAN_ramka)[1][5];
}

void ProcessData_SD(nextion_uart_t* nx_val, uint8_t (*CAN_ramka)[CAN_FRAME_COUNT][8])
{	
		nx_val->bat_percent++;// = (*CAN_ramka)[0][4];
	if( nx_val->bat_percent > 99) nx_val->bat_percent = 0;
//		nx_val->bat_voltage = (*CAN_ramka)[0][5];
//		nx_val->engine_temp = (*CAN_ramka)[0][6];
//		nx_val->controller_temp = (*CAN_ramka)[0][7];

//		nx_val->bat_temps[0] = (*CAN_ramka)[1][0];
//		nx_val->bat_temps[1] = (*CAN_ramka)[1][1];
//		nx_val->bat_temps[2] = (*CAN_ramka)[1][2];
//		nx_val->bat_temps[3] = (*CAN_ramka)[1][3];
//		nx_val->bat_temps[4] = (*CAN_ramka)[1][4];
//		nx_val->bat_temps[5] = (*CAN_ramka)[1][5];
}

void ProcessData_All(nextion_uart_t* nx_val, uint8_t (*CAN_ramka)[CAN_FRAME_COUNT][8])
{	

//	nx_val->speed++;// = (*CAN_ramka)[0][0];
//	if(nx_val->speed >200) nx_val->speed = 0;
//	nx_val->rpm+=5;// = (*CAN_ramka)[0][1];
//	if(nx_val->rpm > 7000) nx_val->rpm = 0;
//	nx_val->power++;// = (*CAN_ramka)[0][2];
//	nx_val->amps+=5;// (*CAN_ramka)[0][3];
//	if(nx_val->amps > 255) nx_val->amps = 0;
//	nx_val->bat_percent++;// = (*CAN_ramka)[0][4];
//	if( nx_val->bat_percent > 99) nx_val->bat_percent = 0;
//	nx_val->bat_voltage = 112;//(*CAN_ramka)[0][5];
//	nx_val->engine_temp = 50;//(*CAN_ramka)[0][6];
//	nx_val->controller_temp = 60;//(*CAN_ramka)[0][7];

//		nx_val->bat_temps[0] = (*CAN_ramka)[1][0];
//		nx_val->bat_temps[1] = (*CAN_ramka)[1][1];
//		nx_val->bat_temps[2] = (*CAN_ramka)[1][2];
//		nx_val->bat_temps[3] = (*CAN_ramka)[1][3];
//		nx_val->bat_temps[4] = (*CAN_ramka)[1][4];
//		nx_val->bat_temps[5] = (*CAN_ramka)[1][5];
	uint16_t speedtmp = ((*CAN_ramka)[0][2] << 8) + (*CAN_ramka)[0][3];
	unsigned int sptmp;
	
	nx_val->rpm = speedtmp * 0.15;
	sptmp = nx_val->rpm * 0.0314;
	nx_val->speed = (uint8_t) sptmp;
	
//	nx_val->engine_temp = (*CAN_ramka)[1][2] - 40;
//	if (nx_val->engine_temp < 0) nx_val->engine_temp = 1;
//	
//	nx_val->controller_temp = (*CAN_ramka)[1][3] - 40;
//	if (nx_val->controller_temp < 0) nx_val->controller_temp = 2;
//	
//	nx_val->bat_percent = (*CAN_ramka)[1][4];
//	nx_val->amps = ((*CAN_ramka)[1][5]) * 2;
//	nx_val->bat_voltage = ((*CAN_ramka)[1][6]) * Vol_presc;
	
}

void AddToBuffor_P(char* buf_nxt, nextion_uart_t* nx_val, volatile bool* do_wysyl)
{
	char value_c[16];
	char parameter[8] = "n1.val=";
	
	sprintf(value_c, "%s%d%c%c%c", parameter, nx_val->speed, 0xff, 0xff, 0xff);
  strcpy(buf_nxt, value_c);
	
	parameter[1]++;
	sprintf(value_c, "%s%d%c%c%c", parameter, nx_val->rpm, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	parameter[1]++;
	sprintf(value_c, "%s%d%c%c%c", parameter, nx_val->power, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	parameter[1]++;
	sprintf(value_c, "%s%d%c%c%c", parameter, nx_val->engine_temp, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	parameter[1]++;
	sprintf(value_c, "%s%d%c%c%c", parameter, nx_val->bat_percent, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	*do_wysyl = true;
}

void AddToBuffor_R(char* buf_nxt, nextion_uart_t* nx_val, volatile bool* do_wysyl)
{
	char value_c[16];
	
	//n2.val=
	sprintf(value_c, "n2.val=%d%c%c%c", nx_val->rpm, 0xff, 0xff, 0xff);
  strcpy(buf_nxt, value_c);
	
	//n3.val=
	sprintf(value_c, "n3.val=%d%c%c%c", nx_val->power, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	//n4.val=
	sprintf(value_c, "n4.val=%d%c%c%c", nx_val->engine_temp, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	//n6.val=
	sprintf(value_c, "n6.val=%d%c%c%c", nx_val->bat_voltage, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	//n7.val=
	sprintf(value_c, "n7.val=%d%c%c%c", nx_val->amps, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	//n8.val=
	sprintf(value_c, "n8.val=%d%c%c%c", nx_val->controller_temp, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);

	//n9.val=
	sprintf(value_c, "n9.val=%d%c%c%c", nx_val->bat_temps[0], 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	//n10.val=
	sprintf(value_c, "n10.val=%d%c%c%c", nx_val->bat_temps[1], 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	//n11.val=
	sprintf(value_c, "n11.val=%d%c%c%c", nx_val->bat_temps[2], 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	//n12.val=
	sprintf(value_c, "n12.val=%d%c%c%c", nx_val->bat_temps[3], 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	//n13.val=
	sprintf(value_c, "n13.val=%d%c%c%c", nx_val->bat_temps[4], 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	//n14.val=
	sprintf(value_c, "n14.val=%d%c%c%c", nx_val->bat_temps[5], 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);

	*do_wysyl = true;
}

void AddToBuffor_SD(char* buf_nxt, nextion_uart_t* nx_val, volatile bool* do_wysyl)
{
	//dodaj parametry zapisywane na SD	
	//----------------1  2  3  4  5  6  7  8  9  10 11 12 13 14 15
	sprintf(buf_nxt, "%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d\r\n",
	/*			1 				   				2 										3										4												5					*/ 			
	nx_val->speed, nx_val->bat_voltage, nx_val->engine_temp, nx_val->controller_temp, nx_val->bat_temps[0],
	/*			6 				   				7 										8										9												10					*/
	nx_val->bat_percent, nx_val->bat_voltage, nx_val->engine_temp, nx_val->controller_temp, nx_val->bat_temps[0],
	/*			11 				   				12 										13									14											15					*/
	nx_val->bat_percent, nx_val->bat_voltage, nx_val->engine_temp, nx_val->controller_temp, nx_val->bat_temps[0]);

	*do_wysyl = true;
}


void IdleRun(void)
{
	//docelowo migaj jakas dioda wirtualna na LCD
	#ifdef _DEBUG
	HAL_UART_Transmit_IT(&huart2, "IDLE\r\n", 6);	//DEBUG
	HAL_Delay(2000);
	#endif
	
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

		*do_wysyl = false;
		#ifdef _DEBUG
			HAL_UART_Transmit_IT(&huart2, "SD_Run\r\n", 8);
		#endif
	}
}

void Process_uart(dash_state_t* dash_state, dash_page_t* dash_page, uint8_t* Uart2_buf, sd_card_t* sd_card)
{
	
	if(Uart2_buf[0] == 0x65){
		switch(Uart2_buf[2]){
			case 0x0A:
				if(NEXTION_SD == *dash_state){
					//HAL_TIM_Base_Stop_IT(&htim2);
					//dodaj zmienna spr stan czy napewno jest running (potwierdzenie otwarcia pliku)
					SDkoniec(sd_card);
					*dash_state = NEXTION_P;
					LED0_Off();
				}
				else{
					sd_card->init = true;
					*dash_state = NEXTION_SD;
					LED0_On();
//					__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
//					HAL_TIM_Base_Start_IT(&htim2);
				}
			break;
			default: 
				*dash_state = NEXTION_P;
				break;
		}
	}
	else if(Uart2_buf[0] == 0x66){
		switch(Uart2_buf[1]){
			case 0x00:
				HAL_TIM_Base_Stop_IT(&htim3);
				__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
				*dash_page = PAGE0;
				if( *dash_state == NEXTION_SD ){
					LED0_On();
				}
				HAL_TIM_Base_Start_IT(&htim1);
				break;
			case 0x01:
				HAL_TIM_Base_Stop_IT(&htim1);
				__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
				HAL_TIM_Base_Start_IT(&htim3);
				*dash_page = PAGE1;
				break;
			case 0x02:
				HAL_TIM_Base_Stop_IT(&htim1);
				HAL_TIM_Base_Stop_IT(&htim3);
				*dash_page = PAGE2;
				break;
			case 0x03:
				HAL_TIM_Base_Stop_IT(&htim3);
				__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
				*dash_page = PAGE3;
				if( *dash_state == NEXTION_SD ){
					LED0_On();
				}
				HAL_TIM_Base_Start_IT(&htim1);
			break;
			default:
				//*dash_page = PAGE0;
			  break;
		}
	}
}


void SDInit(sd_card_t* sd_card)
{
	if(f_mount(&(sd_card->myFatFS), SDPath, 1) == FR_OK){
		//wyslanie pierwszej ikonki na LCD
		HAL_UART_Transmit(&huart2, "vis p1,1", 8, 150);
		HAL_UART_Transmit(&huart2, (uint8_t*)k, 3, 100);
		if(f_open(&(sd_card->myFile), sd_card->SD_nazwapliku, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK){
			//jezeli udalo sie utworzyc plik na sd to wyslij druga ikonke na LCD
			HAL_UART_Transmit(&huart2, "vis p2,1", 8, 150);
			HAL_UART_Transmit(&huart2, (uint8_t*)k, 3, 100);
		}
		else{
			HAL_UART_Transmit(&huart2, "vis p1,0", 8, 150); //usuniecie pierwszej ikonki z lcd
			HAL_UART_Transmit(&huart2, (uint8_t*)k, 3, 100);
		}
	}

	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
	HAL_TIM_Base_Start_IT(&htim2);
	
	#ifdef _DEBUG
		 HAL_UART_Transmit(&huart2, "SD_Init\r\n", 9, 200);
	#endif
}

void SDkoniec(sd_card_t* sd_card)
{
	//HAL_TIM_Base_Stop_IT(&htim2);
	char nowa_nazwa[7];
  static int nr_pliku = 1;

	f_close(&(sd_card->myFile));
	//usuniecie ikonek z LCD
	HAL_UART_Transmit(&huart2, "vis p1,0", 8, 150);
	HAL_UART_Transmit(&huart2, (uint8_t*)k, 3, 100);
	HAL_UART_Transmit(&huart2, "vis p2,0", 8, 150);
	HAL_UART_Transmit(&huart2, (uint8_t*)k, 3, 100);
	//inkrementacja nazwy (nr) pliku
	nr_pliku++;
  sprintf(nowa_nazwa, "%d.TXT\0", nr_pliku);
  strcpy(sd_card->SD_nazwapliku, nowa_nazwa);
	
	#ifdef _DEBUG
		 HAL_UART_Transmit_IT(&huart2, "SD_Koniec\r\n", 11);
	#endif
}

void LED0_Off()
{
	HAL_UART_Transmit(&huart2, "vis p0,0", 8, 150);
	HAL_UART_Transmit(&huart2, (uint8_t*)k, 3, 100);	
}

void LED0_On()
{
	HAL_UART_Transmit(&huart2, "vis p0,1", 8, 150);
	HAL_UART_Transmit(&huart2, (uint8_t*)k, 3, 100);	
}

void CanGetMsgData(uint8_t* aData)
{
  /* Get the data */
  aData[0] = (uint8_t)((CAN_RDL0R_DATA0 & hcan1.Instance->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA0_Pos);
  aData[1] = (uint8_t)((CAN_RDL0R_DATA1 & hcan1.Instance->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA1_Pos);
  aData[2] = (uint8_t)((CAN_RDL0R_DATA2 & hcan1.Instance->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA2_Pos);
  aData[3] = (uint8_t)((CAN_RDL0R_DATA3 & hcan1.Instance->sFIFOMailBox[0].RDLR) >> CAN_RDL0R_DATA3_Pos);
  aData[4] = (uint8_t)((CAN_RDH0R_DATA4 & hcan1.Instance->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA4_Pos);
  aData[5] = (uint8_t)((CAN_RDH0R_DATA5 & hcan1.Instance->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA5_Pos);
  aData[6] = (uint8_t)((CAN_RDH0R_DATA6 & hcan1.Instance->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA6_Pos);
  aData[7] = (uint8_t)((CAN_RDH0R_DATA7 & hcan1.Instance->sFIFOMailBox[0].RDHR) >> CAN_RDH0R_DATA7_Pos);	
	
	/* Release RX FIFO 0 */
  SET_BIT(hcan1.Instance->RF0R, CAN_RF0R_RFOM0);
}








