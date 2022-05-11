#include "nextion.h"
#include <string.h>

//#define _DEBUG
/*===============TO DO===============================================
-dodaj strukture GPS i do buforów nxt wraz z ich rozmiarem i do wysylki ktory bedzie przeliczany w processdata
-dodaj odczyt zasilania z CANa od plytki baterii i sterowanie dioda na zlaczu nextiona
-dodaj wyslanie napiec z bmsa (wartosci 'x' float x.yy) processData z nowym timerem o malej f
====================================================================*/
char k[3] = {0xff, 0xff, 0xff};
bool flaga_led = false;
 /*
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
*/
void ProcessData_All(nextion_uart_t* nx_val, uint8_t (*CAN_ramka)[CAN_FRAME_COUNT][8])
{	
	float fbat;
	float amp_tmp;
	
	(*CAN_ramka)[0][0] &= 1;
  nx_val->contactor = !((*CAN_ramka)[0][0]);
	
	uint16_t speedtmp = ((*CAN_ramka)[0][2] << 8) + (*CAN_ramka)[0][3];
	unsigned int sptmp;
	
	nx_val->rpm = speedtmp * 0.15;
	sptmp = nx_val->rpm * 0.02358912f*0.8f;
//	sptmp = nx_val->rpm * 0.0233672f;
	nx_val->speed = (uint8_t) sptmp;
	
	fbat = (nx_val->bat_voltage-84)*3.05;
	nx_val->bat_percent = (uint8_t) fbat;

	amp_tmp = nx_val->DC_curr / 4;
	nx_val->DC_curr = (uint16_t) amp_tmp;
	
	//pot 1
	nx_val->susp_rear.min = ((*CAN_ramka)[2][1] << 8) + (*CAN_ramka)[2][0];
	nx_val->susp_rear.max = ((*CAN_ramka)[2][3] << 8) + (*CAN_ramka)[2][2];
	nx_val->susp_rear.avg = ((*CAN_ramka)[2][5] << 8) + (*CAN_ramka)[2][4];

	//pot2
	nx_val->susp_front.min = ((*CAN_ramka)[3][1] << 8) + (*CAN_ramka)[3][0];
	nx_val->susp_front.max = ((*CAN_ramka)[3][3] << 8) + (*CAN_ramka)[3][2];
	nx_val->susp_front.avg = ((*CAN_ramka)[3][5] << 8) + (*CAN_ramka)[3][4];
	
	//gyro
	nx_val->imu_data.yaw = ((*CAN_ramka)[4][1] << 8) + (*CAN_ramka)[4][0];
	nx_val->imu_data.pitch = ((*CAN_ramka)[4][3] << 8) + (*CAN_ramka)[4][2];
	nx_val->imu_data.roll = ((*CAN_ramka)[4][5] << 8) + (*CAN_ramka)[4][4];
	
	//acc
	nx_val->imu_data.acc_x = ((*CAN_ramka)[5][1] << 8) + (*CAN_ramka)[5][0];
	nx_val->imu_data.acc_y = ((*CAN_ramka)[5][3] << 8) + (*CAN_ramka)[5][2];
	nx_val->imu_data.acc_z = ((*CAN_ramka)[5][5] << 8) + (*CAN_ramka)[5][4];
	
	
	nx_val->pdm_val.status_field = (*CAN_ramka)[6][0];
	nx_val->pdm_val.temps[0] = (*CAN_ramka)[6][1];
	nx_val->pdm_val.temps[1] = (*CAN_ramka)[6][2];
	nx_val->pdm_val.temps[2] = (*CAN_ramka)[6][3];
	nx_val->pdm_val.temps[3] = (*CAN_ramka)[6][4];
	nx_val->pdm_val.temps[4] = (*CAN_ramka)[6][5];
	nx_val->pdm_val.imd_resistance = ((*CAN_ramka)[6][7] << 8) + (*CAN_ramka)[6][6];

//nx_val->rpm++;
	
}

void AddToBuffor_P(char* buf_nxt, nextion_uart_t* nx_val, volatile bool* do_wysyl)
{
	char value_c[16];

	sprintf(value_c, "n1.val=%d%c%c%c", nx_val->speed, 0xff, 0xff, 0xff);
  strcpy(buf_nxt, value_c);
	
	sprintf(value_c, "n2.val=%d%c%c%c", nx_val->rpm, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	sprintf(value_c, "n4.val=%d%c%c%c", nx_val->engine_temp, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);

  sprintf(value_c, "n5.val=%d%c%c%c", nx_val->bat_voltage, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);

  sprintf(value_c, "j0.val=%d%c%c%c", nx_val->bat_percent, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	sprintf(value_c, "j1.val=%d%c%c%c", nx_val->DC_curr, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	if(nx_val->contactor && (nx_val->can_count < 3) && (flaga_led == false)) {
		HAL_GPIO_WritePin(LED_Pin_GPIO_Port, LED_Pin_Pin, 1);
		flaga_led = true;
	}
	else if(nx_val->can_count >= 3){
		HAL_GPIO_WritePin(LED_Pin_GPIO_Port, LED_Pin_Pin, 0);
		flaga_led  = false;
	}
	
	nx_val->can_count++;
	*do_wysyl = true;
}

void AddToBuffor_R(char* buf_nxt, nextion_uart_t* nx_val, volatile bool* do_wysyl, uint8_t (*CAN_ramka)[CAN_FRAME_COUNT][8])
{
	char value_c[16];
	int i;
	static int t=0;
	
	//n2.val=
	sprintf(value_c, "n2.val=%d%c%c%c", nx_val->rpm, 0xff, 0xff, 0xff);
  strcpy(buf_nxt, value_c);
	
//	//n3.val=
//	sprintf(value_c, "n3.val=%d%c%c%c", nx_val->power, 0xff, 0xff, 0xff);
//  strcat(buf_nxt, value_c);
	
	//n4.val=
	sprintf(value_c, "n4.val=%d%c%c%c", nx_val->engine_temp, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
	//n6.val=
	sprintf(value_c, "n6.val=%d%c%c%c", nx_val->bat_voltage, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
//	//n7.val=
//	sprintf(value_c, "n7.val=%d%c%c%c", nx_val->amps, 0xff, 0xff, 0xff);
//  strcat(buf_nxt, value_c);
	
	//n8.val=
	sprintf(value_c, "n8.val=%d%c%c%c", nx_val->controller_temp, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);

//	//n9.val=
	sprintf(value_c, "n9.val=%d%c%c%c", nx_val->pdm_val.temps[0], 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
//	
//	//n10.val=
	sprintf(value_c, "n10.val=%d%c%c%c", nx_val->pdm_val.temps[1], 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
//	
//	//n11.val=
	sprintf(value_c, "n11.val=%d%c%c%c", nx_val->pdm_val.temps[2], 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
//	
//	//n12.val=
	sprintf(value_c, "n12.val=%d%c%c%c", nx_val->pdm_val.temps[3], 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);
	
//	//n14.val=
	sprintf(value_c, "n14.val=%d%c%c%c", nx_val->susp_rear.avg, 0xff, 0xff, 0xff);
  strcat(buf_nxt, value_c);

	if(t == 30){
		for(i=0; i<8; i++){
			(*CAN_ramka)[0][i] = 0;
			(*CAN_ramka)[2][i] = 0;
			(*CAN_ramka)[3][i] = 0;
			(*CAN_ramka)[5][i] = 0;
		}
		nx_val->bat_voltage = 0;
		nx_val->engine_temp = 0;
		nx_val->controller_temp = 0;
		t=0;
	}
	t++;
	
	
	*do_wysyl = true;
}

void AddToBuffor_SD(char* buf_sd, nextion_uart_t* nx_val, volatile bool* do_wysyl)
{
	char buf[130];
	static int n=0;
	//dodaj parametry zapisywane na SD	
	//------------1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19 20 21 22 23
	sprintf(buf, "%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d\r\n",
	/*			1 				 	2 								3									  	 4												5					*/ 			
	nx_val->speed, nx_val->rpm, nx_val->engine_temp, nx_val->controller_temp, nx_val->bat_voltage,
	/*			6 				   			          	7 										8				        */
	nx_val->susp_front.min, nx_val->susp_front.avg, nx_val->susp_front.max,
	/*			9 				   				     10 								    11	  		 		     */
	nx_val->susp_rear.min, nx_val->susp_rear.avg, nx_val->susp_rear.max,
	/*			12 				   				     13 									     14	  		 		     */
	nx_val->pdm_val.temps[0], nx_val->pdm_val.temps[1], nx_val->pdm_val.temps[2],
	/*			15 				   				     16									     17	  		 		     */
	nx_val->imu_data.yaw, nx_val->imu_data.pitch, nx_val->imu_data.roll,
	/*			18 				   				     19 									     20	  		 		     */
	nx_val->imu_data.acc_x, nx_val->imu_data.acc_y, nx_val->imu_data.acc_z,
	/*			21 				   				     22 									     23	  		 		     */
	nx_val->DC_curr, 			    	nx_val->id_curr, 					nx_val->iq_curr);
	
	strcat(buf_sd, buf);
	n++;
	
	
	if(n == 28){
		*do_wysyl = true;
		n=0;
	}
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
			//HAL_UART_Transmit_DMA(&huart2, "NX_send\r\n", 9);
			*free = false;
			*do_wysyl = false;
		}
	
}

void SDZapis(sd_card_t* sd_card, char* buf_zap, char* buf_usun, volatile bool* do_wysyl)
{
	if(true == sd_card->init){
		SDInit(sd_card);	
		sd_card->init = false;
	}
	
	if(true == *do_wysyl){
		memset(buf_usun, 0 , 2);
		
		f_lseek(&(sd_card->myFile), f_size(&(sd_card->myFile)));
		f_write(&(sd_card->myFile), buf_zap, strlen(buf_zap), &(sd_card->myBytes));
		f_sync(&(sd_card->myFile)); //zapis fat zapobiega utracie danych
				
		sd_card->flaga ^= 1;
		
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
				//HAL_TIM_Base_Stop_IT(&htim3);
				//__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
				*dash_page = PAGE0;
				if( *dash_state == NEXTION_SD ){
					LED0_On();
				}
				//HAL_TIM_Base_Start_IT(&htim1);
				break;
			case 0x01:
				//HAL_TIM_Base_Stop_IT(&htim1);
				//__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
				//HAL_TIM_Base_Start_IT(&htim3);
				*dash_page = PAGE1;
				break;
			case 0x02:
				//HAL_TIM_Base_Stop_IT(&htim1);
				//HAL_TIM_Base_Stop_IT(&htim3);
				*dash_page = PAGE2;
				break;
			case 0x03:
				//HAL_TIM_Base_Stop_IT(&htim3);
				//__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
				*dash_page = PAGE3;
				if( *dash_state == NEXTION_SD ){
					LED0_On();
				}
				//HAL_TIM_Base_Start_IT(&htim1);
				break;
			default:
				//*dash_page = PAGE0;
				break;
		}
	}
}


void SDInit(sd_card_t* sd_card)
{
  char nowa_nazwa[7] = "/0";
  static int nr_pliku = 1;
	if(f_mount(&(sd_card->myFatFS), SDPath, 1) == FR_OK){
		//wyslanie pierwszej ikonki na LCD
		HAL_UART_Transmit(&huart2, "vis p1,1", 8, 150);
		HAL_UART_Transmit(&huart2, (uint8_t*)k, 3, 100);
		
		//sprawdzenie obecnosci plikow
		while(f_stat(sd_card->SD_nazwapliku, NULL) != FR_NO_FILE) {
			nr_pliku++;
			sprintf(nowa_nazwa, "%d.TXT\0", nr_pliku);
			strcpy(sd_card->SD_nazwapliku, nowa_nazwa);
		}
		
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
	
	#ifdef _DEBUG
		 HAL_UART_Transmit(&huart2, "SD_Init\r\n", 9, 200);
	#endif
}

void SDkoniec(sd_card_t* sd_card)
{
	f_close(&(sd_card->myFile));
	//usuniecie ikonek z LCD
	HAL_UART_Transmit(&huart2, "vis p1,0", 8, 150);
	HAL_UART_Transmit(&huart2, (uint8_t*)k, 3, 100);
	HAL_UART_Transmit(&huart2, "vis p2,0", 8, 150);
	HAL_UART_Transmit(&huart2, (uint8_t*)k, 3, 100);

	f_mount(0, SDPath, 0);
	
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








