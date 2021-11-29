#if 0
#include "main.h"


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "rfid_main.h"
#include "mfrc522.h"    // for RFID

uint8_t rfid_check_flag=0;  // 현재 rfid를 checking중인지
uint8_t buzzer_start_flag=0; // 부저 start/stop indicator flag
uint8_t door_open_flag=0;    // 서보 모터 가동

#define CARD_SU  5
extern SPI_HandleTypeDef hspi1;   // for RFID interface
extern TIM_HandleTypeDef htim3;  // for servo motor interface
extern volatile int TIM10_delay_counter;
uint8_t readData;   // 1 byte save variable
uint8_t rxDataStr[MAX_LEN];   // rfid tagging data
uint8_t cardNotFound=0;       // rfid card found ?
uint8_t regCardKey[CARD_SU][5] =
{ {0x55, 0xfa, 0x9a, 0x5c, 0x69},
		{0x93, 0x20, 0x20, 0x20, 0x20},
		// 93 20 20 20 20
};
char sbuff[40];
uint8_t regCardKey_copy[5][5] =    //현재는 하드코딩이지만 자동으로 등록하고/ 삭제 할수 있도록 삭제
{
	{0,0, 0, 0, 0},
	{0,0, 0, 0, 0},
	{0,0, 0, 0, 0},
	{0,0, 0, 0, 0},
	{0,0, 0, 0, 0},
	//{0},

};
extern volatile int TIM11_servo_motor_counter;

// RFID READER 초기화 함수
void rfid_delete_()
{
	printf("card delete start\n");
	strncpy(regCardKey,regCardKey_copy,5);
	for (int i=0; i < CARD_SU; i++)
	{
		printf("%02x ", rxDataStr[i]);
	}

}
void rfid_reader_init(void)
{

	mfrc522_init(&hspi1);
	HAL_Delay(2000);

	readData = mfrc522_read(VersionReg);
	if (readData == 0x92)
	{
		printf("MIFARE RC522v2\n");
		printf("Detected !!!\n");
	}
	else if (readData == 0x91 || readData == 0x90)
	{
		printf("MIFARE RC522v1\n");
		printf("Detected !!!\n");
	}
	else
	{
		printf("No RFID Reader found !!!\n");
	}

}

// RFID tagging 처리 함수
//void rfid_tag_processing(void)
//{
//	if (T300ms_counter >= 300)  // polling 300ms rfid reader
//	{
//		T300ms_counter=0;
//		if (!rfid_check_flag)   // 현재 RFID를 checking중이 아니면
//		{
//			readData = mfrc522_request(PICC_REQALL, rxDataStr);
//			// RFID contack check
//			if (readData == CARD_FOUND)
//			{
//				buzzer_start_flag=1;  // 부저 시작 flag를 set
//				Tms_Buzzer_counter=0;  // 부저 timer counter를 clear
//				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, SET);
//
//				for (int i=0; i < MAX_LEN; i++)
//					rxDataStr[i]=' ';
//				readData = mfrc522_get_card_serial(rxDataStr);
//
//				printf("rfid:");
//				for (int i=0; i < 5; i++)
//				{
//					printf("%02x ", rxDataStr[i]);
//				}
//				printf("\n");
//				// 예) rfid:30 49 90 c2 22\n 으로 PC로 보내진다.
//				rfid_check_flag=1;  // set rfid check flag
//				Trfid_check_timer_counter=0;  // wait 10sec
//				// PC로부터 Valid나 Invalid 응답이 올때까지 time 10초 동안
//			}
//		}
//	}
//
//	if (rfid_check_flag)   // rfid를 checking중인가 ?
//	{
//		if (Trfid_check_timer_counter >= 10000)  // 10초
//		{
//			// rfid:30 49 90 c2 22\n 메세지를 PC로 보낸지 10초가 경과 되었나
//			// 10초가 지난 뒤에도 PC로부터 Valid나 Invalid 응답이 오지 않는경우
//			Trfid_check_timer_counter=0;
//			rfid_check_flag=0;   // rfid_check_flag를 해제를 해서 다음
//			                     // rfid를 처리 가능 하도록 한다.
//			printf("RFID check timeout occurs!!!\n");
//		}
//	}
//
//	if (buzzer_start_flag)   // if buzzer set status
//	{
//		 if (Tms_Buzzer_counter >= 300)  // if buzzer set status and time reach 300ms
//		 {
//			 HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, RESET); // buzzer off
//			 buzzer_start_flag=Tms_Buzzer_counter=0;
//		 }
//	}
//
//	// 문을 열고 2초가 경과 되었는지를 체크 하여 2초가 지났으면 문을 닫는다.
//	if (door_open_flag && T1ms_PWM_counter >= 2000)
//	{
//		door_open_flag=T1ms_PWM_counter=0;
//		__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 50);  //1ms :180degree
//	}
//}
//------- start RFID DEMO ----------------
#if 1
void rfid_control_check()
{

	for(int i = 0; i <CARD_SU; i++)
	{
		sprintf(sbuff,"%02x %02x %02x %02x %02x    ", regCardKey[i][0],regCardKey[i][1],regCardKey[i][2],regCardKey[i][3],regCardKey[i][4],regCardKey[i][5]);
		break;
	}
	move_cursor(0,0);
	lcd_string("Registered CARD ");
	move_cursor(1,0);
	lcd_string(sbuff);
	}
void rfid_control_register()
{
	move_cursor(1,0);
	lcd_string("                    ");
	move_cursor(0,0);
	lcd_string("!!Tag CARD plz!!");
	readData = mfrc522_request(PICC_REQALL, rxDataStr);
	// RFID contack check
	printf("card_resister start\n");


	if (readData == CARD_FOUND)
	{
		printf("cad_found\n");
		for (int i=0; i < MAX_LEN; i++)
			rxDataStr[i]=' ';
		readData = mfrc522_get_card_serial(rxDataStr);
		for (int i=0; i < CARD_SU; i++)
		{
			strncpy(regCardKey[i],rxDataStr,5);
			sprintf	(sbuff,"%02x %02x %02x %02x %02x    ", regCardKey[i][0],regCardKey[i][1],regCardKey[i][2],regCardKey[i][3],regCardKey[i][4],regCardKey[i][5]);
			break;
		}
		move_cursor(0,0);
		lcd_string("!!Register CARD!!");
		move_cursor(1,0);
		lcd_string(sbuff);
   }
}
void rfid_control_register_delay()
{

	TIM11_servo_motor_counter = 0;
	while(TIM11_servo_motor_counter <= 7000)
	{
		if(TIM11_servo_motor_counter % 50 == 0) rfid_control_register();
	}


}


//	void card_resister()
//	{
//		byte = mfrc522_request(PICC_REQALL, str); // 카드의 tagging 유무 확인
//
//		if(byte == CARD_FOUND)
//		{
//			byte = mfrc522_get_card_serial(str);
//
//			for (int i = 0; i < 5; i++)
//			{
//			strncpy(card_record_table[card_list],str,5);
//			printf("%02x ",str[i]);      //0x20 0x40 0x20 0x10 0x50
//
//			}
//			for(int i = 0; i < 10; i++)
//			{
//				sprintf(sbuff,"%02x %02x %02x %02x %02x    ", card_record_table[i][0],card_record_table[i][1],card_record_table[i][2],card_record_table[i][3],card_record_table[i][4],card_record_table[i][5]);
//				I2C_LCD_write_string_XY(1,0, sbuff);
//
//				break;
//			}
//
//		}
//	}

void rfid_control_main(void)
{
	//rfid_reader_init();
	if(TIM11_servo_motor_counter >= 1000)
	{
		TIM11_servo_motor_counter = 0;
		readData = mfrc522_request(PICC_REQALL, rxDataStr);
		// RFID contack check

		if (readData == CARD_FOUND)
		{
			printf("cad_found\n");
			for (int i=0; i < MAX_LEN; i++)
				rxDataStr[i]=' ';
			readData = mfrc522_get_card_serial(rxDataStr);
			for (int i=0; i < CARD_SU; i++)
			{
				if (strncmp(rxDataStr,regCardKey[i],5) == 0)
				{
					for (int i=0; i < 5; i++)
					{
						printf("%02x ", rxDataStr[i]);
					}
					printf("Valid Card !!!\n");
					printf("DOOR Opened !!!\n");
					move_cursor(1,0);
					lcd_string("               ");
					move_cursor(1,0);
					lcd_string("OPEN DOOR!!");
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 100); //2ms 100(2ms)/1000(20ms) :180degree
					HAL_Delay(2000);
					TIM10_delay_counter = 0;
					move_cursor(1,0);
					lcd_string("               ");
					move_cursor(1,0);
					lcd_string("DOOR Close!!");
					HAL_Delay(1000);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 50);  //1ms :180degree
					move_cursor(1,0);
					lcd_string("               ");
					move_cursor(1,0);
					lcd_string("!!Select Mode!!");
					printf("DOOR Closed !!!\n");
					cardNotFound=0;
					break;
				}
				cardNotFound=1;
			}
			if (cardNotFound)
			{
				for (int i=0; i < 5; i++)
				{
					printf("%02x ", rxDataStr[i]);
				}
				printf("Invalid Card !!!\n");
			}
		}
		//	HAL_Delay(1000);
	}
  }



#endif
#endif
//-------- end RFID DEMO -----------------
