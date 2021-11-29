#include "main.h"
#include "motor_control.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "i2c_lcd.h"

#define TRIG_PORT GPIOC
#define TRIG_PIN  GPIO_PIN_7
#define TRIG_PORT1 GPIOB
#define TRIG_PIN1  GPIO_PIN_5

void make_trigger1(void);
void ultrasonic_processing1();
void make_trigger(void);
void ultrasonic_processing();
void distance_control();

extern volatile uint16_t TIM11_servo_motor_counter;
extern volatile uint16_t TIM10_motor_con_counter ;
extern volatile uint16_t TIM11_10ms_ultrasonic_counter;
extern volatile uint16_t TIM10_10ms_ultrasonic_counter1;
extern volatile uint16_t TIM10_delay_counter;


uint32_t distance=0;  // 거리
uint8_t ic_cpt_flag=0;   // rising edge/falling edge를 detect하는 flag;
uint32_t distance1=0;  // 거리 uint32_t
uint8_t ic_cpt_flag1=0;
// rising edge/falling edge INT가 발생되면 이곳으로 집입
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t is_first_captured = 0;
	static uint8_t is_first_captured1= 0;
	if (htim->Instance == TIM1)
	{
		if ( is_first_captured == 0 )   // risring edge detect !!!
		{
		    __HAL_TIM_SET_COUNTER(htim,0);
	    	is_first_captured=1;   // risring edge detect flag set
		}
		else if (is_first_captured == 1)  // falling edge INT detect
		{
			is_first_captured=0;
			distance = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			ic_cpt_flag=1;
		}
	}
	else if (htim->Instance == TIM2)
		 {
			 printf("TIM ULTRA\n");
			if ( is_first_captured1 == 0 )   // risring edge detect !!!
			{
				__HAL_TIM_SET_COUNTER(htim,0);
				is_first_captured1 = 1;   // risring edge detect flag set
			}
			else if (is_first_captured1 == 1)  // falling edge INT detect
			{
				is_first_captured1 = 0;
				distance1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
				ic_cpt_flag1=1;
			}
		 }

}




void ultrasonic_processing()
{
	if (TIM11_10ms_ultrasonic_counter >= 100)   // timer 1sec reached
	{
		TIM11_10ms_ultrasonic_counter=0;
		make_trigger();
		if (ic_cpt_flag==1)
		{
			ic_cpt_flag=0;
			distance = distance * 0.034 / 2;  // 1usdp 0.034cm가 이동 하는데 /2는 왕복 값이 오기 떄문데 편도만 필요

			printf("distance : %d\n", distance);
		}
	}
	distance_control();
}
void make_trigger(void)
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
	delay_us(2);
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
}



void ultrasonic_processing1()
{
	//printf("ultra processing!!\n");
	if (TIM10_10ms_ultrasonic_counter1 >= 100)   // timer 1sec reached
	{
		TIM10_10ms_ultrasonic_counter1 = 0;
		make_trigger1();
		if (ic_cpt_flag1==1)
		{
			ic_cpt_flag1=0;
			distance1 = distance1 * 0.034 / 2;  // 1usdp 0.034cm가 이동 하는데 /2는 왕복 값이 오기 떄문데 편도만 필요

			printf("distance1 : %d\n", distance1);
		}
	}
	distance_control1();
}
void make_trigger1(void)
{
	HAL_GPIO_WritePin(TRIG_PORT1, TRIG_PIN1, GPIO_PIN_RESET);
	delay_us(2);
	HAL_GPIO_WritePin(TRIG_PORT1, TRIG_PIN1, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(TRIG_PORT1, TRIG_PIN1, GPIO_PIN_RESET);
}
void distance_control()
{
	if(distance <=25)
	{
		move_cursor(1,0);
		lcd_string("              ");
		move_cursor(1,0);
		lcd_string(" !! Warning !! ");
		motor_stop();
		TIM10_motor_con_counter = 0;
		while(TIM10_motor_con_counter <= 300);
		motor_forward(2000);
		TIM10_motor_con_counter = 0;
		while(TIM10_motor_con_counter <= 200);
		TIM10_motor_con_counter = 0;
		while(TIM10_motor_con_counter <= 300);
		motor_left();
		TIM10_motor_con_counter = 0;
		while(TIM10_motor_con_counter <= 75);
		motor_forward(2000);
		move_cursor(1,0);
		lcd_string("                ");
		move_cursor(1,0);
		lcd_string("!!Select mode!!");

	}
}
void distance_control1()
{
	if(distance1 <=25)
		{
		move_cursor(1,0);
		lcd_string("              ");
		move_cursor(1,0);
		lcd_string(" !! Warning !! ");
		motor_stop();
		TIM10_motor_con_counter = 0;
		while(TIM10_motor_con_counter <= 300);
		motor_backward(2000);
		TIM10_motor_con_counter = 0;
		while(TIM10_motor_con_counter <= 200);
		TIM10_motor_con_counter = 0;
		while(TIM10_motor_con_counter <= 300);
		motor_right();
		TIM10_motor_con_counter = 0;
		while(TIM10_motor_con_counter <= 75);
		motor_backward(2000);
		move_cursor(1,0);
		lcd_string("                ");
		move_cursor(1,0);
		lcd_string("!!Select mode!!");
		}
}
