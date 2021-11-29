//#include "main.h"
//#include "motor_control.h"
//#define ir_sensor_Pin GPIO_PIN_10
//#define ir_sensor_GPIO_Port GPIOC
//
//#define NOTDETECTION     1
//#define DETECTION  0   // S/W 눌렸을때  ACTIVE LOW
//extern volatile int TIM10_motor_con_counter;
//
//void ir_sensor_control()
//{
//	uint8_t State;
//	State =HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10);
//	if ( State == DETECTION )
//	{
//		while(TIM10_motor_con_counter <= 100);
//	    motor_stop();
//	}
//}
