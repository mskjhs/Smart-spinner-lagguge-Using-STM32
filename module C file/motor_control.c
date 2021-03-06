#if 1
#include "main.h"
#include "motor_control.h"
#include "i2c_lcd.h"
extern TIM_HandleTypeDef htim4;
extern volatile int TIM10_motor_con_counter;

void motor_left_turn()
{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,4000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,4000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}
void motor_right_turn()
{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,4000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,4000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}
void motor_backward(int i)
{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,i);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,i);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}
void motor_forward(int i1)
{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,i1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,i1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}
void motor_stop()
{
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}
void motor_left()
{

	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,4000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,4000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);



}
void motor_right()
{
	motor_right_turn();

}

#endif
