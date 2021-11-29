#include "main.h"

void servo_motor_control_main();

int arm_roates_indicator=0;

extern TIM_HandleTypeDef htim3;
extern volatile int TIM11_servo_motor_counter;
// 840000000HZ / 1600 => 50,000HZ
// T=1/f 1/50000 ->1펄스가 잡아먹는 시간 0.00002
//2ms : 0.00002 *100 = 0.002 sec(2ms)
//2ms pwm to sevo motor --> arm rotate
//100번 (2ms)
//1ms(50) : 0도
//1.5ms(75번):90도
//2ms(100번):180도



void servo_motor_control_main()
{

#if 1
	if(TIM11_servo_motor_counter >= 200)	//1000ms
	{
		//printf("servo_motor_start\n");
		TIM11_servo_motor_counter = 0;
		if (!arm_roates_indicator)	//if (arm_roates_indicator == 0) 이라는뜻
		{
			//180도
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,100);
		}
		else
		{
			//0도
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,50);
		}
		arm_roates_indicator++;	// next arm_roates_indicator 로 간다
		arm_roates_indicator %= 2;
	}
}

#else	//demo version
	while(1)
	{

		//180도
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,100);
		HAL_Delay(1000);

		//0도
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,50);
		HAL_Delay(1000);
	}
}

#endif
