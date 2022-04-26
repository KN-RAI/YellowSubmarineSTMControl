#include <DC_Motor.h>
#include"main.h"



void DC_Motor::run(int direction, float velocity)
{
	if(!direction) stop();
	else
	{
		HAL_GPIO_WritePin(this->EN_GPIO_Port, this->EN_Pin,GPIO_PIN_SET);

		__HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1, velocity); // nie da sie przekazac makro do funkcji :<
	}

}


void DC_Motor::stop()
{
	 HAL_GPIO_WritePin(this->EN_GPIO_Port, this->EN_Pin,GPIO_PIN_RESET); // idk czy to dziala

}


void DC_Motor::init(GPIO_TypeDef *EN_GPIO_Port,
	uint16_t EN_Pin,
	TIM_HandleTypeDef *htim)
{
	this->htim=htim;
	this->EN_GPIO_Port=EN_GPIO_Port;
	this->EN_Pin=EN_Pin;

}

