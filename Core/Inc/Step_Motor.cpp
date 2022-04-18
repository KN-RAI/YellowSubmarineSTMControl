#include <Step_Motor.h>
#include"main.h"



void Step_Motor::run(int direction)
{
	if(direction != SM_STOP)
	{
		if(direction== SM_LEFT && position>0)
				position--;
		if(direction== SM_RIGHT && position<final_position)
				position++;

		step(direction);
	}
	else
		stop();
}

void Step_Motor::step(int direction)
{
	HAL_GPIO_WritePin(EN_GPIO_Port,EN_Pin, GPIO_PIN_RESET); // enable pin
	HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, (GPIO_PinState)direction); // set direction
	HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET); // step low
	HAL_Delay(1);
	HAL_GPIO_WritePin(this->STEP_GPIO_Port,STEP_Pin, GPIO_PIN_SET); // step high
}

void Step_Motor::stop()
{
	HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(EN_GPIO_Port,EN_Pin, GPIO_PIN_SET);
}


void Step_Motor::init(GPIO_TypeDef* STEP_GPIO_Port,	uint16_t STEP_Pin,GPIO_TypeDef* EN_GPIO_Port,	uint16_t EN_Pin, GPIO_TypeDef* DIR_GPIO_Port,uint16_t DIR_Pin,GPIO_TypeDef* INIT_SEQ_GPIO_Port,uint16_t INIT_SEQ_Pin)
{
	this->STEP_GPIO_Port=STEP_GPIO_Port;
	this->STEP_Pin=STEP_Pin;
	this->EN_GPIO_Port=EN_GPIO_Port;
	this->EN_Pin=EN_Pin;
	this->DIR_GPIO_Port=DIR_GPIO_Port;
	this->DIR_Pin=DIR_Pin;
	this->INIT_SEQ_GPIO_Port=INIT_SEQ_GPIO_Port;
	this->INIT_SEQ_Pin=INIT_SEQ_Pin;

	//dociera do krancowki na pozycji 0
	while(HAL_GPIO_ReadPin(this->INIT_SEQ_GPIO_Port, this->INIT_SEQ_Pin)==GPIO_PIN_RESET)
	{
		step(SM_LEFT);
	}

	//dociera na koniec do pozycji startowej
	for(int i=0; i<=final_position;i++)
	{
		step(SM_RIGHT);
	}

	stop();
	position=final_position;
}

