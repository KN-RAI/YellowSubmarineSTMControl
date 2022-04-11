#include <Step_Motor.h>
#include"main.h"


 void Step_Motor::turn_on(int direction)
{
	HAL_GPIO_WritePin(this->DIR_GPIO_Port, this->DIR_Pin, (GPIO_PinState)direction);
	this->direction=direction;
	this->is_turned_on=true;
}

void Step_Motor::run()
{
	if(is_turned_on){

	if(this->position<=this->final_position && this->position>=0)
	{
		HAL_GPIO_WritePin(this->STEP_GPIO_Port, this->STEP_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(this->STEP_GPIO_Port, this->STEP_Pin, GPIO_PIN_SET);
		HAL_Delay(1);
		this->position+=(this->direction)?-1:1;
	}

	}
}


void Step_Motor::stop()
{
	HAL_GPIO_WritePin(this->STEP_GPIO_Port, this->STEP_Pin, GPIO_PIN_RESET);
	this->is_turned_on=false;
}


void Step_Motor::init(GPIO_TypeDef* STEP_GPIO_Port,	uint16_t STEP_Pin, GPIO_TypeDef* DIR_GPIO_Port,uint16_t DIR_Pin,GPIO_TypeDef* INIT_SEQ_GPIO_Port,uint16_t INIT_SEQ_Pin)
{
	this->STEP_GPIO_Port=STEP_GPIO_Port;
	this->STEP_Pin=STEP_Pin;
	this->DIR_GPIO_Port=DIR_GPIO_Port;
	this->DIR_Pin=DIR_Pin;
	this->INIT_SEQ_GPIO_Port=INIT_SEQ_GPIO_Port;
	this->INIT_SEQ_Pin=INIT_SEQ_Pin;

	this->position=0;

	this->turn_on(SM_LEFT);
	//dociera do krancowki na pozycji 0
	while(HAL_GPIO_ReadPin(this->INIT_SEQ_GPIO_Port, this->INIT_SEQ_Pin)==GPIO_PIN_RESET)
	{
		this->run();
	}

	this->turn_on(SM_RIGHT);
	//dociera na koniec do pozycji startowej
	for(int i=0; i<=this->final_position;i++)
	{
		this->run();
	}

	this->stop();
	this->position=this->final_position;
}

