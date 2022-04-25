
#include "main.h"

#ifndef INC_STEP_MOTOR_H_
#define INC_STEP_MOTOR_H_

#endif /* INC_STEP_MOTOR_H_ */

#define DC_RIGHT 1
#define DC_LEFT 0
#define DC_STOP 3

class DC_Motor
{

	GPIO_TypeDef    *EN_GPIO_Port;
		uint16_t    	EN_Pin;
	TIM_HandleTypeDef htim;

	void stop();

 public:
  void init( GPIO_TypeDef    *EN_GPIO_Port,
	uint16_t    	EN_Pin,
TIM_HandleTypeDef htim);

  void run(float velocity);


};
