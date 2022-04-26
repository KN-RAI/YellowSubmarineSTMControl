
#include "main.h"

#ifndef INC_STEP_MOTOR_H_
#define INC_STEP_MOTOR_H_

#endif /* INC_STEP_MOTOR_H_ */

#define SM_RIGHT 1
#define SM_LEFT 0
#define SM_STOP 3

class Step_Motor
{

	GPIO_TypeDef    *STEP_GPIO_Port;
	uint16_t    	STEP_Pin;
	GPIO_TypeDef    *EN_GPIO_Port;
		uint16_t    	EN_Pin;
	GPIO_TypeDef	*DIR_GPIO_Port;
	uint16_t        DIR_Pin;
	GPIO_TypeDef    *INIT_SEQ_GPIO_Port;
	uint16_t 		INIT_SEQ_Pin;

	int 			position;
	const int		final_position=5000;
	void stop();
	void step(int direction);

 public:
  void init( GPIO_TypeDef    *STEP_GPIO_Port,
	uint16_t    	STEP_Pin,
	GPIO_TypeDef    *EN_GPIO_Port,
		uint16_t    	EN_Pin,
	GPIO_TypeDef *DIR_GPIO_Port,
	uint16_t        DIR_Pin,
	GPIO_TypeDef    *INIT_SEQ_GPIO_Port,
	uint16_t 		INIT_SEQ_Pin);

  void run(int direction);


};
