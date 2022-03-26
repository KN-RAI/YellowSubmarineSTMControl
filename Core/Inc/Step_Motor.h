
#include "main.h"

#ifndef INC_STEP_MOTOR_H_
#define INC_STEP_MOTOR_H_

#endif /* INC_STEP_MOTOR_H_ */

#define SM_RIGHT 1
#define SM_LEFT 0

class Step_Motor
{

	GPIO_TypeDef    *STEP_GPIO_Port;
	uint16_t    	STEP_Pin;
	GPIO_TypeDef	*DIR_GPIO_Port;
	uint16_t        DIR_Pin;
	GPIO_TypeDef    *INIT_SEQ_GPIO_Port;
	uint16_t 		INIT_SEQ_Pin;

	int 			position;
	bool 			is_turned_on;
	const int		final_position=5000;
	int 			direction;


 public:
  void init(GPIO_TypeDef    *STEP_GPIO_Port,
	uint16_t    	STEP_Pin,
	GPIO_TypeDef *DIR_GPIO_Port,
	uint16_t        DIR_Pin,
	GPIO_TypeDef    *INIT_SEQ_GPIO_Port,
	uint16_t 		INIT_SEQ_Pin);
  void stop();
  void turn_on(int direction);
  void run();


};
