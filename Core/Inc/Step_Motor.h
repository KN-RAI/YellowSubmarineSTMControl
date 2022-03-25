
#include "main.h"

#ifndef INC_STEP_MOTOR_H_
#define INC_STEP_MOTOR_H_

#endif /* INC_STEP_MOTOR_H_ */

#define SM_UP 1
#define SM_DOWN 0

class Step_Motor
{

	GPIO_TypeDef    *STEP_GPIO_Port;
	uint16_t    	STEP_Pin;
	GPIO_TypeDef	*DIR_GPIO_Port;
	uint16_t        DIR_Pin;
	GPIO_TypeDef    *INIT_SEQ_GPIO_Port;
	uint16_t 		INIT_SEQ_Pin;

	int 			position;
	bool 			is_running;
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
  void  start(int direction);
  void run();


};
