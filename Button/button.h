#ifndef _BUTTON_H_
#define _BUTTON_H_

#include "main.h"

#define BUTTON_NUM 3

typedef enum{
	BUTTON_NONE,
	BUTTON_START_HOLD,
	BUTTON_STOP_HOLD,
	BUTTON_ENCODER_HOLD,
	BUTTON_START_PRESS,
	BUTTON_STOP_PRESS,
	BUTTON_ENCODER_PRESS
}Button_State;

void Button_Init(void);
Button_State Button_Pressing(void);

#endif
