#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include "stm32f1xx_hal.h"

#define MAX_RPM 400.0f
#define MIN_RPM 1.0f
#define TIMER_FREQ 1000000
#define NUMS_OF_MOTOR 2
#define STEP_MODE 200

typedef enum{
	FORWARD, // 0
	BACKWARD // 1
}Direct_State;

typedef enum{
	ACTIVE,
	INACTIVE
}Motor_State;

typedef struct {
    TIM_HandleTypeDef *htim;  
    uint32_t Channel;  
    GPIO_TypeDef *DIR_Port;
    uint16_t DIR_Pin;
    GPIO_TypeDef *ENA_Port;
    uint16_t ENA_Pin;
	uint16_t steps_per_round; 
	volatile uint32_t step_count;
	volatile uint32_t last_step_count;
	volatile uint32_t target_steps;
	volatile Motor_State state;
	float speed_rpm;
	Direct_State direction;
	uint32_t time_run;
	uint32_t start_time;
	uint32_t time_count;
} StepperMotor;


void Stepper_Init(StepperMotor *motor, TIM_HandleTypeDef *htim, uint32_t Channel, 
                  GPIO_TypeDef *DIR_Port, uint16_t DIR_Pin, 
                  GPIO_TypeDef *ENA_Port, uint16_t ENA_Pin);


void Stepper_Enable(StepperMotor *motor);
void Stepper_Disable(StepperMotor *motor);
void Stepper_SetDirection(StepperMotor *motor);
void Stepper_SetSpeedRPM(StepperMotor *motor);
void Stepper_MoveSteps(StepperMotor *motor);
void Stepper_Rotate(StepperMotor* motor);

#endif
