#include "step_motor.h"

static StepperMotor* motors[NUMS_OF_MOTOR] = {NULL};


void Stepper_Init(StepperMotor* motor, TIM_HandleTypeDef* htim, uint32_t channel, GPIO_TypeDef* dir_port, uint16_t dir_pin, GPIO_TypeDef* en_port, uint16_t en_pin){
	motor->htim = htim;
	motor->Channel = channel;
	motor->DIR_Port = dir_port;
	motor->DIR_Pin = dir_pin;
	motor->ENA_Port = en_port;
	motor->ENA_Pin = en_pin;
	motor->step_count = 0;
	motor->last_step_count = 0;
	motor->target_steps = 0;
	motor->state = INACTIVE;
	motor->speed_rpm = 0;
	motor->steps_per_round = STEP_MODE;
	motor->direction = FORWARD;
	motor->time_run = 0;
	motor->start_time = 0;
	motor->time_count = 0;
	for(uint8_t i=0; i<NUMS_OF_MOTOR; ++i){
		if(motors[i] == motor) return;
		if(motors[i] == NULL){
			motors[i] = motor;
			return;
		}
	}
}

void Stepper_Enable(StepperMotor *motor) {
	HAL_TIM_PWM_Start_IT(motor->htim, motor->Channel);
    HAL_GPIO_WritePin(motor->ENA_Port, motor->ENA_Pin, GPIO_PIN_RESET);
	motor->state = ACTIVE;
}

void Stepper_Disable(StepperMotor *motor) {
    HAL_GPIO_WritePin(motor->ENA_Port, motor->ENA_Pin, GPIO_PIN_SET);
	motor->state = INACTIVE;
	HAL_TIM_PWM_Stop_IT(motor->htim, motor->Channel);
}

void Stepper_SetDirection(StepperMotor *motor) {
    HAL_GPIO_WritePin(motor->DIR_Port, motor->DIR_Pin, (GPIO_PinState)motor->direction);
}

void Stepper_SetSpeedRPM(StepperMotor *motor)
{
    if (motor->speed_rpm < MIN_RPM) motor->speed_rpm = MIN_RPM;
	if(motor->speed_rpm > MAX_RPM) motor->speed_rpm = MAX_RPM;
    float step_freq = (motor->speed_rpm * motor->steps_per_round) / 60.0f;
    if (step_freq < 0.5f)
        step_freq = 0.5f;
    uint32_t timer_clk = SystemCoreClock;
    uint32_t total_count = timer_clk / step_freq;
    uint32_t psc;
    uint32_t arr;
    if (total_count <= 65535){
        psc = 0;
        arr = total_count - 1;
    }else{
        psc = total_count / 65536;
        arr = total_count / (psc + 1);
        if (arr > 65535)
            arr = 65535;
        if (arr < 10)
            arr = 10;
    }
    __HAL_TIM_DISABLE(motor->htim);

    motor->htim->Instance->PSC = psc;
    motor->htim->Instance->ARR = arr;
    __HAL_TIM_SET_COMPARE(motor->htim, motor->Channel, arr / 2);

    __HAL_TIM_ENABLE(motor->htim);
}


void Stepper_MoveSteps(StepperMotor *motor) {
//    motor->step_count = 0;
	Stepper_SetDirection(motor);
	Stepper_SetSpeedRPM(motor);
	if (motor->state == INACTIVE) Stepper_Enable(motor);
}

void Stepper_Rotate(StepperMotor* motor){
	Stepper_SetSpeedRPM(motor);
	Stepper_SetDirection(motor);
	Stepper_Enable(motor);
	motor->start_time = HAL_GetTick();
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim){
	for(uint8_t i=0; i < NUMS_OF_MOTOR; ++i){
		if(motors[i] == NULL) continue;
		if(motors[i]->htim == htim && motors[i]->state == ACTIVE){
			motors[i]->step_count++;
			motors[i]->last_step_count = motors[i]->step_count;
			if(motors[i]->time_run > 0){
				motors[i]->time_count = HAL_GetTick() - motors[i]->start_time;
				if(motors[i]->time_count / 1000 >= motors[i]->time_run){
					Stepper_Disable(motors[i]);
					motors[i]->time_run = 0;
					motors[i]->time_count = 0;
					motors[i]->step_count = 0;
				}
			}else{
				if(motors[i]->step_count >= motors[i]->target_steps){
					Stepper_Disable(motors[i]);
					motors[i]->target_steps = 0;
					motors[i]->step_count = 0;
				}
			}
		}
	}
}

