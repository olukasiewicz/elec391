#include "motor.h"
#include "piano_robot_config.h"
#include "main.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#define MOTOR_PWM_MAX 100
#define MOTOR_PWM_TIM htim3
#define MOTOR_PWM_CHANNEL 1

#define POS_DEADBAND_COUNTS 10   

static float motor_target = 0.0f;

void set_pwm_duty(float duty_cycle)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t ccr_value = (uint32_t)((duty_cycle/100.0f) * arr);
    __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, TIM_CHANNEL_3, ccr_value);
    __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, TIM_CHANNEL_4, ccr_value);
}

/* ------------------------------------------------------------------ */
void Motor_Init(void)
{
    // /* Direction pins default to STOP */
    HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_GPIO_Port, M2_Pin, GPIO_PIN_RESET);
 
    /* Start PWM */
    HAL_TIM_PWM_Start(&MOTOR_PWM_TIM, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&MOTOR_PWM_TIM, TIM_CHANNEL_4);
    set_pwm_duty(0);
}

/* ------------------------------------------------------------------ */
void Motor_Update(float PID_output, float PID_error)
{
    float abs_out = fabsf(PID_output);

    if (fabsf(PID_error) <= (float)POS_DEADBAND_COUNTS) {
        Motor_Brake();
    } else if (PID_output > 0.0f){
        Motor_Drive(abs_out, MOTOR_DIR_FORWARD);
    } else {
        Motor_Drive(abs_out, MOTOR_DIR_REVERSE);
    }
}

/* ------------------------------------------------------------------ */
void Motor_Drive(float duty_cycle, Motor_Dir_t dir)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t ccr_value = (uint32_t)((duty_cycle/100.0f) * arr);
    switch (dir) 
    {
        case MOTOR_DIR_FORWARD:
            __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, TIM_CHANNEL_3, ccr_value);
            __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, TIM_CHANNEL_4, 0);
            break;
 
        case MOTOR_DIR_REVERSE:
            __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, TIM_CHANNEL_4, ccr_value);
            break;
 
        case MOTOR_DIR_BRAKE:
            __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, TIM_CHANNEL_3, arr);
            __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, TIM_CHANNEL_4, arr);
            break;
 
        default:
        case MOTOR_DIR_STOP:
            __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, TIM_CHANNEL_4, 0);
            break;
    }
}

/* ------------------------------------------------------------------ */
void Motor_Coast(void)
{
    set_pwm_duty(0);
    HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_GPIO_Port, M2_Pin, GPIO_PIN_RESET);
}
 
/* ------------------------------------------------------------------ */
void Motor_Brake(void)
{
    set_pwm_duty(MOTOR_PWM_MAX);
    HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M2_GPIO_Port, M2_Pin, GPIO_PIN_SET);
}

/* ------------------------------------------------------------------ */

void Motor_SetTarget(float target)
{
    motor_target = target;
}

float Motor_GetTarget(void)
{
    return motor_target;
}

bool Motor_AtTarget(const Encoder_t *enc)
{
    return abs(enc->position - motor_target) < POS_DEADBAND_COUNTS;
}