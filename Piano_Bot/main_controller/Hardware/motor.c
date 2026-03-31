#include "motor.h"
#include "piano_robot_config.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#define MOTOR_PWM_MAX 100
#define MOTOR_PWM_TIM htim1
#define MOTOR_PWM_CHANNEL 1

#define MOTOR_IN1_GPIO_PORT 0
#define MOTOR_IN1_PIN 0
#define MOTOR_IN2_GPIO_PORT 0
#define MOTOR_IN2_PIN 0

#define POS_DEADBAND_COUNTS 10   


static inline void set_pwm_duty(uint32_t duty_counts)
{
    if (duty_counts > MOTOR_PWM_MAX) duty_counts = MOTOR_PWM_MAX;
    __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, MOTOR_PWM_CHANNEL, duty_counts);
}

/* ------------------------------------------------------------------ */
void Motor_Init(void)
{
    /* Direction pins default to STOP */
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
 
    /* Start PWM */
    HAL_TIM_PWM_Start(&MOTOR_PWM_TIM, MOTOR_PWM_CHANNEL);
    set_pwm_duty(0);
}

/* ------------------------------------------------------------------ */
void Motor_Update(float PID_output, float PID_error)
{
    float abs_out = fabsf(PID_output);
    uint8_t speed_pct = (uint8_t)((abs_out / (float)MOTOR_PWM_MAX) * 100.0f);

    if (fabsf(PID_error) <= (float)POS_DEADBAND_COUNTS) {
        Motor_Brake();
    } else if (PID_output > 0.0f){
        Motor_Drive(speed_pct, MOTOR_DIR_FORWARD);
    } else {
        Motor_Drive(speed_pct, MOTOR_DIR_REVERSE);
    }
}

/* ------------------------------------------------------------------ */
void Motor_Drive(uint8_t speed_pct, Motor_Dir_t dir)
{
    uint32_t duty = ((uint32_t)speed_pct * MOTOR_PWM_MAX) / 100u;

    switch (dir) 
    {
        case MOTOR_DIR_FORWARD:
            HAL_GPIO_WritePin(MOTOR_IN1_GPIO_PORT, MOTOR_IN1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MOTOR_IN2_GPIO_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
            set_pwm_duty(duty);
            break;
 
        case MOTOR_DIR_REVERSE:
            HAL_GPIO_WritePin(MOTOR_IN1_GPIO_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_IN2_GPIO_PORT, MOTOR_IN2_PIN, GPIO_PIN_SET);
            set_pwm_duty(duty);
            break;
 
        case MOTOR_DIR_BRAKE:
            Motor_Brake();
            break;
 
        default:
        case MOTOR_DIR_STOP:
            Motor_Coast();
            break;
    }
}

/* ------------------------------------------------------------------ */
void Motor_Coast(void)
{
    set_pwm_duty(0);
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
}
 
/* ------------------------------------------------------------------ */
void Motor_Brake(void)
{
    set_pwm_duty(MOTOR_PWM_MAX);
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_PORT, MOTOR_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_PORT, MOTOR_IN2_PIN, GPIO_PIN_SET);
}