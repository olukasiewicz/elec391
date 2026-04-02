#ifndef MOTOR_H
#define MOTOR_H
 
/* ============================================================
 *  motor.h  —  H-bridge DC motor driver with PID position control
 * ============================================================ */
 
#include "stm32f4xx_hal.h"
#include "encoder.h"
#include <stdint.h>
#include <stdbool.h>
#include "app_pid.h"

typedef enum {
    MOTOR_DIR_STOP    = 0,
    MOTOR_DIR_FORWARD = 1,
    MOTOR_DIR_REVERSE = 2,
    MOTOR_DIR_BRAKE   = 3,
} Motor_Dir_t;


void set_pwm_duty(float duty_cycle);

/* Initialise GPIO & PWM timer */
void Motor_Init(void);
 
/* Raw drive: speed 0–100 %, direction */
void Motor_Drive(float duty_cycle, Motor_Dir_t dir);
 
/* Immediate stop (coast) */
void Motor_Coast(void);
 
/* Active braking (both low-side on) */
void Motor_Brake(void);
 
/* PID update — call at CONTROL_LOOP_HZ rate with current encoder */
void Motor_Update(PID *pid, Encoder_t *enc);
 
/* Returns true when position is within deadband of target */
bool Motor_AtTarget(const Encoder_t *enc);

/* Sets motor target in counts*/
void Motor_SetTarget(float target);


float Motor_GetTarget(void);
 
#endif /* MOTOR_H */