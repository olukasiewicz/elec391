#ifndef PIANO_ROBOT_CONFIG_H
#define PIANO_ROBOT_CONFIG_H
 
/* ============================================================
 *  piano_robot_config.h
 *  Central hardware & tuning configuration for the piano robot.
 *  Edit this file to match your PCB pin assignments and tuning.
 * ============================================================ */
 
#include "stm32f4xx_hal.h"
 
/* ----------------------------------------------------------
 * MCU PERIPHERAL HANDLES
 * Declare externs here; define in main.c / CubeMX generated code
 * ---------------------------------------------------------- */
extern TIM_HandleTypeDef htim1;   /* PWM timer for H-bridge (TIM1 CH1/CH2) */
extern TIM_HandleTypeDef htim2;   /* Encoder timer (TIM2, encoder mode)     */
extern TIM_HandleTypeDef htim3;   /* Solenoid PWM / tick timer (optional)   */

/* ----------------------------------------------------------
 * ENCODER
 * ---------------------------------------------------------- */
#define ENCODER_TIM             htim2
#define ENCODER_CPR             1024u        /* Counts per revolution (×4 mode) */
#define ENCODER_PULLEY_MM       (50.0f * 3.14159265f) /* Circumference in mm   */
/* counts per mm = (ENCODER_CPR * 4) / ENCODER_PULLEY_MM  — computed at runtime */
 
/* ----------------------------------------------------------
 * SOLENOIDS
 * ---------------------------------------------------------- */
#define SOLENOID_COUNT          5u
#define SOL_MIN_OFF_MS          20u    /* ms minimum rest before re-trigger        */
#define SOL_MAX_ON_MS           1000u    /* Hard safety cutoff (heat protection)     */

/* Finger index aliases */
#define FINGER_WHITE_0   0u
#define FINGER_BLACK_1   1u
#define FINGER_WHITE_2   2u
#define FINGER_WHITE_3   3u
#define FINGER_WHITE_4   4u

/* ----------------------------------------------------------
 * MOTOR
 * ---------------------------------------------------------- */
#define POS_DEADBAND_COUNTS 10   

/* ----------------------------------------------------------
 * HOMING
 * ---------------------------------------------------------- */
#define HOMING_TIMEOUT_MS          100000u   /* Fault if homing not done in 5s    */
#define MOTOR_HOMING_DUTY_CYCLE    40      /* speed towards 1st switch hit */
#define MOTOR_CREEP_DUTY_CYCLE     35      /* speed for second switch hit */
#define BACKOFF_MM                 25.0f     /* mm to back off after switch triggers  */
 

#define CONTROL_LOOP_HZ 1000

#endif /* PIANO_ROBOT_CONFIG_H */