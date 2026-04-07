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
#define ENCODER_CPR             1200u // this is after gear (64*18.75)
#define ENCODER_PULLEY_MM       (100f) /* Circumference in mm   */ //30 * 3.14159265
#define COUNTS_TO_MM            12.0f
/* counts per mm = (ENCODER_CPR * 1) / ENCODER_PULLEY_MM  — computed at runtime */
 
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
#define POS_DEADBAND_COUNTS 4.0f  
#define MAX_DUTY_FORWARD 65.0f
#define MAX_DUTY_REVERSE -65.0f

/* ----------------------------------------------------------
 * HOMING
 * ---------------------------------------------------------- */
#define HOMING_TIMEOUT_MS          100000u   /* Fault if homing not done   */
#define MOTOR_HOMING_DUTY_CYCLE    65      /* speed towards 1st switch hit */
#define MOTOR_CREEP_DUTY_CYCLE     65      /* speed for second switch hit */
#define BACKOFF_MM                 5.0f     /* mm to back off after switch triggers  */
 

#define CONTROL_LOOP_HZ 1000

/* ----------------------------------------------------------------*/
#define SKIP_HOMING 0
#define SKIP_SONG 0
#define MOTOR_ALWAYS_AT_TARGET 0
/* ----------------------------------------------------------------*/


/* ----------------------------------------------------------
 * UI
 * ---------------------------------------------------------- */
 #define AUTO_BLINK_DELAY 5000
 #define BLINK_DELAY 200
 #define DEBOUNCE_MS 100

#define NUM_SONGS 2

/* key constants */
#define WHITE_MM 21.5f
#define BLACK_MM 11.7f
#define BLACK_OFFSET_MM 25.0f
#define OCTAVE_MM 160.0f

#define OCTAVE_CNT 1920
#define WHITE_CNT 276

// all relative to home at C3=0
#define C3_note 0.0f
#define E3_note 552
#define G3_note 1104

#define C4_note OCTAVE_CNT
#define E4_note OCTAVE_CNT + 552
#define G4_note OCTAVE_CNT + G3_note

#endif /* PIANO_ROBOT_CONFIG_H */