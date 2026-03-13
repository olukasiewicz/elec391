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
 * H-BRIDGE / MOTOR GPIO
 * ---------------------------------------------------------- */
#define MOTOR_IN1_GPIO_PORT     GPIOA
#define MOTOR_IN1_PIN           GPIO_PIN_0   /* H-bridge direction IN1 */
#define MOTOR_IN2_GPIO_PORT     GPIOA
#define MOTOR_IN2_PIN           GPIO_PIN_1   /* H-bridge direction IN2 */
#define MOTOR_PWM_TIM           htim1
#define MOTOR_PWM_CHANNEL       TIM_CHANNEL_1
#define MOTOR_PWM_MAX           1000u        /* ARR value → 100 % duty  */
 
/* ----------------------------------------------------------
 * ENCODER
 * ---------------------------------------------------------- */
#define ENCODER_TIM             htim2
#define ENCODER_CPR             1024u        /* Counts per revolution (×4 mode) */
#define ENCODER_PULLEY_MM       (50.0f * 3.14159265f) /* Circumference in mm   */
/* counts per mm = (ENCODER_CPR * 4) / ENCODER_PULLEY_MM  — computed at runtime */
 
/* ----------------------------------------------------------
 * HOMING SWITCH
 * ---------------------------------------------------------- */
#define HOME_SWITCH_GPIO_PORT   GPIOB
#define HOME_SWITCH_PIN         GPIO_PIN_0
#define HOME_SWITCH_ACTIVE      GPIO_PIN_RESET  /* Active-low (NC switch to GND) */
 
/* ----------------------------------------------------------
 * SOLENOID GPIO  (4 fingers)
 * Finger 0 & 1 → white keys
 * Finger 2 & 3 → black keys
 * ---------------------------------------------------------- */
#define SOLENOID_COUNT          4u
 
#define SOL0_GPIO_PORT          GPIOC
#define SOL0_PIN                GPIO_PIN_0
#define SOL1_GPIO_PORT          GPIOC
#define SOL1_PIN                GPIO_PIN_1
#define SOL2_GPIO_PORT          GPIOC
#define SOL2_PIN                GPIO_PIN_2
#define SOL3_GPIO_PORT          GPIOC
#define SOL3_PIN                GPIO_PIN_3
 
/* ---- Solenoid timing ---------------------------------------- */
#define SOL_STRIKE_MS           30u    /* ms solenoid stays energised (key press)  */
#define SOL_MIN_OFF_MS          20u    /* ms minimum rest before re-trigger        */
#define SOL_MAX_ON_MS           80u    /* Hard safety cutoff (heat protection)     */
 
/* ----------------------------------------------------------
 * MOTION CONTROL
 * ---------------------------------------------------------- */
#define MOTOR_MAX_SPEED_PCT     80u    /* % of PWM_MAX used during normal travel   */
#define MOTOR_HOMING_SPEED_PCT  30u    /* % of PWM_MAX used during homing          */
#define MOTOR_CREEP_SPEED_PCT   15u    /* % of PWM_MAX used for fine positioning   */
#define POS_DEADBAND_COUNTS     3      /* ±counts tolerance for "at target"        */
#define PID_KP                  2.0f
#define PID_KI                  0.5f
#define PID_KD                  0.1f
#define PID_INTEGRAL_LIMIT      500.0f
 
/* ----------------------------------------------------------
 * PIANO LAYOUT
 * White key pitch (C4 = middle C = key 0 in our map)
 * All positions stored in encoder counts from home.
 * ---------------------------------------------------------- */
#define PIANO_WHITE_KEY_PITCH_MM    23.5f
#define PIANO_BLACK_KEY_OFFSET_MM   13.5f  /* relative to white key centre */
#define PIANO_NUM_OCTAVES           3u
 
/* Finger offsets from carriage datum (mm) */
#define FINGER0_OFFSET_MM       0.0f    /* White finger 0 */
#define FINGER1_OFFSET_MM       23.5f   /* White finger 1 (one white key right)  */
#define FINGER2_OFFSET_MM       0.0f    /* Black finger 2 (same column, black row)*/
#define FINGER3_OFFSET_MM       13.5f   /* Black finger 3                        */
 
/* ----------------------------------------------------------
 * SYSTEM TICK
 * ---------------------------------------------------------- */
#define CONTROL_LOOP_HZ         500u   /* Position control loop rate (Hz) */
#define SCHEDULER_TICK_MS       1u     /* Main scheduler tick period (ms)  */
 
#endif /* PIANO_ROBOT_CONFIG_H */