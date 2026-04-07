#include "homing.h"
#include "app_pid.h"
#include "encoder.h"
#include "motor.h"
#include "piano_robot_config.h"
#include "main.h"
#include "stm32f4xx_hal_gpio.h"
#include "stdlib.h"
#include <stdbool.h>
#include <stdint.h>

#define TIMEOUT_TICKS   (HOMING_TIMEOUT_MS * CONTROL_LOOP_HZ / 1000u)

static inline bool switch_active(void)
{
    return HAL_GPIO_ReadPin(Home_SENS_GPIO_Port, Home_SENS_Pin) == GPIO_PIN_RESET;
}

/* ------------------------------------------------------------------ */
void Homing_Start(Homing_t *h)
{   
    h->state            = HOMING_MOVING;
    h->elapsed_ticks    = 0;
    h->timeout_ticks    = TIMEOUT_TICKS;
}

/* ------------------------------------------------------------------ */
Homing_State_t Homing_Update(Homing_t *h, Encoder_t *enc, PID *pid)
{
    h->elapsed_ticks++;
    /* homing timout */ 
    // if (h->elapsed_ticks > h->timeout_ticks &&
    //     h->state != HOMING_COMPLETE)
    // {
    //     Motor_Brake();
    //     h->state = HOMING_FAULT;
    //     return h->state;
    // } 

    // statemachine
    switch(h->state)
    {
        case HOMING_MOVING:
            Motor_Drive(MOTOR_HOMING_DUTY_CYCLE, MOTOR_DIR_REVERSE);

            if (switch_active()){
                Motor_Coast();
                // back off by small amount
                int32_t backoff_counts = Encoder_MmToCounts(BACKOFF_MM);
                // set motor target to enc_position + backoff counts
                Motor_SetTarget(enc->position + backoff_counts);
                h->state = HOMING_BACKOFF;
            }
            break;

        case HOMING_BACKOFF:
            // update pid and drive motor
            Motor_Update(pid, enc);
            if (Motor_AtTarget(enc)) {
                app_pid_requestReset(pid);
                Motor_Coast();
                Encoder_ResetPosition(enc);
                h->state = HOMING_COMPLETE;
            }
            break;

        case HOMING_COMPLETE:
        case HOMING_FAULT:
        case HOMING_IDLE:
        default:
            break;
    }
    return h->state;
}

/* ------------------------------------------------------------------ */
bool Homing_IsDone(const Homing_t *h)
{
    return (h->state == HOMING_COMPLETE);
}

bool Homing_HasFault(const Homing_t *h)
{
    return (h->state == HOMING_FAULT);
}

