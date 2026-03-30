#ifndef ENCODER_H
#define ENCODER_H
 
/* ============================================================
 *  encoder.h  —  Quadrature encoder interface
 * ============================================================ */
 
#include "stm32f4xx_hal.h"
#include <stdint.h>

extern TIM_HandleTypeDef htim2;

/* Raw 32-bit signed position (counts from home after homing) */
typedef struct {
    int32_t  position;       /* Current position in counts                */
    int32_t  velocity;       /* Counts per control tick (signed)          */
    uint16_t last_raw;       /* Previous raw timer value (overflow detect)*/
} Encoder_t;
 
/* Init: configure TIM in encoder mode (call after HAL_TIM_Encoder_Start) */
void     Encoder_Init(Encoder_t *enc);
 
/* Call at fixed rate (control loop tick) to update position & velocity */
void     Encoder_Update(Encoder_t *enc);
 
/* Reset position to zero (call after homing) */
void     Encoder_ResetPosition(Encoder_t *enc);
 
/* Convert mm to encoder counts */
int32_t  Encoder_MmToCounts(float mm);
 
/* Convert encoder counts to mm */
float    Encoder_CountsToMm(int32_t counts);
 
#endif /* ENCODER_H */