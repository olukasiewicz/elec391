/* ============================================================
 *  encoder.c  —  Quadrature encoder interface (STM32 TIM encoder mode)
 * ============================================================ */
 
#include "encoder.h"
#include "piano_robot_config.h"
#include <math.h>

// #define ENCODER_TIM htim2

#define ENCODER_CPR             1024u        /* Counts per revolution (×4 mode) */
#define ENCODER_PULLEY_MM       (50.0f * 3.14159265f) /* Circumference in mm   */
/* counts per mm = (ENCODER_CPR * 4) / ENCODER_PULLEY_MM  — computed at runtime */
 
/* Counts per mm (computed once at init) */
static float s_counts_per_mm = 0.0f;
 
/* ------------------------------------------------------------------ */
void Encoder_Init(Encoder_t *enc)
{
    /* Start encoder timer in interrupt mode (or polling — your choice) */
    HAL_TIM_Encoder_Start(&ENCODER_TIM, TIM_CHANNEL_ALL);
 
    /* Pre-compute resolution */
    s_counts_per_mm = ((float)(ENCODER_CPR * 4u)) / ENCODER_PULLEY_MM;
 
    enc->position  = 0;
    enc->velocity  = 0;
    enc->last_raw  = (uint16_t)__HAL_TIM_GET_COUNTER(&ENCODER_TIM);
}
 
/* ------------------------------------------------------------------ */
void Encoder_Update(Encoder_t *enc)
{
    uint16_t raw     = (uint16_t)__HAL_TIM_GET_COUNTER(&ENCODER_TIM);
    int16_t  delta   = (int16_t)(raw - enc->last_raw); /* handles 16-bit overflow */
    enc->last_raw    = raw;
    enc->position   += (int32_t)delta;
    enc->velocity    = (int32_t)delta;
}
 
/* ------------------------------------------------------------------ */
void Encoder_ResetPosition(Encoder_t *enc)
{
    __HAL_TIM_SET_COUNTER(&ENCODER_TIM, 0);
    enc->position = 0;
    enc->velocity = 0;
    enc->last_raw = 0;
}
 
/* ------------------------------------------------------------------ */
int32_t Encoder_MmToCounts(float mm)
{
    return (int32_t)(mm * s_counts_per_mm);
}
 
/* ------------------------------------------------------------------ */
float Encoder_CountsToMm(int32_t counts)
{
    if (s_counts_per_mm == 0.0f) return 0.0f;
    return (float)counts / s_counts_per_mm;
}