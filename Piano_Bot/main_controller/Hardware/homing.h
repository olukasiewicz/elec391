#ifndef HOMING_H
#define HOMING_H
 
/* ============================================================
 *  homing.h  —  Carriage homing state machine
 * ============================================================ */

#include "motor.h"
#include "encoder.h"
#include <stdbool.h>

#define HOMING_TIMEOUT_MS   5000u  /* Fault if homing not done in 5 s    */

typedef enum {
    HOMING_IDLE        = 0,
    HOMING_MOVING      ,   /* Moving toward switch at homing speed     */
    HOMING_BACKOFF     ,   /* Backing off the switch (debounce travel)  */
    HOMING_CREEP       ,   /* Slow creep back onto switch               */
    HOMING_COMPLETE    ,   /* Success — encoder zeroed                  */
    HOMING_FAULT       ,   /* Timeout or switch never triggered         */
} Homing_State_t;


typedef struct {
    Homing_State_t state;
    uint32_t       timeout_ticks;  /* Tick counter for fault detection   */
    uint32_t       elapsed_ticks;
} Homing_t;

/* Start a homing sequence */
void Homing_Start(Homing_t *h);
 
/* Call every control tick; returns current state */
Homing_State_t Homing_Update(Homing_t *h, Encoder_t *enc, PID *pid);
 
/* True once homing has completed successfully */
bool Homing_IsDone(const Homing_t *h);

bool Homing_HasFault(const Homing_t *h);

#endif /* HOMING_H */