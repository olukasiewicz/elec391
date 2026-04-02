#ifndef SOLENOID_H
#define SOLENOID_H
 
/* ============================================================
 *  solenoid.h  —  4-finger solenoid driver with safety timing
 * ============================================================ */
 
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

 
typedef enum {
    SOL_IDLE     = 0,
    SOL_ACTIVE   = 1,
    SOL_COOLDOWN = 2,
} Sol_State_t;
 
typedef struct {
    Sol_State_t state;
    volatile uint32_t    strike_time;    /* How long to play current note    */
    volatile uint32_t    on_ticks;       /* How long currently energised     */
    volatile uint32_t    off_ticks;      /* How long in cooldown             */
} Solenoid_t;
 
/* Initialise all solenoid GPIO pins (all OFF) */
void Solenoid_Init(void);
 
/* Trigger a single finger strike.  Non-blocking.
 * Returns false if the solenoid is in cooldown. */
bool Solenoid_Strike(uint8_t finger_idx, uint16_t strike_ms);
 
/* Release a finger immediately (force off) */
void Solenoid_Release(uint8_t finger_idx);
 
/* Release all fingers immediately */
void Solenoid_ReleaseAll(void);
 
/* Tick update — call every SCHEDULER_TICK_MS milliseconds */
void Solenoid_Tick(uint8_t finger_idx);
 
/* Convenience: tick all solenoids */
void Solenoid_TickAll(void);
 
/* Query if a finger is currently available (not striking/cooling) */
bool Solenoid_IsReady(uint8_t finger_idx);
 
#endif /* SOLENOID_H */