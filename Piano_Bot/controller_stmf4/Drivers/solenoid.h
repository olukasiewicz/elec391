#ifndef SOLENOID_H
#define SOLENOID_H
 
/* ============================================================
 *  solenoid.h  —  4-finger solenoid driver with safety timing
 * ============================================================ */
 
#include "stm32f4xx_hal.h"
#include "piano_robot_config.h"
#include <stdint.h>
#include <stdbool.h>

/* Finger index aliases */
#define FINGER_WHITE_0   0u
#define FINGER_WHITE_1   1u
#define FINGER_BLACK_0   2u
#define FINGER_BLACK_1   3u
 
typedef enum {
    SOL_IDLE     = 0,
    SOL_ACTIVE   = 1,
    SOL_COOLDOWN = 2,
} Sol_State_t;
 
typedef struct {
    Sol_State_t state;
    uint32_t    on_ticks;       /* How long currently energised     */
    uint32_t    off_ticks;      /* How long in cooldown             */
} Solenoid_t;
 
/* Initialise all solenoid GPIO pins (all OFF) */
void Solenoid_Init(void);
 
/* Trigger a single finger strike.  Non-blocking.
 * Returns false if the solenoid is in cooldown. */
bool Solenoid_Strike(uint8_t finger_idx);
 
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