/* ============================================================
 *  solenoid.c  —  4-finger solenoid driver with safety timing
 * ============================================================ */
 
#include "solenoid.h"
 
/* ---- GPIO lookup table ------------------------------------------- */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} Sol_GPIO_t;
 
static const Sol_GPIO_t s_gpio[SOLENOID_COUNT] = {
    { SOL0_GPIO_PORT, SOL0_PIN },
    { SOL1_GPIO_PORT, SOL1_PIN },
    { SOL2_GPIO_PORT, SOL2_PIN },
    { SOL3_GPIO_PORT, SOL3_PIN },
};
 
/* ---- State array -------------------------------------------------- */
static Solenoid_t s_sol[SOLENOID_COUNT];

/* ------------------------------------------------------------------ */
static inline void sol_set(uint8_t idx, bool on)
{
    HAL_GPIO_WritePin(s_gpio[idx].port, s_gpio[idx].pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
 
/* ------------------------------------------------------------------ */
void Solenoid_Init(void)
{
    for (uint8_t i = 0; i < SOLENOID_COUNT; i++) {
        sol_set(i, false);
        s_sol[i].state    = SOL_IDLE;
        s_sol[i].on_ticks = 0;
        s_sol[i].off_ticks= 0;
    }
}
 
/* ------------------------------------------------------------------ */
bool Solenoid_Strike(uint8_t finger_idx)
{
    if (finger_idx >= SOLENOID_COUNT) return false;
    Solenoid_t *s = &s_sol[finger_idx];
 
    if (s->state != SOL_IDLE) return false;  /* busy or cooling */
 
    sol_set(finger_idx, true);
    s->state    = SOL_ACTIVE;
    s->on_ticks = 0;
    return true;
}

/* ------------------------------------------------------------------ */
void Solenoid_Release(uint8_t finger_idx)
{
    if (finger_idx >= SOLENOID_COUNT) return;
    sol_set(finger_idx, false);
    s_sol[finger_idx].state     = SOL_COOLDOWN;
    s_sol[finger_idx].off_ticks = 0;
}

/* ------------------------------------------------------------------ */
void Solenoid_ReleaseAll(void)
{
    for (uint8_t i = 0; i < SOLENOID_COUNT; i++)
        Solenoid_Release(i);
}

/* ------------------------------------------------------------------ */
void Solenoid_Tick(uint8_t finger_idx)
{
    if (finger_idx >= SOLENOID_COUNT) return;
    Solenoid_t *s = &s_sol[finger_idx];
 
    switch (s->state)
    {
        case SOL_ACTIVE:
            s->on_ticks++;
            /* Safety cutoff — never stay on beyond maximum */
            if (s->on_ticks >= SOL_MAX_ON_MS ||
                s->on_ticks >= SOL_STRIKE_MS)
            {
                sol_set(finger_idx, false);
                s->state     = SOL_COOLDOWN;
                s->off_ticks = 0;
            }
            break;
 
        case SOL_COOLDOWN:
            s->off_ticks++;
            if (s->off_ticks >= SOL_MIN_OFF_MS) {
                s->state = SOL_IDLE;
            }
            break;
 
        case SOL_IDLE:
        default:
            break;
    }
}

/* ------------------------------------------------------------------ */
void Solenoid_TickAll(void)
{
    for (uint8_t i = 0; i < SOLENOID_COUNT; i++)
        Solenoid_Tick(i);
}

/* ------------------------------------------------------------------ */
bool Solenoid_IsReady(uint8_t finger_idx)
{
    if (finger_idx >= SOLENOID_COUNT) return false;
    return (s_sol[finger_idx].state == SOL_IDLE);
}