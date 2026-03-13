/* ============================================================
 *  main_app.c  —  Top-level application logic for piano robot
 *
 *  Integration pattern:
 *  ─────────────────────
 *  1. CubeMX generates main.c with HAL_Init, SystemClock_Config,
 *     MX_GPIO_Init, MX_TIMx_Init, etc.
 *  2. Add  #include "main_app.h"  to main.c
 *  3. Call App_Init()  inside main() after all MX_ inits.
 *  4. Call App_Tick()  from inside the while(1) loop.
 *  5. Call App_ControlTick() from your 2 kHz timer ISR.
 * ============================================================ */
 
#include "main_app.h"
#include "piano_robot_config.h"
#include "encoder.h"
#include "motor.h"
#include "homing.h"
#include "solenoid.h"
#include "piano_map.h"
#include "sequencer.h"
 
/* ---- Module instances ----------------------------------------- */
static Encoder_t    g_encoder;
static Motor_PID_t  g_pid;
static Homing_t     g_homing;
static Sequencer_t  g_sequencer;
 
/* ---- Application state --------------------------------------- */
typedef enum {
    APP_BOOT       = 0,
    APP_HOMING     ,
    APP_READY      ,
    APP_PLAYING    ,
    APP_FAULT      ,
} App_State_t;
 
static volatile App_State_t g_app_state = APP_BOOT;
 
/* ---- Control loop flag (set by timer ISR) --------------------- */
static volatile bool g_control_tick = false;
 
/* ================================================================
 *  Public API
 * ================================================================ */
 
void App_Init(void)
{
    Motor_Init();
    Encoder_Init(&g_encoder);
    Solenoid_Init();
    PianoMap_Init();
    Sequencer_Init(&g_sequencer);
 
    /* Start homing immediately */
    Homing_Start(&g_homing);
    g_app_state = APP_HOMING;
}
 
/* ------------------------------------------------------------------ */
/* Call from while(1) — handles state transitions and non-RT work      */
void App_Tick(void)
{
    /* Process control loop if flagged by ISR */
    if (g_control_tick) {
        g_control_tick = false;
        App_ControlTick();
    }
 
    switch (g_app_state)
    {
        case APP_BOOT:
            /* Should not linger here after App_Init */
            break;
 
        case APP_HOMING:
            /* State transitions happen inside App_ControlTick */
            break;
 
        case APP_READY:
            /* Load and start demo when ready */
            Sequencer_LoadDemo(&g_sequencer);
            Sequencer_Play(&g_sequencer);
            g_app_state = APP_PLAYING;
            break;
 
        case APP_PLAYING:
            if (Sequencer_IsDone(&g_sequencer)) {
                g_app_state = APP_READY;   /* Loop demo */
            }
            break;
 
        case APP_FAULT:
            /* TODO: signal fault LED, log error */
            Motor_Brake();
            Solenoid_ReleaseAll();
            break;
 
        default:
            break;
    }
}
 
/* ------------------------------------------------------------------ */
/* Call from high-rate timer ISR (CONTROL_LOOP_HZ)                     */
void App_ControlTick(void)
{
    Encoder_Update(&g_encoder);
 
    switch (g_app_state)
    {
        case APP_HOMING: {
            Homing_State_t hs = Homing_Update(&g_homing, &g_encoder, &g_pid);
            if (hs == HOMING_COMPLETE) {
                g_app_state = APP_READY;
            } else if (hs == HOMING_FAULT) {
                g_app_state = APP_FAULT;
            }
            break;
        }
 
        case APP_PLAYING:
            Sequencer_Tick(&g_sequencer, &g_encoder, &g_pid);
            break;
 
        default:
            break;
    }
}
 
/* ------------------------------------------------------------------ */
/* Call this from your TIMx period-elapsed callback in stm32f4xx_it.c */
void App_SetControlFlag(void)
{
    g_control_tick = true;
}
 
/* ------------------------------------------------------------------ */
/* Optional: directly play a single MIDI note (e.g. from UART command) */
void App_PlayNote(uint8_t midi_note)
{
    if (g_app_state != APP_READY && g_app_state != APP_PLAYING) return;
 
    const Piano_Key_t *key = PianoMap_Lookup(midi_note);
    if (key == NULL) return;
 
    int32_t target = Encoder_MmToCounts(key->position_mm);
    Motor_SetTarget(&g_pid, target);
    /* Solenoid will be triggered once Motor_AtTarget in next tick */
}
