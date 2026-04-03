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
 *  5. Call App_SetControlFlag from your 1 kHz timer ISR.
 * ============================================================ */
 
#include "app_main.h"
#include "app_pid.h"
#include "main.h"
#include "piano_robot_config.h"
#include "encoder.h"
#include "motor.h"
#include "homing.h"
#include "solenoid.h"
#include "note_player.h"

#include "stm32f4xx_hal_gpio.h"
#include <stdbool.h>

// #include "sequencer.h"

/* ---- Module instances ----------------------------------------- */
static Encoder_t    g_encoder;
static Homing_t     g_homing;
static PID          g_pid;
const PID_Config PID_CONF = {
    .Kp = 1.0f, //5
    .Ki = 0.0f, // 5
    .Kd = 0.5f, //0.05
    .Kb = 0.0f,

    .Kff = 0.0f, // 0.5
    .smoothing_coeff = 0.0f, // 0.2

    .out_max = 95.0f,
    .out_min = -95.0f,
    .max_integral = 50.0f,
    .min_integral = -50.0f,

    .clamp_output = true,
    .clamp_integral = true,
    .back_calculation = false,
    .feed_forward = false, // true
    .sample_time = 0.001f
};
// static Sequencer_t  g_sequencer;

/* ---- Application state --------------------------------------- */
typedef enum {
    APP_BOOT       = 0,
    APP_IDLE       ,
    APP_HOMING     ,
    APP_READY      ,
    APP_PLAYING    ,
    APP_DONE       ,
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
    app_pid_init(&g_pid, &PID_CONF);
    NotePlayer_Init();

    g_homing.state = HOMING_IDLE;

    // PianoMap_Init();
    // Sequencer_Init(&g_sequencer);

    g_app_state = APP_IDLE;
}

void App_Tick(void)
{
    /* run on control tick (CONTROLL_HZ) */
    if (g_control_tick) {
        g_control_tick = false;
        App_ControlTick();
        HAL_GPIO_TogglePin(DEBUG_GPIO_Port, DEBUG_Pin);
        Solenoid_TickAll();
    }

    /* runs faster between control ticks*/
    switch (g_app_state) {
        case APP_BOOT:
            /* pre-init state, should not remain here*/
            break;
        
        case APP_IDLE:
            /* after App_Init(), wait for home button to start*/
            if (HAL_GPIO_ReadPin(Home_Button_GPIO_Port, Home_Button_Pin) == GPIO_PIN_SET){
                g_app_state = APP_HOMING;
                g_app_state = APP_READY; // TODO ---------------------------- DELETE
            }
            break;  
        
        case APP_HOMING:
            /* homing state transitions happen inside App_ControlTick*/
            break;

        case APP_READY:
            /* transition state to start playing */
            // call any player start funcs
            g_app_state = APP_PLAYING;
            break;

        case APP_PLAYING:
            /* stay until app is done playing */
            if (NotePlayer_IsDone()) {
                g_app_state = APP_DONE;
            }
            break;
        
        case APP_DONE:
            /* just jump back to idle */
            HAL_GPIO_WritePin(DEBUG_GPIO_Port, DEBUG_Pin, GPIO_PIN_SET);
            //g_app_state = APP_IDLE;
            break;

        case APP_FAULT:
            /* some fault */
            Motor_Brake();
            Solenoid_ReleaseAll();
            break;

        default: 
            break;
    }
}

bool sol_triggered = false;
/* ------------------------------------------------------------------ */
/* Call from high-rate timer ISR (CONTROL_LOOP_HZ)                    */
void App_ControlTick(void)
{
    switch (g_app_state) {
        case APP_READY:
            /* TODO delete state */
            break;

        case APP_HOMING:
            Homing_State_t hs = Homing_Update(&g_homing, &g_encoder, &g_pid);
            if (hs == HOMING_COMPLETE){
                g_app_state = APP_READY;
            } else if (hs == HOMING_FAULT) {
                g_app_state = APP_FAULT;
            }
            break;
        
        case APP_PLAYING:
            // TODO: call sequencer function
            // Motor_SetTarget(float target)
            // Motor_Update(PID *pid, Encoder_t *enc)
            // Solenoid_Strike(uint8_t finger_idx, uint16_t strike_ms)
            NotePlayer_Run(NotePlayer_GetState(), &g_pid, &g_encoder);
            Motor_Update(&g_pid, &g_encoder);
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