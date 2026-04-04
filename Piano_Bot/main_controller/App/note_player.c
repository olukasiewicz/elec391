#include "app_pid.h"
#include "motor.h"
#include "piano_robot_config.h"
#include "main.h"
#include "solenoid.h"
#include "stm32f4xx_hal.h"
#include "note_player.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>


#define BPM 120
#define QUARTER_TIME 60000 / BPM // 500 at 120bpm
#define EIGTH_TIME 30000 / BPM // 250 at 120bpm


static NotePlayer_t g_notePlayer;
uint32_t note_delay_ms = 0;

static const BaseEvent eventArray[] = {
    {EVENT_SINGLE, {true, false, false, false, false}, QUARTER_TIME, 0, 0},
    {EVENT_SINGLE, {true, false, false, false, false}, EIGTH_TIME, 0, 700},
    {EVENT_SINGLE, {false, false, true, false, false}, EIGTH_TIME, 0, 700},
    {EVENT_SINGLE, {false, false, false, true, false}, EIGTH_TIME, 500, 700},
    {EVENT_DOUBLE, {true, true, false, false, false}, QUARTER_TIME, 0, 1500},
};



/* ------------------------------------------------------------------ */
void NotePlayer_Init(void)
{
    g_notePlayer.song           = eventArray;
    g_notePlayer.song_length    = sizeof(eventArray) / sizeof(eventArray[0]);
    g_notePlayer.current_index  = 0U;
    g_notePlayer.noteStart_ms   = 0U;
    g_notePlayer.run_state      = NOTE_PLAYER_STATE_IDLE;
}

/* ------------------------------------------------------------------ */
void NotePlayer_Run(NotePlayer_t *state, PID *pid, const Encoder_t *encoder)
{   
    const BaseEvent *event = &state->song[state->current_index];
    uint32_t now = HAL_GetTick();

    switch(state->run_state)
    {
        /* waiting for first call*/
        case NOTE_PLAYER_STATE_IDLE:
            state->current_index = 0u;
            state->run_state     = NOTE_PLAYER_STATE_MOVE_MOTOR;
            break;
        
        /* start moving to target and start delay timer*/
        case NOTE_PLAYER_STATE_MOVE_MOTOR:
            app_pid_requestReset(pid);
            Motor_SetTarget(event->target_position);
            state->noteStart_ms = now;
            state->run_state    = NOTE_PLAYER_STATE_WAIT_READY;
            break;
        
        /* wait until motor is at target and delay timer reached*/
        case NOTE_PLAYER_STATE_WAIT_READY:
            if (Motor_AtTarget(encoder) && (now - state->noteStart_ms >= note_delay_ms)) {
                state->run_state = NOTE_PLAYER_STATE_STRIKE_NOTE;
            }
            break;
        
        /* fire all solenoids in this event*/
        case NOTE_PLAYER_STATE_STRIKE_NOTE:
            for (uint8_t i = 0; i < SOLENOID_COUNT; i++) {
                if (event->solenoid_index[i]) {
                    Solenoid_Strike(i, event->duration_ms);
                }
            }
            note_delay_ms = event->time_to_next_ms;
            state->run_state = NOTE_PLAYER_STATE_WAIT_NOTE_COMPLETE;
            break;
        
        /* wait until all solenoids released */
        case NOTE_PLAYER_STATE_WAIT_NOTE_COMPLETE:
            if (Solenoid_AllReady()){
                state->current_index++;
                if (state->current_index >= state->song_length) {
                    state->run_state = NOTE_PLAYER_STATE_FINISHED;
                } else {
                    state->run_state = NOTE_PLAYER_STATE_MOVE_MOTOR;
                }
            }
            break;
        
        
        case NOTE_PLAYER_STATE_FINISHED:

            break;
        
        /* should not be reached */
        default:
            break;

    }
}


/* ------------------------------------------------------------------ */
NotePlayer_t *NotePlayer_GetState(void)
{
    return &g_notePlayer;
}


bool NotePlayer_IsDone(void){
    return g_notePlayer.run_state == NOTE_PLAYER_STATE_FINISHED; 
}