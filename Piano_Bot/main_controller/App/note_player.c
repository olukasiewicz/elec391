#include "app_pid.h"
#include "encoder.h"
#include "motor.h"
#include "piano_robot_config.h"
#include "main.h"
#include "solenoid.h"
#include "stm32f4xx_hal.h"
#include "note_player.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>


#define BPM 91
#define WHOLE_TIME      240000 / BPM     // 2000 at 120bpm
#define HALF_TIME       120000 / BPM      // 1000 at 120bpm
#define QUARTER_TIME     60000 / BPM    // 500 at 120bpm
#define EIGTH_TIME       30000 / BPM      // 250 at 120bpm
#define TRIPLET_TIME     20000 / BPM      // 250 at 120bpm

#define FED_TRIPLET \
    {F2, TRIPLET_TIME, 0, C4_note+WHITE_CNT}, \
    {F2, TRIPLET_TIME, 0, C4_note}, \
    {F1, TRIPLET_TIME, 0, C4_note+WHITE_CNT}

#define MELODY \
    {F4, EIGTH_TIME, 0, G3_note}, \
    {F4, EIGTH_TIME, 0, G3_note-WHITE_CNT},  \
    {F3, EIGTH_TIME, 0, G3_note},\
    {F3, EIGTH_TIME, 0, G3_note-WHITE_CNT}, \
    {F2|F3, TRIPLET_TIME, 0, G3_note-WHITE_CNT}, \
    {F3, TRIPLET_TIME, 0, G3_note},\
    {F4, TRIPLET_TIME, 0, G3_note-WHITE_CNT}, \
    {F3, QUARTER_TIME, 0, G3_note}

static NotePlayer_t g_notePlayer;
uint32_t note_delay_ms = 0;

static const BaseEvent Main_Theme[] = { 
    {F1, TRIPLET_TIME, 0, G3_note},
    {F1, TRIPLET_TIME, 0, G3_note},
    {F1, TRIPLET_TIME, 0, G3_note},

    {F1, HALF_TIME, 0, C4_note},
    {F3, HALF_TIME, 0, C4_note},
    
    FED_TRIPLET,
    {F4, HALF_TIME, 0, C4_note+WHITE_CNT}, // outside C
    {F3, QUARTER_TIME, 0, C4_note}, 

    FED_TRIPLET,
    {F4, HALF_TIME, 0, C4_note+WHITE_CNT}, // outside C
    {F3, QUARTER_TIME, 0, C4_note}, 
    
    {F3, TRIPLET_TIME, 0, C4_note-WHITE_CNT}, 
    {F3, TRIPLET_TIME, 0, C4_note-WHITE_CNT*2}, 
    {F4, TRIPLET_TIME, 0, G3_note}, 
    
    {F1|F2|F3, HALF_TIME, 0, G3_note}, 

    {F1, EIGTH_TIME, 0, G3_note}, 
    {F1, EIGTH_TIME, 0, G3_note}, 

    /* repeat above*/
    
    {F1, HALF_TIME, 0, C4_note}, // C
    {F3, HALF_TIME, 0, C4_note}, // G

    FED_TRIPLET,
    {F4, HALF_TIME, 0, C4_note+WHITE_CNT}, // outside C
    {F3, QUARTER_TIME, 0, C4_note}, 

    FED_TRIPLET,
    {F4, HALF_TIME, 0, C4_note+WHITE_CNT}, // outside C
    {F3, QUARTER_TIME, 0, C4_note}, 

    {F3, TRIPLET_TIME, 0, C4_note-WHITE_CNT}, 
    {F3, TRIPLET_TIME, 0, C4_note-WHITE_CNT*2}, 
    {F4, TRIPLET_TIME, 0, G3_note}, 
    
    {F1|F2|F3, HALF_TIME, 0, G3_note}, 

    {F1, EIGTH_TIME, 0, G3_note}, // g
    {F1, EIGTH_TIME, 0, G3_note}, // g

    /* end repeat */

    {F2, QUARTER_TIME+EIGTH_TIME, 0, G3_note-WHITE_CNT}, // A
    {F2, EIGTH_TIME, 0, G3_note-WHITE_CNT}, // A

    MELODY,
    
    {F2, QUARTER_TIME, 0, G3_note}, // B
    {F1, EIGTH_TIME, 0, G3_note}, // G
    {F1, EIGTH_TIME, 0, G3_note}, // G
    {F1, QUARTER_TIME+EIGTH_TIME, 0, G3_note+WHITE_CNT}, // A
    {F1, EIGTH_TIME, 0, G3_note+WHITE_CNT}, // A

    {F3, EIGTH_TIME, 0, C4_note-WHITE_CNT}, // F
    {F3, EIGTH_TIME, 0, C4_note-WHITE_CNT*2}, // E
    {F2, EIGTH_TIME, 0, C4_note-WHITE_CNT}, // D

    {F2, EIGTH_TIME, 0, C4_note-WHITE_CNT*2}, // C
    {FB|F4, QUARTER_TIME, 0, C4_note-WHITE_CNT*2}, // B sharp g

    {F1|F2|F3, HALF_TIME, 0, G3_note}, // chord @37s

    /* repeat */

    {F1, EIGTH_TIME, 0, G3_note}, // g
    {F1, EIGTH_TIME, 0, G3_note}, // g

    {F1, QUARTER_TIME+EIGTH_TIME, 0, G3_note+WHITE_CNT}, // A
    {F1, EIGTH_TIME, 0, G3_note+WHITE_CNT}, // A

    MELODY,

    {F2, QUARTER_TIME, 0, G3_note}, // B

    {F2, EIGTH_TIME, 0, C4_note+WHITE_CNT*2}, // G
    {F2, EIGTH_TIME, 0, C4_note+WHITE_CNT*2}, // G
    {F1|F3, EIGTH_TIME, 0, C4_note+WHITE_CNT*3}, // F|C

    {FB, EIGTH_TIME, 0, G4_note+WHITE_CNT}, // B sharp
    {FB, EIGTH_TIME, 0, G4_note}, // A sharp
    {F1, EIGTH_TIME, 0, G4_note}, // G 
    {F2, EIGTH_TIME, 0, C4_note+WHITE_CNT}, // F
    {FB, EIGTH_TIME, 0, C4_note+WHITE_CNT}, // E sharp
    {F1, EIGTH_TIME, 0, C4_note+WHITE_CNT}, // D
    {F1, EIGTH_TIME, 0, C4_note}, // C
    {F3, HALF_TIME, 0, C4_note}, // G

    {F1, TRIPLET_TIME, 0, G3_note}, // G
    {F1, TRIPLET_TIME, 0, G3_note}, // G
    {F1, TRIPLET_TIME, 0, G3_note}, // G
    {F1, QUARTER_TIME, 0, G3_note}, // G
    {F1, TRIPLET_TIME, 0, G3_note}, // G
    {F1, TRIPLET_TIME, 0, G3_note}, // G
    {F1, TRIPLET_TIME, 0, G3_note}, // G

    {F1, HALF_TIME, 0, C4_note}, // C
    {F3, HALF_TIME, 0, C4_note}, // G

    FED_TRIPLET,
    {F4, HALF_TIME, 0, C4_note+WHITE_CNT}, // outside C
    {F3, QUARTER_TIME, 0, C4_note}, 

    FED_TRIPLET,
    {F4, HALF_TIME, 0, C4_note+WHITE_CNT}, // outside C
    {F3, QUARTER_TIME, 0, C4_note}, 

    {F3, TRIPLET_TIME, 0, C4_note-WHITE_CNT}, 
    {F3, TRIPLET_TIME, 0, C4_note-WHITE_CNT*2}, 
    {F4, TRIPLET_TIME, 0, G3_note}, 
    
    {F1|F2|F3, HALF_TIME, 0, G3_note}, 
    {F1, EIGTH_TIME, 0, G3_note}, 
    {F1, EIGTH_TIME, 0, G3_note}, 

    {F1, HALF_TIME, 0, C4_note}, // C
    {F3, HALF_TIME, 0, C4_note}, // G

    FED_TRIPLET,
    {F4, HALF_TIME, 0, C4_note+WHITE_CNT}, // outside C
    {F3, QUARTER_TIME, 0, C4_note}, 

    FED_TRIPLET,
    {F4, HALF_TIME, 0, C4_note+WHITE_CNT}, // outside C
    {F3, QUARTER_TIME, 0, C4_note}, 

    {F3, TRIPLET_TIME, 0, C4_note-WHITE_CNT}, 
    {F3, TRIPLET_TIME, 0, C4_note-WHITE_CNT*2}, 
    {F4, TRIPLET_TIME, 0, G3_note}, 
    
    {F1|F2|F3, HALF_TIME, 0, G3_note}, 

    {F3, TRIPLET_TIME, 0, C4_note},
    {F3, TRIPLET_TIME, 0, C4_note},
    {F3, TRIPLET_TIME, 0, C4_note},
    {F4, QUARTER_TIME, 0, C4_note+WHITE_CNT},
    {F1, TRIPLET_TIME, 0, C4_note},
    {F1, TRIPLET_TIME, 0, C4_note},
    {F1, TRIPLET_TIME, 0, C4_note},
    {F1, HALF_TIME, 0, C4_note}
};

static const BaseEvent Force_Theme[] {

}


static const BaseEvent *Songs[] = {
    Main_Theme,
    Force_Theme
}

/* ------------------------------------------------------------------ */
void NotePlayer_Init(uint8_t song_ID)
{
    if (song_ID > sizeof(Songs) / sizeof(int)) song_ID = 0;
    else {
        g_notePlayer.song           = Songs[song_ID];
        g_notePlayer.song_length    = sizeof(Songs[song_ID]) / sizeof(Songs[song_ID][0]);
    }
    
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
            state->current_index = 0U;
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
                if (event->fingers & (1U << i)) {
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