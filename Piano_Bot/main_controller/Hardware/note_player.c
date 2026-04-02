#include "app_pid.h"
#include "motor.h"
#include "piano_robot_config.h"
#include "main.h"
#include "solenoid.h"
#include "note_player.h"
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#define NOTE_DELAY_MS 1000

static NotePlayerState g_notePlayer;

static const BaseEvent eventArray[] = {
    {EVENT_SINGLE, {true, false, false, false, false}, 500, 1000, 0},
    {EVENT_SINGLE, {false, true, false, false, false}, 1000, 1000, 700},
    {EVENT_DOUBLE, {true, true, false, false, false}, 1500, 1000, 1500},
};

/* ------------------------------------------------------------------ */
void NotePlayer_Init(void)
{
    g_notePlayer.song = eventArray;
    g_notePlayer.song_length = sizeof(eventArray) / sizeof(eventArray[0]);
    g_notePlayer.current_index = 0;
    g_notePlayer.songelapsed_ms = 0;
    g_notePlayer.noteplayed_ms = 0;
    g_notePlayer.finished = false;
    g_notePlayer.note_status = false;
    g_notePlayer.song_started = false;
    g_notePlayer.carriage_in_position = false;
}

/* ------------------------------------------------------------------ */
void NotePlayer_Run(NotePlayerState *state, PID *pid, const Encoder_t *encoder)
{   

    int solenoidReadyCounter = 0;
    uint32_t currTime = HAL_GetTick();

    if (!state->song_started) {
        state->songStartTime_ms = HAL_GetTick();
        state->song_started = true;
    }

    if (state->current_index >= state->song_length) {
        state->finished = true;
        return;
    }

    if (state->note_status == false) 
    {   

        if (state->carriage_in_position == false) 
        {
            Motor_SetTarget(state->song[state->current_index].target_position);
            if (Motor_AtTarget(encoder)) 
            {
                state->carriage_in_position = true;
            }
        }

        else 
        {

            // Trigger solenoids based on the event's solenoid_index array
            for (int i = 0; i < SOLENOID_COUNT; i++) 
            {
                if (state->song[state->current_index].solenoid_index[i]) 
                {
                    Solenoid_Strike(i, state->song[state->current_index].duration_ms);
                }
            }
            state->note_status = true;               // Mark note as played
            state->noteplayed_ms = HAL_GetTick();    // Record when the note was played
        }

    }
    else 
    {
        for (int i = 0; i < SOLENOID_COUNT; i++) 
        {
            if (Solenoid_IsReady(i)) 
            {
                solenoidReadyCounter++;
            }
        }
    }

    if (solenoidReadyCounter == SOLENOID_COUNT 
            && (currTime - state->noteplayed_ms >= 
                (state->song[state->current_index].duration_ms + NOTE_DELAY_MS))
            && state->carriage_in_position)
    {       
            app_pid_requestReset(pid);
            state->current_index++;              // Go to the next event
            state->carriage_in_position = false; // Reset for next note
            state->note_status = false;          
    }
}
/* ------------------------------------------------------------------ */
NotePlayerState *NotePlayer_GetState(void)
{
    return &g_notePlayer;
}
