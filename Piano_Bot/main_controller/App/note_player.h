/*
    NOTE-PLAYER.C SHOULD HANDLE
    current song pointer
    current note index
    whether playback has started
    current song time
    whether the current note has already been triggered
    whether the carriage is in position
    whether playback is finished
*/

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "motor.h"
#include "solenoid.h"
#include "piano_robot_config.h"

/* ============================================================
 *  song.h  —  PLaying the chords from song.c
 * ============================================================ */

/* Event types */
typedef enum {
    EVENT_SINGLE    = 1,
    EVENT_DOUBLE,
    EVENT_CHORD,
} EventType;


typedef uint8_t FingerMask;
enum {
    FINGER_0 = 1u << 0,
    FINGER_1 = 1u << 1,
    FINGER_2 = 1u << 2,
    FINGER_3 = 1u << 3,
    FINGER_4 = 1u << 4
};

/* Base event */
typedef struct {
    FingerMask fingers;          // Which solenoids to actuate
    uint16_t duration_ms;       // How long to hold the note for (solenoids down)
    uint32_t time_to_next_ms;   // delay untill the next note should start (motor is allowed to move during this period, solenoids up)
    uint32_t target_position;   // Motor position
} BaseEvent;

/* state machine states */
typedef enum
{
    NOTE_PLAYER_STATE_IDLE = 0,
    NOTE_PLAYER_STATE_MOVE_MOTOR,
    NOTE_PLAYER_STATE_WAIT_READY,
    NOTE_PLAYER_STATE_STRIKE_NOTE,
    NOTE_PLAYER_STATE_WAIT_NOTE_COMPLETE,
    NOTE_PLAYER_STATE_FINISHED
} NotePlayerState_t;

/* Player state */
typedef struct {
    const BaseEvent *song;      // Pointing to entire song data
    uint16_t            song_length;       // How many events (chords or singlets) are in the song
    uint16_t            current_index;     // Where we currently are in the song
    uint32_t            noteStart_ms;
    NotePlayerState_t   run_state;
} NotePlayer_t;

/* Load in the song */
void NotePlayer_Init(void);

/* Playing the events (notes), and when it's time to move to the next event, release the solenoids and move forward */
void NotePlayer_Run(NotePlayer_t *notePlayer, PID *pid, const Encoder_t *encoder);

/* Used for returning the global Note Playing state*/
NotePlayer_t *NotePlayer_GetState(void);

bool NotePlayer_IsDone(void);