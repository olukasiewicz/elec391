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

/* ============================================================
 *  song.h  —  PLaying the chords from song.c
 * ============================================================ */

/* Event types */
typedef enum {
    EVENT_SINGLE    = 1,
    EVENT_DOUBLE,
    EVENT_CHORD,
} EventType;

/* Base event */
typedef struct {
    EventType type;
    bool solenoid_index[5];     // Which solenoid to actuate
    uint32_t onset_ms;          // Time at which the event should be played (relative to song start)
    uint16_t duration_ms;       // How long to hold the note for
    uint32_t target_position;   // Motor position
} BaseEvent;

/* State of note being played */
typedef struct {
    const BaseEvent *song;      // Pointing to entire song data
    uint16_t song_length;       // How many events (chords or singlets) are in the song
    uint16_t current_index;     // Where we currently are in the song
    uint32_t songelapsed_ms;    // How much time has passed since the start of the song
    uint32_t songStartTime_ms;  // When the song started
    uint32_t noteplayed_ms;     // When was the note played?
    bool finished;              // Prolly not needed
    bool note_status;           // Has the note been played?
    bool song_started;          // Has the song started?
    bool carriage_in_position;  // Is the carriage in position for the note to be played?
} NotePlayerState;

/* Load in the song */
void NotePlayer_Init(void);

/* Playing the events (notes), and when it's time to move to the next event, release the solenoids and move forward */
void NotePlayer_Run(NotePlayerState *state, PID *pid, const Encoder_t *encoder);

/* Used for returning the global Note Playing state*/
NotePlayerState *NotePlayer_GetState(void);