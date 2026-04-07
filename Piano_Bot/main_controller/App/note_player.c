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


#define BPM1 91
#define BPM2 76

#define WHOLE_TIME(bpm)     (240000 / bpm)     // 2000 at 120bpm
#define HALF_TIME(bpm)      (120000 / bpm)      // 1000 at 120bpm
#define QUARTER_TIME(bpm)    (60000 / bpm)    // 500 at 120bpm
#define EIGHTH_TIME(bpm)     (30000 / bpm)      // 250 at 120bpm
#define TRIPLET_TIME(bpm)    (20000 / bpm)      // 250 at 120bpm

#define FED_TRIPLET \
    {F2, TRIPLET_TIME(BPM1), 0, C4_note+WHITE_CNT}, \
    {F2, TRIPLET_TIME(BPM1), 0, C4_note}, \
    {F1, TRIPLET_TIME(BPM1), 0, C4_note+WHITE_CNT}

#define MELODY \
    {F4, EIGHTH_TIME(BPM1), 0, G3_note}, \
    {F4, EIGHTH_TIME(BPM1), 0, G3_note-WHITE_CNT},  \
    {F3, EIGHTH_TIME(BPM1), 0, G3_note},\
    {F3, EIGHTH_TIME(BPM1), 0, G3_note-WHITE_CNT}, \
    {F2|F3, TRIPLET_TIME(BPM1), 0, G3_note-WHITE_CNT}, \
    {F3, TRIPLET_TIME(BPM1), 0, G3_note},\
    {F4, TRIPLET_TIME(BPM1), 0, G3_note-WHITE_CNT}, \
    {F3, QUARTER_TIME(BPM1), 0, G3_note}

static NotePlayer_t g_notePlayer;
uint32_t note_delay_ms = 0;

static const BaseEvent Main_Theme[] = { 
    {F1, TRIPLET_TIME(BPM1), 0, G3_note},
    {F1, TRIPLET_TIME(BPM1), 0, G3_note},
    {F1, TRIPLET_TIME(BPM1), 0, G3_note},

    {F1, HALF_TIME(BPM1), 0, C4_note},
    {F3, HALF_TIME(BPM1), 0, C4_note},
    
    FED_TRIPLET,
    {F4, HALF_TIME(BPM1), 0, C4_note+WHITE_CNT}, // outside C
    {F3, QUARTER_TIME(BPM1), 0, C4_note}, 

    FED_TRIPLET,
    {F4, HALF_TIME(BPM1), 0, C4_note+WHITE_CNT}, // outside C
    {F3, QUARTER_TIME(BPM1), 0, C4_note}, 
    
    {F3, TRIPLET_TIME(BPM1), 0, C4_note-WHITE_CNT}, 
    {F3, TRIPLET_TIME(BPM1), 0, C4_note-WHITE_CNT*2}, 
    {F4, TRIPLET_TIME(BPM1), 0, G3_note}, 
    
    {F1|F2|F3, HALF_TIME(BPM1), 0, G3_note}, 

    {F1, EIGHTH_TIME(BPM1), 0, G3_note}, 
    {F1, EIGHTH_TIME(BPM1), 0, G3_note}, 

    /* repeat above*/
    
    {F1, HALF_TIME(BPM1),0, C4_note}, // C
    {F3, HALF_TIME(BPM1),0, C4_note}, // G

    FED_TRIPLET,
    {F4, HALF_TIME(BPM1),0, C4_note+WHITE_CNT}, // outside C
    {F3, QUARTER_TIME(BPM1),0, C4_note}, 

    FED_TRIPLET,
    {F4, HALF_TIME(BPM1),0, C4_note+WHITE_CNT}, // outside C
    {F3, QUARTER_TIME(BPM1),0, C4_note}, 

    {F3, TRIPLET_TIME(BPM1),0, C4_note-WHITE_CNT}, 
    {F3, TRIPLET_TIME(BPM1),0, C4_note-WHITE_CNT*2}, 
    {F4, TRIPLET_TIME(BPM1),0, G3_note}, 
    
    {F1|F2|F3, HALF_TIME(BPM1),0, G3_note}, 

    {F1, EIGHTH_TIME(BPM1),0, G3_note}, // g
    {F1, EIGHTH_TIME(BPM1),0, G3_note}, // g

    /* end repeat */

    {F2, QUARTER_TIME(BPM1)+EIGHTH_TIME(BPM1),0, G3_note-WHITE_CNT}, // A
    {F2, EIGHTH_TIME(BPM1),0, G3_note-WHITE_CNT}, // A

    MELODY,
    
    {F2, QUARTER_TIME(BPM1),0, G3_note}, // B
    {F1, EIGHTH_TIME(BPM1),0, G3_note}, // G
    {F1, EIGHTH_TIME(BPM1),0, G3_note}, // G
    {F1, QUARTER_TIME(BPM1)+EIGHTH_TIME(BPM1),0, G3_note+WHITE_CNT}, // A
    {F1, EIGHTH_TIME(BPM1),0, G3_note+WHITE_CNT}, // A

    {F3, EIGHTH_TIME(BPM1),0, C4_note-WHITE_CNT}, // F
    {F3, EIGHTH_TIME(BPM1),0, C4_note-WHITE_CNT*2}, // E
    {F2, EIGHTH_TIME(BPM1),0, C4_note-WHITE_CNT}, // D

    {F2, EIGHTH_TIME(BPM1),0, C4_note-WHITE_CNT*2}, // C
    {FB|F4, QUARTER_TIME(BPM1),0, C4_note-WHITE_CNT*2}, // B sharp g

    {F1|F2|F3, HALF_TIME(BPM1),0, G3_note}, // chord @37s

    /* repeat */

    {F1, EIGHTH_TIME(BPM1),0, G3_note}, // g
    {F1, EIGHTH_TIME(BPM1),0, G3_note}, // g

    {F1, QUARTER_TIME(BPM1)+EIGHTH_TIME(BPM1),0, G3_note+WHITE_CNT}, // A
    {F1, EIGHTH_TIME(BPM1),0, G3_note+WHITE_CNT}, // A

    MELODY,

    {F2, QUARTER_TIME(BPM1),0, G3_note}, // B

    {F2, EIGHTH_TIME(BPM1),0, C4_note+WHITE_CNT*2}, // G
    {F2, EIGHTH_TIME(BPM1),0, C4_note+WHITE_CNT*2}, // G
    {F1|F3, QUARTER_TIME(BPM1),0, C4_note+WHITE_CNT*3}, // F|C

    {F2, EIGHTH_TIME(BPM1),0, G4_note}, // B sharp // manual because we ran out
    {FB, EIGHTH_TIME(BPM1),0, G4_note}, // A sharp
    {F1, EIGHTH_TIME(BPM1),0, G4_note}, // G 
    {F2, EIGHTH_TIME(BPM1),0, C4_note+WHITE_CNT}, // F
    {FB, EIGHTH_TIME(BPM1),0, C4_note+WHITE_CNT}, // E sharp
    {F1, EIGHTH_TIME(BPM1),0, C4_note+WHITE_CNT}, // D
    {F1, EIGHTH_TIME(BPM1),0, C4_note}, // C
    {F3, HALF_TIME(BPM1),0, C4_note}, // G

    {F1, TRIPLET_TIME(BPM1),0, G3_note}, // G
    {F1, TRIPLET_TIME(BPM1),0, G3_note}, // G
    {F1, TRIPLET_TIME(BPM1),0, G3_note}, // G
    {F1, QUARTER_TIME(BPM1),0, G3_note}, // G
    {F1, TRIPLET_TIME(BPM1),0, G3_note}, // G
    {F1, TRIPLET_TIME(BPM1),0, G3_note}, // G
    {F1, TRIPLET_TIME(BPM1),0, G3_note}, // G

    {F1, HALF_TIME(BPM1),0, C4_note}, // C
    {F3, HALF_TIME(BPM1),0, C4_note}, // G

    FED_TRIPLET,
    {F4, HALF_TIME(BPM1),0, C4_note+WHITE_CNT}, // outside C
    {F3, QUARTER_TIME(BPM1),0, C4_note}, 

    FED_TRIPLET,
    {F4, HALF_TIME(BPM1),0, C4_note+WHITE_CNT}, // outside C
    {F3, QUARTER_TIME(BPM1),0, C4_note}, 

    {F3, TRIPLET_TIME(BPM1),0, C4_note-WHITE_CNT}, 
    {F3, TRIPLET_TIME(BPM1),0, C4_note-WHITE_CNT*2}, 
    {F4, TRIPLET_TIME(BPM1),0, G3_note}, 
    
    {F1|F2|F3, HALF_TIME(BPM1),0, G3_note}, 
    {F1, EIGHTH_TIME(BPM1),0, G3_note}, 
    {F1, EIGHTH_TIME(BPM1),0, G3_note}, 

    {F1, HALF_TIME(BPM1),0, C4_note}, // C
    {F3, HALF_TIME(BPM1),0, C4_note}, // G

    FED_TRIPLET,
    {F4, HALF_TIME(BPM1),0, C4_note+WHITE_CNT}, // outside C
    {F3, QUARTER_TIME(BPM1),0, C4_note}, 

    FED_TRIPLET,
    {F4, HALF_TIME(BPM1),0, C4_note+WHITE_CNT}, // outside C
    {F3, QUARTER_TIME(BPM1),0, C4_note}, 

    {F3, TRIPLET_TIME(BPM1),0, C4_note-WHITE_CNT}, 
    {F3, TRIPLET_TIME(BPM1),0, C4_note-WHITE_CNT*2}, 
    {F4, TRIPLET_TIME(BPM1),0, G3_note}, 
    
    {F1|F2|F3, HALF_TIME(BPM1),0, G3_note}, 

    {F3, TRIPLET_TIME(BPM1),0, C4_note},
    {F3, TRIPLET_TIME(BPM1),0, C4_note},
    {F3, TRIPLET_TIME(BPM1),0, C4_note},
    {F4, QUARTER_TIME(BPM1),0, C4_note+WHITE_CNT},
    {F1, TRIPLET_TIME(BPM1),0, C4_note},
    {F1, TRIPLET_TIME(BPM1),0, C4_note},
    {F1, TRIPLET_TIME(BPM1),0, C4_note},
    {F1, HALF_TIME(BPM1),0, C4_note}
};


#define EGB_TRIPLET \
    {F1, TRIPLET_TIME(BPM2), 0, E3_note}, \
    {F2, TRIPLET_TIME(BPM2), 0, E3_note}, \
    {F3, TRIPLET_TIME(BPM2), 0, E3_note}

#define DFA_TRIPLET \
    {F1, TRIPLET_TIME(BPM2), 0, C3_note+WHITE_CNT}, \
    {F2, TRIPLET_TIME(BPM2), 0, C3_note+WHITE_CNT}, \
    {F3, TRIPLET_TIME(BPM2), 0, C3_note+WHITE_CNT}

static const BaseEvent Force_Theme[] = {
    EGB_TRIPLET,
    EGB_TRIPLET,
    EGB_TRIPLET,
    {FB, QUARTER_TIME(BPM2), 0, E3_note+WHITE_CNT},
    EGB_TRIPLET,
    EGB_TRIPLET,
    EGB_TRIPLET,
    {FB, QUARTER_TIME(BPM2), 0, E3_note+WHITE_CNT},
    EGB_TRIPLET,
    EGB_TRIPLET,
    EGB_TRIPLET,
    {FB, QUARTER_TIME(BPM2), 0, E3_note+WHITE_CNT},
    EGB_TRIPLET,
    EGB_TRIPLET, /* 16s*/

    {F1, QUARTER_TIME(BPM2), 0, E3_note},
    {F3, QUARTER_TIME(BPM2), 0, E3_note},
    {F2, HALF_TIME(BPM2), 0, E4_note}, 
    {F1, QUARTER_TIME(BPM2), 0, E4_note}, 

    {F2, TRIPLET_TIME(BPM2), 0, E4_note+WHITE_CNT}, // a 
    {F2, TRIPLET_TIME(BPM2), 0, E4_note}, // g
    {FB, TRIPLET_TIME(BPM2), 0, E4_note+WHITE_CNT}, // f#
    {F2, QUARTER_TIME(BPM2), 0, E4_note}, // g
    {F1, QUARTER_TIME(BPM2), 0, E4_note}, // E
    
    {F2, TRIPLET_TIME(BPM2), 0, E4_note}, // g
    {FB, TRIPLET_TIME(BPM2), 0, E4_note+WHITE_CNT}, // f#
    {F1, TRIPLET_TIME(BPM2), 0, E4_note}, // e
    {FB, QUARTER_TIME(BPM2), 0, E4_note+WHITE_CNT}, // f#
    
    {F2, QUARTER_TIME(BPM2), 0, C4_note-WHITE_CNT}, // d
    {F2, HALF_TIME(BPM2), 0, C4_note}, // E
    {F2, QUARTER_TIME(BPM2), 0, C4_note-WHITE_CNT}, // d
    {F1, HALF_TIME(BPM2), 0, C4_note-WHITE_CNT}, // B
    {F1, QUARTER_TIME(BPM2), 0, C4_note-WHITE_CNT}, // B
    {F3, HALF_TIME(BPM2), 0, C4_note}, // G
    {F2, HALF_TIME(BPM2), 0, C4_note}, // E
    
    {F2, TRIPLET_TIME(BPM2), 0, C4_note}, // E
    {FB, TRIPLET_TIME(BPM2), 0, E4_note+WHITE_CNT}, // F#
    {F2, TRIPLET_TIME(BPM2), 0, E4_note}, // G
    {F2, QUARTER_TIME(BPM2), 0, E4_note+WHITE_CNT}, // A
    {FB, QUARTER_TIME(BPM2), 0, E4_note+WHITE_CNT}, // F#
    {F2, QUARTER_TIME(BPM2), 0, G4_note}, // B
    {F2, QUARTER_TIME(BPM2), 0, E4_note+WHITE_CNT}, // A
    {F3, QUARTER_TIME(BPM2), 0, E4_note}, // B
    
    {F2, QUARTER_TIME(BPM2), 0, G3_note}, // B @38s
    {F3, HALF_TIME(BPM2), 0, G3_note}, // D
    {F3, EIGHTH_TIME(BPM2), 0, G3_note-WHITE_CNT}, // C
    {F2, EIGHTH_TIME(BPM2), 0, G3_note}, // B
    {F2, EIGHTH_TIME(BPM2), 0, G3_note-WHITE_CNT}, // A
    {F2, TRIPLET_TIME(BPM2), 0, G3_note-WHITE_CNT}, // A
    {F2, TRIPLET_TIME(BPM2), 0, G3_note}, // B
    {F3, TRIPLET_TIME(BPM2), 0, G3_note-WHITE_CNT}, // C
    {F3, QUARTER_TIME(BPM2), 0, G3_note-WHITE_CNT}, // C
    {F2, QUARTER_TIME(BPM2), 0, G3_note-WHITE_CNT}, // A
    
    {F3, HALF_TIME(BPM2), 0, G3_note}, // D
    {F3, EIGHTH_TIME(BPM2), 0, G3_note-WHITE_CNT}, // C
    {F2, EIGHTH_TIME(BPM2), 0, G3_note}, // B
    {F1, EIGHTH_TIME(BPM2), 0, G3_note+WHITE_CNT}, // A
    {FB, EIGHTH_TIME(BPM2), 0, G3_note}, // Ab
    {F2, EIGHTH_TIME(BPM2), 0, G3_note}, // B 
    {F3, QUARTER_TIME(BPM2), 0, G3_note+WHITE_CNT}, // E 
    {F3, QUARTER_TIME(BPM2), 0, G3_note+WHITE_CNT}, // E 
    {F4, HALF_TIME(BPM2), 0, G3_note+WHITE_CNT}, // G

    {F3, EIGHTH_TIME(BPM2), 0, C4_note-WHITE_CNT}, // F
    {F3, EIGHTH_TIME(BPM2), 0, C4_note-WHITE_CNT*2}, // E
    {F2, EIGHTH_TIME(BPM2), 0, C4_note-WHITE_CNT}, // D
    {F2, TRIPLET_TIME(BPM2), 0, C4_note-WHITE_CNT}, // D
    {F3, TRIPLET_TIME(BPM2), 0, C4_note-WHITE_CNT*2}, // E
    {F3, TRIPLET_TIME(BPM2), 0, C4_note-WHITE_CNT}, // F
    
    {F3, QUARTER_TIME(BPM2), 0, C4_note-WHITE_CNT}, // F
    {F2, QUARTER_TIME(BPM2), 0, C4_note-WHITE_CNT}, // D
    {F3, QUARTER_TIME(BPM2), 0, C4_note}, // G
    {F3, QUARTER_TIME(BPM2), 0, C4_note-WHITE_CNT}, // F
    {F2, QUARTER_TIME(BPM2), 0, C4_note}, // E
    {F1, QUARTER_TIME(BPM2), 0, C4_note+WHITE_CNT}, // D
    {FB, QUARTER_TIME(BPM2), 0, C4_note}, // C#
    {F2, QUARTER_TIME(BPM2), 0, C4_note}, // E
    {F3, WHOLE_TIME(BPM2), 0, C4_note+WHITE_CNT}, // A

    DFA_TRIPLET,
    DFA_TRIPLET,
    DFA_TRIPLET,
    DFA_TRIPLET,
    DFA_TRIPLET,
    DFA_TRIPLET,
    DFA_TRIPLET,
    DFA_TRIPLET,
    DFA_TRIPLET,
    DFA_TRIPLET,

    {F1, QUARTER_TIME(BPM2), 0, C3_note+WHITE_CNT}, // D @1:04
    {F3, QUARTER_TIME(BPM2), 0, C3_note+WHITE_CNT}, // A
    {F3, HALF_TIME(BPM2), 0, C4_note-WHITE_CNT}, // F
    {F2, HALF_TIME(BPM2), 0, C4_note-WHITE_CNT}, // D
    
    {F2, QUARTER_TIME(BPM2), 0, C4_note-WHITE_CNT}, // D
    {F3, TRIPLET_TIME(BPM2), 0, C4_note}, // G
    {F3, TRIPLET_TIME(BPM2), 0, C4_note-WHITE_CNT}, // F
    {F2, TRIPLET_TIME(BPM2), 0, C4_note}, // E
    {F3, QUARTER_TIME(BPM2), 0, C4_note-WHITE_CNT}, // F
    {F2, QUARTER_TIME(BPM2), 0, C4_note-WHITE_CNT}, // D
    
    {F3, TRIPLET_TIME(BPM2), 0, C4_note-WHITE_CNT}, // F
    {F2, TRIPLET_TIME(BPM2), 0, C4_note}, //  E
    {F2, TRIPLET_TIME(BPM2), 0, C4_note-WHITE_CNT}, // D
    {F2, QUARTER_TIME(BPM2), 0, C4_note}, //  E
    {F1, QUARTER_TIME(BPM2), 0, C4_note}, //  C
    {F2, HALF_TIME(BPM2), 0, C4_note-WHITE_CNT}, // D
    {F2, QUARTER_TIME(BPM2), 0, C4_note-WHITE_CNT*2}, //C
    {F1, HALF_TIME(BPM2), 0, C4_note-WHITE_CNT*2}, //A
    {F1, QUARTER_TIME(BPM2), 0, C4_note-WHITE_CNT*2}, //A
    {F3, HALF_TIME(BPM2), 0, C4_note-WHITE_CNT}, //F
    
    {F2, QUARTER_TIME(BPM2), 0, C4_note-WHITE_CNT}, // D @1:18
    {F2, TRIPLET_TIME(BPM2), 0, C4_note-WHITE_CNT}, // D 
    {F2, TRIPLET_TIME(BPM2), 0, C4_note}, // E
    {F2, TRIPLET_TIME(BPM2), 0, C4_note+WHITE_CNT}, // F
    {F3, QUARTER_TIME(BPM2), 0, C4_note}, // G
    {F2, QUARTER_TIME(BPM2), 0, C4_note}, // E
    {F3, QUARTER_TIME(BPM2), 0, C4_note+WHITE_CNT}, // A
    {F3, HALF_TIME(BPM2), 0, C4_note}, // G
    {F3, HALF_TIME(BPM2), 0, C4_note+WHITE_CNT}, // A
    {F1, QUARTER_TIME(BPM2), 0, C4_note+WHITE_CNT}, // D

    {F4, HALF_TIME(BPM2), 0, G3_note+WHITE_CNT}, // G
    {F3, EIGHTH_TIME(BPM2), 0, C4_note-WHITE_CNT}, // F
    {F3, EIGHTH_TIME(BPM2), 0, C4_note-WHITE_CNT*2}, // E
    {F2, EIGHTH_TIME(BPM2), 0, C4_note-WHITE_CNT}, // D
    {F2, TRIPLET_TIME(BPM2), 0, C4_note-WHITE_CNT}, // D
    {F3, TRIPLET_TIME(BPM2), 0, C4_note-WHITE_CNT*2}, // E
    {F3, TRIPLET_TIME(BPM2), 0, C4_note-WHITE_CNT}, // F
    {F3, QUARTER_TIME(BPM2), 0, C4_note-WHITE_CNT}, // F

    {F1, QUARTER_TIME(BPM2), 0, C4_note+WHITE_CNT}, // D
    {F3, HALF_TIME(BPM2), 0, C4_note}, // G
    {F3, EIGHTH_TIME(BPM2), 0, C4_note-WHITE_CNT}, // F
    {F2, EIGHTH_TIME(BPM2), 0, C4_note}, // E
    {F1, EIGHTH_TIME(BPM2), 0, C4_note+WHITE_CNT}, // D
    {FB, EIGHTH_TIME(BPM2), 0, C4_note}, // C#
    {F2, EIGHTH_TIME(BPM2), 0, C4_note}, // E
    {F3, QUARTER_TIME(BPM2), 0, C4_note+WHITE_CNT}, // A4
    
    {F1, QUARTER_TIME(BPM2), 0, C4_note-WHITE_CNT*2}, // A3 @1:34
    {F2, QUARTER_TIME(BPM2), 0, C4_note-WHITE_CNT*2}, // C
    {FB, EIGHTH_TIME(BPM2), 0, C4_note-WHITE_CNT*2}, // Bb
    {F1, EIGHTH_TIME(BPM2), 0, C4_note-WHITE_CNT*2}, // A
    {F1, EIGHTH_TIME(BPM2), 0, G3_note}, // G
    {F1, TRIPLET_TIME(BPM2), 0, G3_note}, // G
    {F1, TRIPLET_TIME(BPM2), 0, G3_note+WHITE_CNT}, // A
    {FB, TRIPLET_TIME(BPM2), 0, G3_note+WHITE_CNT}, // Bb
    {FB, QUARTER_TIME(BPM2), 0, G3_note+WHITE_CNT}, // Bb
    {F1, QUARTER_TIME(BPM2), 0, G3_note}, // G
    
    {F2, HALF_TIME(BPM2), 0, G3_note+WHITE_CNT}, // C @1:40
    {FB, EIGHTH_TIME(BPM2), 0, G3_note+WHITE_CNT}, // Bb
    {F1, EIGHTH_TIME(BPM2), 0, G3_note+WHITE_CNT}, // A
    {F1, EIGHTH_TIME(BPM2), 0, G3_note}, // G
    {FB, EIGHTH_TIME(BPM2), 0, G3_note-WHITE_CNT}, // Gb
    {F2, EIGHTH_TIME(BPM2), 0, G3_note-WHITE_CNT}, // A
    {F3, QUARTER_TIME(BPM2), 0, G3_note}, // D
    {F3, QUARTER_TIME(BPM2), 0, G3_note}, // D

    {F2, WHOLE_TIME(BPM2), 0, G4_note}, // Bb @1:44 
    {F1, QUARTER_TIME(BPM2), 0, G4_note}, // G
    {F3, TRIPLET_TIME(BPM2), 0, G4_note-WHITE_CNT}, // C 
    {F2, TRIPLET_TIME(BPM2), 0, G4_note}, // b 
    {F2, TRIPLET_TIME(BPM2), 0, G4_note-WHITE_CNT}, // A 
    {F2, QUARTER_TIME(BPM2), 0, G4_note}, // Bb
    {F1, QUARTER_TIME(BPM2), 0, G4_note}, // G
    {F2, TRIPLET_TIME(BPM2), 0, G4_note}, // Bb
    {F2, TRIPLET_TIME(BPM2), 0, G4_note-WHITE_CNT}, // A
    {F1, TRIPLET_TIME(BPM2), 0, G4_note}, // G
    {F2, TRIPLET_TIME(BPM2), 0, G4_note-WHITE_CNT}, // A
    {F2, QUARTER_TIME(BPM2), 0, C4_note+WHITE_CNT}, // F
    {F3, HALF_TIME(BPM2), 0, C4_note}, // G
    {F3, QUARTER_TIME(BPM2), 0, C4_note-WHITE_CNT}, // F
    {F2, HALF_TIME(BPM2), 0, C4_note-WHITE_CNT}, // D

    {F1, QUARTER_TIME(BPM2), 0, C3_note}, // D @1:54
    {FB, HALF_TIME(BPM2), 0, G3_note + WHITE_CNT}, // Bb
    {F1, QUARTER_TIME(BPM2), 0, G3_note}, // G
    {F1, TRIPLET_TIME(BPM2), 0, G3_note}, // G
    {F1, TRIPLET_TIME(BPM2), 0, G3_note+WHITE_CNT}, // A
    {FB, TRIPLET_TIME(BPM2), 0, G3_note+WHITE_CNT}, // Bb
    {F2, QUARTER_TIME(BPM2), 0, G3_note+WHITE_CNT}, // C
    {F1, QUARTER_TIME(BPM2), 0, G3_note+WHITE_CNT}, // A
    {F1, QUARTER_TIME(BPM2), 0, G3_note}, // G
    {F3, HALF_TIME(BPM2), 0, G3_note}, // D
    {FB, QUARTER_TIME(BPM2), 0, E3_note+WHITE_CNT}, // Gb
    {F3, HALF_TIME(BPM2), 0, G3_note}, // D
    {F1, HALF_TIME(BPM2), HALF_TIME(BPM2), G3_note}, // G
    
    {F1, QUARTER_TIME(BPM2), 0, C3_note+WHITE_CNT}, // D @2:08
    {FB, HALF_TIME(BPM2), 0, G3_note+WHITE_CNT}, // Bb
    {F1, QUARTER_TIME(BPM2), 0, G3_note}, // G
    
    {F2, TRIPLET_TIME(BPM2), 0, C4_note-WHITE_CNT*2}, // C4
    {FB, TRIPLET_TIME(BPM2), 0, G3_note+WHITE_CNT}, // Bb
    {F1, TRIPLET_TIME(BPM2), 0, G3_note+WHITE_CNT}, // A
    {FB, QUARTER_TIME(BPM2), 0, G3_note+WHITE_CNT}, // Bb
    {F1, QUARTER_TIME(BPM2), 0, G3_note}, // G
    {FB, TRIPLET_TIME(BPM2), 0, G3_note+WHITE_CNT}, // Bb
    {F1, TRIPLET_TIME(BPM2), 0, G3_note+WHITE_CNT}, // A
    {F1, TRIPLET_TIME(BPM2), 0, G3_note}, // G
    {F1, QUARTER_TIME(BPM2), 0, G3_note+WHITE_CNT}, // A
    
    {F3, QUARTER_TIME(BPM2), 0, C4_note-WHITE_CNT}, // F
    {F4, WHOLE_TIME(BPM2), 0, C4_note-WHITE_CNT*2}, // G

};


static const Song songs[] = {
    { Main_Theme, sizeof(Main_Theme)/sizeof(Main_Theme[0]) },
    { Force_Theme, sizeof(Force_Theme)/sizeof(Force_Theme[0]) }
};

/* ------------------------------------------------------------------ */
void NotePlayer_Init(uint8_t song_ID)
{
    if (song_ID > NUM_SONGS-1) song_ID = 0;
    else {
        g_notePlayer.song        = songs[song_ID].data;
        g_notePlayer.song_length = songs[song_ID].length;
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