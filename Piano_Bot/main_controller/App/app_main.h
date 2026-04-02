#ifndef APP_MAIN_H
#define APP_MAIN_H
 
/* ============================================================
 *  main_app.h  —  Top-level application interface
 * ============================================================ */
 
#include <stdint.h>
#include <stdbool.h>
 
/* Call once after all MX_ peripheral inits in main.c */
void App_Init(void);
 
/* Call from while(1) main loop */
void App_Tick(void);
 
/* Call from high-rate timer ISR (CONTROL_LOOP_HZ) */
void App_ControlTick(void);
 
/* Set the control tick flag — call from TIM period-elapsed ISR */
void App_SetControlFlag(void);
 
// /* Directly queue a note to play (MIDI note number) */
// void App_PlayNote(uint8_t midi_note);
 
#endif /* APP_MAIN_H */