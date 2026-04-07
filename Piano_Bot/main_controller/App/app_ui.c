#include "app_ui.h"
#include "main.h"
#include "main.h"
#include "piano_robot_config.h"
#include "stm32f4xx_hal_gpio.h"
#include <stdatomic.h>
#include <stdint.h>

static uint8_t blinking = 0;
static uint8_t blink_count = 0;
static uint8_t target_blinks = 0;

static uint32_t last_toggle_time = 0;

static uint32_t last_press = 0;
static uint32_t last_auto_trigger = 0;

static uint8_t released = 1;

static uint8_t song_num = 1;

void UI_Update(void)
{
    uint32_t now = HAL_GetTick();

    // --- BUTTON TRIGGER ---
    if (HAL_GPIO_ReadPin(GPIO_BTN_GPIO_Port, GPIO_BTN_Pin) == GPIO_PIN_RESET)
    {
        if (now - last_press > DEBOUNCE_MS && released)
        {
            released = 0;
            last_press = now;
            song_num = (song_num % NUM_SONGS) + 1;

            //if (!blinking)
            //{
                blinking = 1;
                target_blinks = song_num * 2;
                blink_count = 0;
                last_toggle_time = now;

                last_auto_trigger = now; // reset timer so it doesn't immediately retrigger
            //}
        }
    } else{
        released = 1;
    }

    // // --- 5-SECOND AUTO TRIGGER ---
    // if (!blinking && (now - last_auto_trigger >= AUTO_BLINK_DELAY))
    // {
    //     last_auto_trigger = now;

    //     blinking = 1;
    //     target_blinks = song_num * 2;
    //     blink_count = 0;
    //     last_toggle_time = now;
    // }

    // --- NON-BLOCKING BLINK ---
    if (blinking)
    {
        if (now - last_toggle_time >= BLINK_DELAY)
        {
            last_toggle_time = now;

            HAL_GPIO_TogglePin(GPIO_LEFT_LED_GPIO_Port, GPIO_LEFT_LED_Pin);
            blink_count++;

            if (blink_count >= target_blinks)
            {
                blinking = 0;
                HAL_GPIO_WritePin(GPIO_LEFT_LED_GPIO_Port, GPIO_LEFT_LED_Pin, GPIO_PIN_RESET);
            }
        }
    }
}

uint8_t Get_SongID(void)
{
    return song_num-1;
}
