/**
 * @file beep.h
 * @brief Piezo buzzer driver (on/off, timed pulse), driven via GPIO.
 *
 * Used both directly by the low-battery warning (see timer_battery_callback()
 * in main.c) and via the `/beep` micro-ROS topic (see beep_callback()).
 */
#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"

/** GPIO pin the piezo buzzer is wired to. */
#define BEEP_GPIO           46

// 蜂鸣器有效电平，0=低电平响，1=高电平响
/** Buzzer active level: 0 = active low, 1 = active high. */
#define BEEP_ACTIVE_LEVEL  1

/** @name Beep_Handle() state machine states.
 *  @{
 */
#define BEEP_STATE_OFF       0 /**< Buzzer is off. */
#define BEEP_STATE_ON_ALWAYS 1 /**< Buzzer is on continuously, until Beep_Off() is called. */
#define BEEP_STATE_ON_DELAY  2 /**< Buzzer is on for a timed pulse, auto-off when beep_on_time reaches 0. */
/** @} */

/** @name Convenience aliases for Beep_On()/Beep_Off(). */
#define BEEP_ON()       Beep_On()
#define BEEP_OFF()      Beep_Off()

/**
 * @brief Configure the buzzer GPIO and start the background handler task.
 *
 * Must be called once during startup before any other Beep_*() function.
 */
void Beep_Init(void);

/** @brief Turn the buzzer on continuously (state = BEEP_STATE_ON_ALWAYS). */
void Beep_On(void);

/** @brief Turn the buzzer off immediately (state = BEEP_STATE_OFF). */
void Beep_Off(void);

/**
 * @brief Turn the buzzer on for a specific duration, or set it on/off directly.
 * @param time `0` turns the buzzer off; `1` turns it on continuously;
 *             `10`-`10000` sounds a pulse for that many milliseconds, after
 *             which Beep_Handle() automatically turns it off again (values
 *             are clamped to this range).
 */
void Beep_On_Time(uint16_t time);

/**
 * @brief Buzzer auto-off state machine tick, called every 10ms by Beep_Task().
 *
 * Counts down the remaining time while in BEEP_STATE_ON_DELAY and turns the
 * buzzer off once it reaches zero. No effect in BEEP_STATE_OFF/ON_ALWAYS.
 */
void Beep_Handle(void);

#ifdef __cplusplus
}
#endif
