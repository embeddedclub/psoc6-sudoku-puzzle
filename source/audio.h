/*
 * audio.h
 *
 *  Created on: 02-Jul-2021
 *      Author: Embedded Club
 */

#ifndef SOURCE_AUDIO_H_
#define SOURCE_AUDIO_H_

/*******************************************************************************
 * Header file includes
 ******************************************************************************/
#include <stdint.h>


void play_tone(uint8_t index);
void audio_clock_init(void);
void task_audio(void* param);

extern TaskHandle_t audio_task_handle;

#endif /* SOURCE_AUDIO_H_ */
