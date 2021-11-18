#include "main.h"
#include"music.h"

#define LIMIT(A, MIN, MAX) 						 ((A < MIN) ? (MIN) : ((A > MAX) ? (MAX) : (A)))
#define MUSIC_SAPMLING_TIME                      (int16_t)((1.0/((float)PWM_FREQUENCY)) * ((float)((int32_t)(INT16_MAX))))
#define _2_PI_S16                                (uint32_t)(UINT16_MAX)
#define _2_PI_U8                                 (uint16_t)(U8_MAX)

#define PRODUCT_MUSIC_AMPLITUDE                  (uint8_t)5 /* 0 to 7 levels */
#define MAX_MUSIC_AMPLITUDE                      10000
#define MUSIC_BUFF_TERMINATOR_INDEX              (uint8_t)0xFF
#define MUSIC_INFINITE_PERIODICITY_FLAG          (uint8_t)0xF
#define HAPPY_BIRTHDAY_TUNE_DURATION             (50)

uint8_t music_pattern1[31];
uint8_t music_pattern2[31];
uint8_t music_pattern3[31];
extern const int16_t hSin_Cos_Table[256];

int16_t Sin_Function(int16_t hAngle);

/* Tone Buffer index definition:
 * Index [0 % 3] represents Frequency 0.1 Hz.
 * Index [1 % 3] represents On Time in 0.01 second.
 * Index [2 % 3] represents Amplitude in (amplitude)^Index. -ve value means the amplitude should be zero for that tone.
 */
const uint8_t music_pattern_off[] = { 20, 5, 0, MUSIC_BUFF_TERMINATOR_INDEX};

const uint8_t happy_birthday_to_you[] = {
							40,HAPPY_BIRTHDAY_TUNE_DURATION,PRODUCT_MUSIC_AMPLITUDE,
							45,HAPPY_BIRTHDAY_TUNE_DURATION,PRODUCT_MUSIC_AMPLITUDE,
							40,HAPPY_BIRTHDAY_TUNE_DURATION,PRODUCT_MUSIC_AMPLITUDE,
							60,HAPPY_BIRTHDAY_TUNE_DURATION,PRODUCT_MUSIC_AMPLITUDE,
							55,HAPPY_BIRTHDAY_TUNE_DURATION,PRODUCT_MUSIC_AMPLITUDE,
							40,100,0,
							40,HAPPY_BIRTHDAY_TUNE_DURATION,PRODUCT_MUSIC_AMPLITUDE,
							45,HAPPY_BIRTHDAY_TUNE_DURATION,PRODUCT_MUSIC_AMPLITUDE,
							40,HAPPY_BIRTHDAY_TUNE_DURATION,PRODUCT_MUSIC_AMPLITUDE,
							65,HAPPY_BIRTHDAY_TUNE_DURATION,PRODUCT_MUSIC_AMPLITUDE,
							60,HAPPY_BIRTHDAY_TUNE_DURATION,PRODUCT_MUSIC_AMPLITUDE,
							40,100,0,
							40,HAPPY_BIRTHDAY_TUNE_DURATION,PRODUCT_MUSIC_AMPLITUDE,
							70,HAPPY_BIRTHDAY_TUNE_DURATION,PRODUCT_MUSIC_AMPLITUDE,
							65,HAPPY_BIRTHDAY_TUNE_DURATION,PRODUCT_MUSIC_AMPLITUDE,
							60,HAPPY_BIRTHDAY_TUNE_DURATION,PRODUCT_MUSIC_AMPLITUDE,
							50,HAPPY_BIRTHDAY_TUNE_DURATION,PRODUCT_MUSIC_AMPLITUDE,
							40,100,0,
							70,HAPPY_BIRTHDAY_TUNE_DURATION,PRODUCT_MUSIC_AMPLITUDE,
							60,HAPPY_BIRTHDAY_TUNE_DURATION,PRODUCT_MUSIC_AMPLITUDE,
							65,HAPPY_BIRTHDAY_TUNE_DURATION,PRODUCT_MUSIC_AMPLITUDE,
							60,HAPPY_BIRTHDAY_TUNE_DURATION,PRODUCT_MUSIC_AMPLITUDE,
							40,255,0,
							MUSIC_BUFF_TERMINATOR_INDEX};

int32_t music_init(MUSIC_PARAM* music_param)
{
	uint8_t i;
	for(i = 0; i < 30; i++)
	{
		if(i%3 == 0)
		{
			music_pattern1[i] = (2*i) + 10;
			music_pattern2[i] = (-2*i) + 100;
			music_pattern3[i] = (200 - (i * 2))/10;
		}
		else if(i%3 == 1)
		{
			music_pattern1[i] = (10 + i)/10;
			music_pattern2[i] = (7 + i)/10;
			music_pattern3[i] = (100 + i)/10;
		}
		else
		{
			music_pattern1[i] = (uint16_t)PRODUCT_MUSIC_AMPLITUDE;
			music_pattern2[i] = (uint16_t)PRODUCT_MUSIC_AMPLITUDE;
			music_pattern3[i] = (uint16_t)PRODUCT_MUSIC_AMPLITUDE;
		}
	}
	music_pattern1[i] = MUSIC_BUFF_TERMINATOR_INDEX;
	music_pattern2[i] = MUSIC_BUFF_TERMINATOR_INDEX;
	music_pattern3[i] = MUSIC_BUFF_TERMINATOR_INDEX;

	music_param->overriding_frequency_feature = 1;
	music_param->overriding_frequency = 500;
	//SET_BITS(GlobalData.CmdMap.device_state_h, MOTOR_MUSIC_BREAK);
	music_param->music_pattern = Music_OFF;

	return 1;
}

void set_music_command(MUSIC_PATTERNS cmd, MUSIC_PARAM* music_param)
{
	music_param->music_pattern = cmd;

	switch(music_param->music_pattern)
	{
	case Music1: /* Idle wait */
		music_param->music_buff = happy_birthday_to_you;
		music_param->music_perodicity = MUSIC_INFINITE_PERIODICITY_FLAG;
		music_param->traverse_direction = 0;
		music_param->buffer_size = (uint8_t)(sizeof(happy_birthday_to_you));
		music_param->traverse_direction_alternate = 0;
		break;
	case Music_OFF:
		music_param->music_buff = music_pattern_off;
		music_param->music_perodicity = 1;
		music_param->music_signal = 0;
		music_param->buff_counter = 0;
		music_param->cycle_counter = 0;
		music_param->peridicity_counter = 1;
		break;

	}
	 music_param->number_of_cycles = 0;
	 music_param->cycle_counter = 0;
	 music_param->buff_counter = 0;
	 music_param->peridicity_counter = 1;
}

void music_update(MUSIC_PARAM* music_param)
{
	int32_t theta;
	int16_t index;

	if(music_param->music_pattern != Music_OFF)
	{
		if(music_param->cycle_counter < music_param->number_of_cycles)
		{
			music_param->time_interval += MUSIC_SAPMLING_TIME;
			theta = (uint16_t)music_param->frequency * (uint16_t)music_param->time_interval * 10;
			theta *= _2_PI_S16;
			theta >>= 15;
			if(theta >= _2_PI_S16)
			{
				music_param->time_interval = 0;
				music_param->cycle_counter++;
			}
			if(music_param->overriding_frequency_feature)
			{
				if(music_param->overriding_frequency < 6000)
				{
					music_param->overriding_frequency += 1;
				}
				theta = music_param->time_interval;
				theta = ((((int32_t)_2_PI_S16 * (int32_t)music_param->overriding_frequency) * theta) >> 15);
				theta -= ((int32_t)INT16_MIN);
			}
			else
			{
				theta -= ((int32_t)INT16_MIN);
			}

			music_param->music_signal = LIMIT((((music_param->music_amplitude) * MCM_Trig_Functions((int16_t)theta).hSin) >> 5), -MAX_MUSIC_AMPLITUDE, MAX_MUSIC_AMPLITUDE);
		}
		else if(music_param->music_buff[music_param->buff_counter] != MUSIC_BUFF_TERMINATOR_INDEX)
		{
			music_param->cycle_counter = 0;
			if(!music_param->traverse_direction)
			{
				index = music_param->buff_counter;
			}
			else
			{
				index = music_param->buffer_size - (music_param->buff_counter + 4);
			}
			music_param->frequency = (music_param->music_buff[index++] + 20);
			music_param->on_time = music_param->music_buff[index++];
			music_param->music_amplitude = music_param->music_buff[index];
			music_param->number_of_cycles = ((uint16_t)music_param->frequency * (uint16_t)music_param->on_time)/10;
			music_param->buff_counter += 3;
			if(music_param->overriding_frequency_feature)
			{
				music_param->overriding_frequency = music_param->frequency * 10;
			}
		}
		else if(music_param->peridicity_counter < music_param->music_perodicity)
		{
			music_param->peridicity_counter++;
			if(music_param->music_perodicity == MUSIC_INFINITE_PERIODICITY_FLAG)
			{
				music_param->peridicity_counter = 0;
			}
			music_param->buff_counter = 0;
			music_param->cycle_counter = 0;
			if(music_param->traverse_direction_alternate)
			{
				music_param->traverse_direction += 1;
			}
		}
		else
		{
			music_param->music_signal = 0;
			music_param->buff_counter = 0;
			music_param->cycle_counter = 0;
			music_param->peridicity_counter = 1;
			music_param->music_pattern = Music_OFF;
		}
	}
}
