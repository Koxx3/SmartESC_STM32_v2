#ifndef ___MUSIC_H
#define ___MUSIC_H

typedef enum{
	Music1,
	Music_OFF
}MUSIC_PATTERNS;

typedef struct music_param{
	uint16_t time_interval;
	uint16_t number_of_cycles;
	int16_t music_signal;
	uint16_t cycle_counter;
	const uint8_t* music_buff;
	uint16_t overriding_frequency;
	uint8_t on_time;
	uint8_t frequency;
	MUSIC_PATTERNS music_pattern;
	uint8_t buffer_size ;
	uint8_t buff_counter;
	uint32_t music_perodicity : 4;
	uint32_t peridicity_counter : 4;
	uint32_t music_amplitude : 3;
	uint32_t traverse_direction : 1;
	uint32_t traverse_direction_alternate : 1;
	uint32_t overriding_frequency_feature : 1;
}MUSIC_PARAM;

int32_t music_init(MUSIC_PARAM* music_param);
void music_update(MUSIC_PARAM* music_param);
void set_music_command(MUSIC_PATTERNS cmd, MUSIC_PARAM* music_param);

#endif 
