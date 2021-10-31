/*
 * m365
 *
 * Copyright (c) 2021 Jens Kerrinnes
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef VESCCOMMAND_H_
#define VESCCOMMAND_H_

#include "VescDatatypes.h"
#include "mc_stm_types.h"
#include "mc_config.h"
#include "mc_interface.h"
#include "speed_pos_fdbk.h"
#include "drive_parameters.h"

extern qd_t currComp;

void commands_process_packet(unsigned char *data, unsigned int len, void(*reply_func)(unsigned char *data, unsigned int len));
void commands_send_packet(unsigned char *data, unsigned int len);
void send_sample();
void commands_printf(const char* format, ...);

typedef enum {
	SAMP_IDLE,
	SAMP_START,
	SAMP_SAMPLING,
	SAMP_FINISHED,
	SAMP_SENDING
}SAMP_STATES;

#define ADC_SAMPLE_MAX_LEN 500

typedef struct samp_struct samp_str;
struct samp_struct{
	SAMP_STATES state;
	uint8_t dec;
	uint8_t dec_state;
	uint16_t index;
	uint16_t n_samp;
	uint16_t vesc_tool_samples;
	debug_sampling_mode mode;
	int16_t m_curr0_samples[ADC_SAMPLE_MAX_LEN];
	int16_t m_curr1_samples[ADC_SAMPLE_MAX_LEN];
	int8_t m_phase_samples[ADC_SAMPLE_MAX_LEN];
};

extern volatile samp_str samples;

#endif
