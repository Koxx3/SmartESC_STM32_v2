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

#include "VescCommand.h"
#include "defines.h"
#include "buffer.h"
#include "packet.h"
#include <string.h>
#include "FreeRTOS.h"
#include "confgenerator.h"
#include <math.h>
#include "utils.h"
#include "system.h"
#include "parameters_conversion.h"
#include "tune.h"
#include "conf_general.h"
#include "VescToSTM.h"
#include "stdarg.h"
#include <printf.h>
#include "terminal.h"
#include "product.h"


static void(* volatile send_func)(unsigned char *data, unsigned int len) = 0;
static volatile int fw_version_sent_cnt = 0;
static uint8_t send_buffer_global[PACKET_MAX_PL_LEN];
static disp_pos_mode display_position_mode;



#define PRINTF_STACK_SIZE 128u
void commands_printf(const char* format, ...) {

	va_list arg;
	va_start (arg, format);
	int len;
	static char print_buffer[PRINTF_STACK_SIZE];

	print_buffer[0] = COMM_PRINT;
	len = vsnprintf(print_buffer + 1, PRINTF_STACK_SIZE - 1, format, arg);
	va_end (arg);

	if(len > 0) {
		commands_send_packet((unsigned char*)print_buffer,
				(len < 254) ? len + 1 : 255);
	}

}




//#define  MC_NO_ERROR  (uint16_t)(0x0000u)      /**< @brief No error.*/
//#define  MC_NO_FAULTS  (uint16_t)(0x0000u)     /**< @brief No error.*/
//#define  MC_FOC_DURATION  (uint16_t)(0x0001u)  /**< @brief Error: FOC rate to high.*/
//#define  MC_OVER_VOLT  (uint16_t)(0x0002u)     /**< @brief Error: Software over voltage.*/
//#define  MC_UNDER_VOLT  (uint16_t)(0x0004u)    /**< @brief Error: Software under voltage.*/
//#define  MC_OVER_TEMP  (uint16_t)(0x0008u)     /**< @brief Error: Software over temperature.*/
//#define  MC_START_UP  (uint16_t)(0x0010u)      /**< @brief Error: Startup failed.*/
//#define  MC_SPEED_FDBK  (uint16_t)(0x0020u)    /**< @brief Error: Speed feedback.*/
//#define  MC_BREAK_IN  (uint16_t)(0x0040u)      /**< @brief Error: Emergency input (Over current).*/
//#define  MC_SW_ERROR  (uint16_t)(0x0080u)      /**< @brief Software Error.*/



/**
 * Send a packet using the set send function.
 *
 * @param data
 * The packet data.
 *
 * @param len
 * The data length.
 */
void commands_send_packet(unsigned char *data, unsigned int len) {
	if (send_func) {
		send_func(data, len);
	}
}

void commands_send_mcconf(COMM_PACKET_ID packet_id, mc_configuration *mcconf) {
	send_buffer_global[0] = packet_id;
	int32_t len = confgenerator_serialize_mcconf(send_buffer_global + 1, mcconf);
	commands_send_packet(send_buffer_global, len + 1);
}

volatile samp_str samples;
/*
samples.data[0][samples.index] = FOCVars[M1].Iab.a;
samples.data[1][samples.index] = FOCVars[M1].Iab.b;
samples.data[2][samples.index] = PWM_Handle_M1._Super.CntPhA;
samples.data[3][samples.index] = PWM_Handle_M1._Super.CntPhB;
samples.data[4][samples.index] = PWM_Handle_M1._Super.CntPhC;
samples.data[5][samples.index] = HALL_M1.MeasuredElAngle;
*/
void send_sample(){
	if(samples.state == SAMP_FINISHED){

		uint8_t buffer[40];
		int32_t index = 0;
		buffer[index++] = COMM_SAMPLE_PRINT;
		buffer_append_float32_auto(buffer, (float)samples.m_curr0_samples[samples.index] / CURRENT_FACTOR_A, &index);
		buffer_append_float32_auto(buffer, (float)samples.m_curr1_samples[samples.index] / CURRENT_FACTOR_A, &index);
		buffer_append_float32_auto(buffer, 0, &index);
		buffer_append_float32_auto(buffer, 0, &index);
		buffer_append_float32_auto(buffer, 0, &index);
		buffer_append_float32_auto(buffer, 0, &index);
		buffer_append_float32_auto(buffer, 0, &index);
		buffer_append_float32_auto(buffer, 16000, &index);
		buffer[index++] = 1;
		buffer[index++] = samples.m_phase_samples[samples.index];


		samples.index++;

		if(samples.index == samples.n_samp){

			samples.index = 0;
			samples.state = SAMP_IDLE;
		}

		if(samples.vesc_tool_samples == 1000) commands_send_packet(buffer, index);
		commands_send_packet(buffer, index);
	}
}




void commands_process_packet(unsigned char *data, unsigned int len,
		void(*reply_func)(unsigned char *data, unsigned int len)) {

	if (!len) {
		return;
	}

	COMM_PACKET_ID packet_id;

	packet_id = data[0];
	data++;
	len--;


	send_func = reply_func;

	// Avoid calling invalid function pointer if it is null.
	// commands_send_packet will make the check.
	if (!reply_func) {
		reply_func = commands_send_packet;
	}


	switch (packet_id) {
	case COMM_FW_VERSION: {
		int32_t ind = 0;
		uint8_t send_buffer[50];
		send_buffer[ind++] = COMM_FW_VERSION;
		send_buffer[ind++] = FW_VERSION_MAJOR;
		send_buffer[ind++] = FW_VERSION_MINOR;

		strcpy((char*)(send_buffer + ind), HW_NAME);
		ind += strlen(HW_NAME) + 1;

		ind += VescToSTM_get_uid(send_buffer + ind, 12);

		send_buffer[ind++] = appconf.pairing_done;
		send_buffer[ind++] = FW_TEST_VERSION_NUMBER;

		send_buffer[ind++] = HW_TYPE_VESC;

		send_buffer[ind++] = 0; // No custom config

		fw_version_sent_cnt++;

		reply_func(send_buffer, ind);
		} break;

		case COMM_JUMP_TO_BOOTLOADER_ALL_CAN:
		case COMM_JUMP_TO_BOOTLOADER:
			break;
		case COMM_ERASE_NEW_APP_ALL_CAN:
		case COMM_ERASE_NEW_APP: {
			int32_t ind = 0;
			uint8_t send_buffer[50];
			send_buffer[ind++] = COMM_ERASE_NEW_APP;
			send_buffer[ind++] = 1;
			reply_func(send_buffer, ind);
		} break;
		case COMM_WRITE_NEW_APP_DATA_ALL_CAN_LZO:
		case COMM_WRITE_NEW_APP_DATA_ALL_CAN:
		case COMM_WRITE_NEW_APP_DATA_LZO:
		case COMM_WRITE_NEW_APP_DATA: {
			int32_t ind = 0;
			uint8_t send_buffer[50];
			send_buffer[ind++] = COMM_WRITE_NEW_APP_DATA;
			send_buffer[ind++] = 1;
			//buffer_append_uint32(send_buffer, new_app_offset, &ind);
			buffer_append_uint32(send_buffer, 0, &ind);
			reply_func(send_buffer, ind);
		} break;

		case COMM_GET_VALUES:
		case COMM_GET_VALUES_SELECTIVE: {
			int32_t ind = 0;
			uint8_t *send_buffer = send_buffer_global;
			send_buffer[ind++] = packet_id;

			uint32_t mask = 0xFFFFFFFF;
			if (packet_id == COMM_GET_VALUES_SELECTIVE) {
				int32_t ind2 = 0;
				mask = buffer_get_uint32(data, &ind2);
				buffer_append_uint32(send_buffer, mask, &ind);
			}

			if (mask & ((uint32_t)1 << 0)) {
				buffer_append_float16(send_buffer, VescToSTM_get_temperature() , 1e1, &ind);
			}
			if (mask & ((uint32_t)1 << 1)) {
				//buffer_append_float16(send_buffer, mc_interface_temp_motor_filtered(), 1e1, &ind);
				buffer_append_float16(send_buffer, 0, 1e1, &ind);
			}
			if (mask & ((uint32_t)1 << 2)) {
				buffer_append_float32(send_buffer, VescToSTM_get_phase_current(), 1e2, &ind);
			}
			if (mask & ((uint32_t)1 << 3)) {
				buffer_append_float32(send_buffer, VescToSTM_get_input_current(), 1e2, &ind);
			}
			if (mask & ((uint32_t)1 << 4)) {
				buffer_append_float32(send_buffer, VescToSTM_get_id(), 1e2, &ind);
			}
			if (mask & ((uint32_t)1 << 5)) {
				buffer_append_float32(send_buffer, VescToSTM_get_iq(), 1e2, &ind);
			}
			if (mask & ((uint32_t)1 << 6)) {
				buffer_append_float16(send_buffer, VescToSTM_get_duty_cycle_now(), 1e3, &ind);
			}
			if (mask & ((uint32_t)1 << 7)) {
				buffer_append_float32(send_buffer, VescToSTM_get_erpm(), 1e0, &ind);
			}
			if (mask & ((uint32_t)1 << 8)) {
				buffer_append_float16(send_buffer, VescToSTM_get_bus_voltage(), 1e1, &ind);
			}
			if (mask & ((uint32_t)1 << 9)) {
				//buffer_append_float32(send_buffer, mc_interface_get_amp_hours(false), 1e4, &ind);
				buffer_append_float32(send_buffer, 0, 1e4, &ind);
			}
			if (mask & ((uint32_t)1 << 10)) {
				//buffer_append_float32(send_buffer, mc_interface_get_amp_hours_charged(false), 1e4, &ind);
				buffer_append_float32(send_buffer, 0, 1e4, &ind);
			}
			if (mask & ((uint32_t)1 << 11)) {
				//buffer_append_float32(send_buffer, mc_interface_get_watt_hours(false), 1e4, &ind);
				buffer_append_float32(send_buffer, 0, 1e4, &ind);
			}
			if (mask & ((uint32_t)1 << 12)) {
				//buffer_append_float32(send_buffer, mc_interface_get_watt_hours_charged(false), 1e4, &ind);
				buffer_append_float32(send_buffer, 0, 1e4, &ind);
			}
			if (mask & ((uint32_t)1 << 13)) {
				buffer_append_int32(send_buffer, VescToSTM_get_tachometer_value(false), &ind);
			}
			if (mask & ((uint32_t)1 << 14)) {
				buffer_append_int32(send_buffer, VescToSTM_get_tachometer_abs_value(false), &ind);
			}
			if (mask & ((uint32_t)1 << 15)) {
				send_buffer[ind++] = VescToSTM_get_fault();
			}
			if (mask & ((uint32_t)1 << 16)) {
				buffer_append_float32(send_buffer, VescToSTM_get_pid_pos_now(), 1e6, &ind);
			}
			if (mask & ((uint32_t)1 << 17)) {
				//uint8_t current_controller_id = app_get_configuration()->controller_id;
				uint8_t current_controller_id = 1;
				send_buffer[ind++] = current_controller_id;
			}
			if (mask & ((uint32_t)1 << 18)) {
				//buffer_append_float16(send_buffer, NTC_TEMP_MOS1(), 1e1, &ind);
				buffer_append_float16(send_buffer, 0, 1e1, &ind);
				//buffer_append_float16(send_buffer, NTC_TEMP_MOS2(), 1e1, &ind);
				buffer_append_float16(send_buffer, 0, 1e1, &ind);
				//buffer_append_float16(send_buffer, NTC_TEMP_MOS3(), 1e1, &ind);
				buffer_append_float16(send_buffer, 0, 1e1, &ind);
			}
			if (mask & ((uint32_t)1 << 19)) {
				buffer_append_float32(send_buffer, VescToSTM_get_Vd(), 1e3, &ind);
			}
			if (mask & ((uint32_t)1 << 20)) {
				buffer_append_float32(send_buffer, VescToSTM_get_Vq(), 1e3, &ind);
			}

			reply_func(send_buffer, ind);
		} break;

			case COMM_SET_DUTY: {
				//int32_t ind = 0;
				//mc_interface_set_duty((float)buffer_get_int32(data, &ind) / 100000.0);
				VescToSTM_timeout_reset();
			} break;

			case COMM_SET_CURRENT: {
				int32_t ind = 0;
				VescToSTM_set_torque(buffer_get_int32(data, &ind));
				VescToSTM_timeout_reset();
			} break;

			case COMM_SET_CURRENT_BRAKE: {
				int32_t ind = 0;
				VescToSTM_set_brake(buffer_get_int32(data, &ind)*-1);
				VescToSTM_timeout_reset();
			} break;

			case COMM_SET_RPM: {
				int32_t ind = 0;
				VescToSTM_set_speed(buffer_get_int32(data, &ind));
				VescToSTM_timeout_reset();
			} break;

			case COMM_SET_POS: {
				//int32_t ind = 0;
				//mc_interface_set_pid_pos((float)buffer_get_int32(data, &ind) / 1000000.0);
				VescToSTM_timeout_reset();
			} break;

			case COMM_SET_HANDBRAKE: {
				int32_t ind = 0;
				VescToSTM_set_handbrake(buffer_get_float32(data, 1e3, &ind));
				VescToSTM_timeout_reset();
			} break;

			case COMM_SET_DETECT: {
				int32_t ind = 0;
				display_position_mode = data[ind++];
				/*
				if (mc_interface_get_configuration()->motor_type == MOTOR_TYPE_BLDC) {
					if (display_position_mode == DISP_POS_MODE_NONE) {
						mc_interface_release_motor();
					} else if (display_position_mode == DISP_POS_MODE_INDUCTANCE) {
						mcpwm_set_detect();
					}
				}*/

				VescToSTM_timeout_reset();
			} break;

			case COMM_SET_SERVO_POS:
				break;

			case COMM_SET_MCCONF: {
				mc_configuration *mcconf = pvPortMalloc(sizeof(mc_configuration));
				*mcconf = *mc_interface_get_configuration();


				if (confgenerator_deserialize_mcconf(data, mcconf)) {
					utils_truncate_number(&mcconf->l_current_max_scale , 0.0, 1.0);
					utils_truncate_number(&mcconf->l_current_min_scale , 0.0, 1.0);


					/*mcconf->lo_current_max = mcconf->l_current_max * mcconf->l_current_max_scale;
					mcconf->lo_current_min = mcconf->l_current_min * mcconf->l_current_min_scale;
					mcconf->lo_in_current_max = mcconf->l_in_current_max;
					mcconf->lo_in_current_min = mcconf->l_in_current_min;
					mcconf->lo_current_motor_max_now = mcconf->lo_current_max;
					mcconf->lo_current_motor_min_now = mcconf->lo_current_min;*/

					conf_general_setup_mc(mcconf);
					conf_general_store_mc_configuration(mcconf, 0);

					vTaskDelay(100);

					int32_t ind = 0;
					uint8_t send_buffer[50];
					send_buffer[ind++] = packet_id;
					reply_func(send_buffer, ind);
				} else {
					//commands_printf("Warning: Could not set mcconf due to wrong signature");
				}

				int32_t ind = 0;
				uint8_t send_buffer[50];
				send_buffer[ind++] = packet_id;
				reply_func(send_buffer, ind);

				vPortFree(mcconf);
				} break;

				case COMM_GET_MCCONF:
				case COMM_GET_MCCONF_DEFAULT: {
					mc_configuration *mcconf = pvPortMalloc(sizeof(mc_configuration));

					if (packet_id == COMM_GET_MCCONF) {
						*mcconf = *mc_interface_get_configuration();
					} else {
						confgenerator_set_defaults_mcconf(mcconf);
					}
					commands_send_mcconf(packet_id, mcconf);
					vPortFree(mcconf);
				} break;

				case COMM_SET_APPCONF:
					break;
				case COMM_GET_APPCONF:
				case COMM_GET_APPCONF_DEFAULT:
					break;

				case COMM_SAMPLE_PRINT: {

					uint16_t sample_len;

					int32_t ind = 0;
					samples.mode = data[ind++];
					sample_len = buffer_get_uint16(data, &ind);
					samples.dec = data[ind++];
					samples.vesc_tool_samples = sample_len;

					sample_len = sample_len > ADC_SAMPLE_MAX_LEN ? ADC_SAMPLE_MAX_LEN : sample_len;

					samples.n_samp = sample_len;
					samples.index = 0;
					samples.dec_state = 0;
					samples.state = SAMP_START;

				} break;

				case COMM_REBOOT:
					NVIC_SystemReset();
					break;

				case COMM_ALIVE:
					//SHUTDOWN_RESET();
					VescToSTM_timeout_reset();
					break;

				case COMM_GET_DECODED_PPM: {
					int32_t ind = 0;
					uint8_t send_buffer[50];
					send_buffer[ind++] = COMM_GET_DECODED_PPM;
					//buffer_append_int32(send_buffer, (int32_t)(app_ppm_get_decoded_level() * 1000000.0), &ind);
					buffer_append_int32(send_buffer, 0, &ind);
					//buffer_append_int32(send_buffer, (int32_t)(servodec_get_last_pulse_len(0) * 1000000.0), &ind);
					buffer_append_int32(send_buffer, 0, &ind);
					reply_func(send_buffer, ind);

				} break;

				case COMM_GET_DECODED_ADC: {

					int32_t ind = 0;
					uint8_t send_buffer[50];
					send_buffer[ind++] = COMM_GET_DECODED_ADC;
					buffer_append_int32(send_buffer, (int32_t)(VescToSTM_get_battery_level(NULL) * 1000000.0), &ind);
					//buffer_append_int32(send_buffer, (int32_t)(app_adc_get_voltage() * 1000000.0), &ind);
					buffer_append_int32(send_buffer, (int32_t)(VBS_GetAvBusVoltage_V(pMCT[M1]->pBusVoltageSensor) * 1000000.0), &ind);
					//buffer_append_int32(send_buffer, (int32_t)(app_adc_get_decoded_level2() * 1000000.0), &ind);
					buffer_append_int32(send_buffer, 0, &ind);
					//buffer_append_int32(send_buffer, (int32_t)(app_adc_get_voltage2() * 1000000.0), &ind);
					buffer_append_int32(send_buffer, 0, &ind);
					reply_func(send_buffer, ind);
				} break;

				case COMM_GET_DECODED_CHUK: {
					int32_t ind = 0;
					uint8_t send_buffer[50];
					send_buffer[ind++] = COMM_GET_DECODED_CHUK;
					//buffer_append_int32(send_buffer, (int32_t)(app_nunchuk_get_decoded_chuk() * 1000000.0), &ind);
					buffer_append_int32(send_buffer, 0, &ind);
					reply_func(send_buffer, ind);
				} break;

				case COMM_GET_DECODED_BALANCE: {
					int32_t ind = 0;
					uint8_t send_buffer[50];
					send_buffer[ind++] = COMM_GET_DECODED_BALANCE;
					memset(send_buffer,0,36);
					ind+=36;
					reply_func(send_buffer, ind);
				} break;

				case COMM_FORWARD_CAN:
					break;

				case COMM_SET_CHUCK_DATA:{
					chuck_data chuck_d_tmp;

					int32_t ind = 0;
					chuck_d_tmp.js_x = data[ind++];
					chuck_d_tmp.js_y = data[ind++];
					chuck_d_tmp.bt_c = data[ind++];
					chuck_d_tmp.bt_z = data[ind++];
					chuck_d_tmp.acc_x = buffer_get_int16(data, &ind);
					chuck_d_tmp.acc_y = buffer_get_int16(data, &ind);
					chuck_d_tmp.acc_z = buffer_get_int16(data, &ind);

					if (len >= (unsigned int)ind + 2) {
						chuck_d_tmp.rev_has_state = data[ind++];
						chuck_d_tmp.is_rev = data[ind++];
					} else {
						chuck_d_tmp.rev_has_state = false;
						chuck_d_tmp.is_rev = false;
					}
					VescToStm_nunchuk_update_output(&chuck_d_tmp);
				} break;

				case COMM_CUSTOM_APP_DATA:
					break;

				case COMM_NRF_START_PAIRING: {
					int32_t ind = 0;
					uint8_t send_buffer[50];
					send_buffer[ind++] = packet_id;
					send_buffer[ind++] = NRF_PAIR_STARTED;
					reply_func(send_buffer, ind);
				} break;

				case COMM_GPD_BUFFER_SIZE_LEFT: {
					int32_t ind = 0;
					uint8_t send_buffer[50];
					send_buffer[ind++] = COMM_GPD_BUFFER_SIZE_LEFT;
					//buffer_append_int32(send_buffer, gpdrive_buffer_size_left(), &ind);
					buffer_append_int32(send_buffer, 128, &ind);
					reply_func(send_buffer, ind);
				} break;
				case COMM_GPD_SET_FSW:
				case COMM_GPD_FILL_BUFFER:
				case COMM_GPD_OUTPUT_SAMPLE:
				case COMM_GPD_SET_MODE:
				case COMM_GPD_FILL_BUFFER_INT8:
				case COMM_GPD_FILL_BUFFER_INT16:
				case COMM_GPD_SET_BUFFER_INT_SCALE:
				break;

				case COMM_GET_VALUES_SETUP:
				case COMM_GET_VALUES_SETUP_SELECTIVE: {
					//setup_values val = mc_interface_get_setup_values();
					setup_values val;
					val.ah_charge_tot = 0;
					val.ah_tot = 0;
					val.current_in_tot = 0;
					val.current_tot = 0;
					val.num_vescs = 0;
					val.wh_charge_tot = 0;
					val.wh_tot = 0;

					float wh_batt_left = 0.0;
					float battery_level = VescToSTM_get_battery_level(&wh_batt_left);

					int32_t ind = 0;
					//chMtxLock(&send_buffer_mutex);
					uint8_t *send_buffer = send_buffer_global;
					send_buffer[ind++] = packet_id;

					uint32_t mask = 0xFFFFFFFF;
					if (packet_id == COMM_GET_VALUES_SETUP_SELECTIVE) {
						int32_t ind2 = 0;
						mask = buffer_get_uint32(data, &ind2);
						buffer_append_uint32(send_buffer, mask, &ind);
					}

					if (mask & ((uint32_t)1 << 0)) {
						buffer_append_float16(send_buffer, VescToSTM_get_temperature(), 1e1, &ind);
					}
					if (mask & ((uint32_t)1 << 1)) {
						//buffer_append_float16(send_buffer, mc_interface_temp_motor_filtered(), 1e1, &ind);
						buffer_append_float16(send_buffer, 0, 1e1, &ind);
					}
					if (mask & ((uint32_t)1 << 2)) {
						buffer_append_float32(send_buffer, val.current_tot, 1e2, &ind);
					}
					if (mask & ((uint32_t)1 << 3)) {
						buffer_append_float32(send_buffer, val.current_in_tot, 1e2, &ind);
					}
					if (mask & ((uint32_t)1 << 4)) {
						buffer_append_float16(send_buffer, VescToSTM_get_duty_cycle_now(), 1e3, &ind);
					}
					if (mask & ((uint32_t)1 << 5)) {
						buffer_append_float32(send_buffer, VescToSTM_get_erpm(), 1e0, &ind);
					}
					if (mask & ((uint32_t)1 << 6)) {
						buffer_append_float32(send_buffer, VescToSTM_get_odometer(), 1e3, &ind);
					}
					if (mask & ((uint32_t)1 << 7)) {
						buffer_append_float16(send_buffer, VescToSTM_get_bus_voltage(), 1e1, &ind);
					}
					if (mask & ((uint32_t)1 << 8)) {
						buffer_append_float16(send_buffer, battery_level, 1e3, &ind);
					}
					if (mask & ((uint32_t)1 << 9)) {
						buffer_append_float32(send_buffer, val.ah_tot, 1e4, &ind);
					}
					if (mask & ((uint32_t)1 << 10)) {
						buffer_append_float32(send_buffer, val.ah_charge_tot, 1e4, &ind);
					}
					if (mask & ((uint32_t)1 << 11)) {
						buffer_append_float32(send_buffer, val.wh_tot, 1e4, &ind);
					}
					if (mask & ((uint32_t)1 << 12)) {
						buffer_append_float32(send_buffer, val.wh_charge_tot, 1e4, &ind);
					}
					if (mask & ((uint32_t)1 << 13)) {
						buffer_append_float32(send_buffer, VescToSTM_get_distance(), 1e3, &ind);
					}
					if (mask & ((uint32_t)1 << 14)) {
						buffer_append_float32(send_buffer, VescToSTM_get_distance_abs(), 1e3, &ind);
					}
					if (mask & ((uint32_t)1 << 15)) {
						buffer_append_float32(send_buffer, VescToSTM_get_pid_pos_now(), 1e6, &ind);
					}
					if (mask & ((uint32_t)1 << 16)) {
						send_buffer[ind++] = VescToSTM_get_fault();
					}
					if (mask & ((uint32_t)1 << 17)) {
						//uint8_t current_controller_id = app_get_configuration()->controller_id;
						//send_buffer[ind++] = current_controller_id;
						send_buffer[ind++] = 0;
					}
					if (mask & ((uint32_t)1 << 18)) {
						send_buffer[ind++] = val.num_vescs;
					}
					if (mask & ((uint32_t)1 << 19)) {
						buffer_append_float32(send_buffer, wh_batt_left, 1e3, &ind);
					}
					if (mask & ((uint32_t)1 << 20)) {
						buffer_append_uint32(send_buffer, VescToSTM_get_odometer(), &ind);
					}

					reply_func(send_buffer, ind);
					//chMtxUnlock(&send_buffer_mutex);
				    } break;

				case COMM_SET_ODOMETER: {
					int32_t ind = 0;
					VescToSTM_set_odometer(buffer_get_uint32(data, &ind));
					VescToSTM_timeout_reset();
				} break;

				case COMM_SET_MCCONF_TEMP:
				case COMM_SET_MCCONF_TEMP_SETUP: {
					mc_configuration *mcconf = &mc_conf;

					int32_t ind = 0;
					bool store = data[ind++];
					bool forward_can = data[ind++];
					bool ack = data[ind++];
					bool divide_by_controllers = data[ind++];

					float controller_num = 1.0;

					mcconf->l_current_min_scale = buffer_get_float32_auto(data, &ind);
					mcconf->l_current_max_scale = buffer_get_float32_auto(data, &ind);

					if (packet_id == COMM_SET_MCCONF_TEMP_SETUP) {
						const float fact = ((mcconf->si_motor_poles / 2.0) * 60.0 *
								mcconf->si_gear_ratio) / (mcconf->si_wheel_diameter * M_PI);

						mcconf->l_min_erpm = buffer_get_float32_auto(data, &ind) * fact;
						mcconf->l_max_erpm = buffer_get_float32_auto(data, &ind) * fact;

						// Write computed RPM back and change forwarded packet id to
						// COMM_SET_MCCONF_TEMP. This way only the master has to be
						// aware of the setup information.
						ind -= 8;
						buffer_append_float32_auto(data, mcconf->l_min_erpm, &ind);
						buffer_append_float32_auto(data, mcconf->l_max_erpm, &ind);
					} else {
						mcconf->l_min_erpm = buffer_get_float32_auto(data, &ind);
						mcconf->l_max_erpm = buffer_get_float32_auto(data, &ind);
					}

					mcconf->l_min_duty = buffer_get_float32_auto(data, &ind);
					mcconf->l_max_duty = buffer_get_float32_auto(data, &ind);
					mcconf->l_watt_min = buffer_get_float32_auto(data, &ind) / controller_num;
					mcconf->l_watt_max = buffer_get_float32_auto(data, &ind) / controller_num;

					// Write divided data back to the buffer, as the other controllers have no way to tell
					// how many controllers are on the bus and thus need pre-divided data.
					// We set divide by controllers to false before forwarding.
					ind -= 8;
					buffer_append_float32_auto(data, mcconf->l_watt_min, &ind);
					buffer_append_float32_auto(data, mcconf->l_watt_max, &ind);

					// Battery limits can be set optionally in a backwards-compatible way.
					if ((int32_t)len >= (ind + 8)) {
						mcconf->l_in_current_min = buffer_get_float32_auto(data, &ind);
						mcconf->l_in_current_max = buffer_get_float32_auto(data, &ind);
					}

					mcconf->lo_current_min = mcconf->l_current_min * mcconf->l_current_min_scale;
					mcconf->lo_current_max = mcconf->l_current_max * mcconf->l_current_max_scale;
					mcconf->lo_current_motor_min_now = mcconf->lo_current_min;
					mcconf->lo_current_motor_max_now = mcconf->lo_current_max;
					mcconf->lo_in_current_min = mcconf->l_in_current_min;
					mcconf->lo_in_current_max = mcconf->l_in_current_max;

					//VescToSTM_set_erpm_limits(mcconf);

					//commands_apply_mcconf_hw_limits(mcconf);

					//if (store) {
					//	conf_general_store_mc_configuration(mcconf, mc_interface_get_motor_thread() == 2);
					//}


					if (ack) {
						ind = 0;
						uint8_t send_buffer[50];
						send_buffer[ind++] = packet_id;
						reply_func(send_buffer, ind);
					}

				} break;

				case COMM_GET_MCCONF_TEMP: {
					mc_configuration *mcconf = &mc_conf;
					int32_t ind = 0;
					uint8_t send_buffer[60];

					send_buffer[ind++] = packet_id;
					buffer_append_float32_auto(send_buffer, mcconf->l_current_min_scale, &ind);
					buffer_append_float32_auto(send_buffer, mcconf->l_current_max_scale, &ind);
					buffer_append_float32_auto(send_buffer, mcconf->l_min_erpm, &ind);
					buffer_append_float32_auto(send_buffer, mcconf->l_max_erpm, &ind);
					buffer_append_float32_auto(send_buffer, mcconf->l_min_duty, &ind);
					buffer_append_float32_auto(send_buffer, mcconf->l_max_duty, &ind);
					buffer_append_float32_auto(send_buffer, mcconf->l_watt_min, &ind);
					buffer_append_float32_auto(send_buffer, mcconf->l_watt_max, &ind);
					buffer_append_float32_auto(send_buffer, mcconf->l_in_current_min, &ind);
					buffer_append_float32_auto(send_buffer, mcconf->l_in_current_max, &ind);
					// Setup config needed for speed calculation
					send_buffer[ind++] = (uint8_t)mcconf->si_motor_poles;
					buffer_append_float32_auto(send_buffer, mcconf->si_gear_ratio, &ind);
					buffer_append_float32_auto(send_buffer, mcconf->si_wheel_diameter, &ind);

					reply_func(send_buffer, ind);
				} break;

				case COMM_EXT_NRF_PRESENT:
				case COMM_EXT_NRF_ESB_RX_DATA:
				case COMM_APP_DISABLE_OUTPUT:
					break;

				case COMM_TERMINAL_CMD_SYNC:
					data[len] = '\0';
					terminal_process_string((char*)data);
					break;

				case COMM_GET_IMU_DATA: {
					int32_t ind = 0;
					uint8_t send_buffer[70];
					send_buffer[ind++] = packet_id;

					int32_t ind2 = 0;
					uint32_t mask = buffer_get_uint16(data, &ind2);

					buffer_append_uint16(send_buffer, mask, &ind);

					for(uint8_t i=0;i<16;i++){
						if (mask & ((uint32_t)1 << i)) {
							buffer_append_float32_auto(send_buffer, 0, &ind);
						}
					}

					reply_func(send_buffer, ind);
				} break;

				case COMM_ERASE_BOOTLOADER_ALL_CAN:
				case COMM_ERASE_BOOTLOADER: {
					int32_t ind = 0;
					uint8_t send_buffer[50];
					send_buffer[ind++] = COMM_ERASE_BOOTLOADER;
					send_buffer[ind++] = 1;
					reply_func(send_buffer, ind);
				} break;

				case COMM_SET_CURRENT_REL: {
					int32_t ind = 0;
					VescToSTM_set_current_rel(buffer_get_float32(data, 1e5, &ind));
					VescToSTM_timeout_reset();
				} break;

				case COMM_CAN_FWD_FRAME:
					break;
				case COMM_SET_BATTERY_CUT: {
					int32_t ind = 0;
					float start = buffer_get_float32(data, 1e3, &ind);
					float end = buffer_get_float32(data, 1e3, &ind);
					bool store = data[ind++];
					bool fwd_can = data[ind++];

					/*if (fwd_can) {
						comm_can_conf_battery_cut(255, store, start, end);
					}*/

					mc_configuration *mcconf = &mc_conf;

					if (mcconf->l_battery_cut_start != start || mcconf->l_battery_cut_end != end) {
						mcconf->l_battery_cut_start = start;
						mcconf->l_battery_cut_end = end;

						/*if (store) {
							conf_general_store_mc_configuration(mcconf,
									mc_interface_get_motor_thread() == 2);
						}*/

						//mc_interface_set_configuration(mcconf);
					}


					// Send ack
					ind = 0;
					uint8_t send_buffer[50];
					send_buffer[ind++] = packet_id;
					reply_func(send_buffer, ind);
				} break;

				case COMM_SET_CAN_MODE: {
					int32_t ind = 0;
					bool ack = data[ind++];
					if (ack) {
						ind = 0;
						uint8_t send_buffer[50];
						send_buffer[ind++] = packet_id;
						reply_func(send_buffer, ind);
					}
				} break;

				case COMM_BMS_GET_VALUES:
				case COMM_BMS_SET_CHARGE_ALLOWED:
				case COMM_BMS_SET_BALANCE_OVERRIDE:
				case COMM_BMS_RESET_COUNTERS:
				case COMM_BMS_FORCE_BALANCE:
				case COMM_BMS_ZERO_CURRENT_OFFSET: {
					//bms_process_cmd(data - 1, len + 1, reply_func);
					break;
				}

				// Blocking commands. Only one of them runs at any given time, in their
				// own thread. If other blocking commands come before the previous one has
				// finished, they are discarded.
				case COMM_TERMINAL_CMD:
					data[len] = '\0';
					terminal_process_string((char*)data);
					break;
				case COMM_DETECT_MOTOR_PARAM:
				case COMM_DETECT_MOTOR_R_L:{
					float r = 0.0;
					float l = 1.0;
					uint8_t send_buffer[50];

					tune_foc_measure_res_ind(&r, &l);

					int32_t ind = 0;
					send_buffer[ind++] = COMM_DETECT_MOTOR_R_L;
					buffer_append_float32(send_buffer, r, 1e6, &ind);
					buffer_append_float32(send_buffer, l, 1e3, &ind);
					reply_func(send_buffer, ind);
				}
					break;
				case COMM_DETECT_MOTOR_FLUX_LINKAGE:
				case COMM_DETECT_ENCODER:
					break;
				case COMM_DETECT_HALL_FOC:{
					int32_t ind = 0;
					uint8_t hall_tab[8];
					uint8_t send_buffer[50];
					bool res = tune_mcpwm_foc_hall_detect(buffer_get_int32(data, &ind), hall_tab);
					ind=0;
					send_buffer[ind++] = COMM_DETECT_HALL_FOC;
					memcpy(send_buffer + ind, hall_tab, 8);
					ind += 8;
					send_buffer[ind++] = res ? 0 : 1;
					reply_func(send_buffer, ind);

				} break;
				case COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP: {
					int32_t ind = 0;
					uint8_t send_buffer[50];
					float current = buffer_get_float32(data, 1e3, &ind);
					float erpm_per_sec = buffer_get_float32(data, 1e3, &ind);
					float duty = buffer_get_float32(data, 1e3, &ind);
					float resistance = buffer_get_float32(data, 1e6, &ind);
					float inductance = 0.0;

					if (len >= (uint32_t)ind + 4) {
						inductance = buffer_get_float32(data, 1e8, &ind);
					}

					float linkage, linkage_undriven, undriven_samples;
					bool res = tune_foc_measure_flux_linkage_openloop(current, duty,
							erpm_per_sec, resistance, inductance,
							&linkage, &linkage_undriven, &undriven_samples);

					if (undriven_samples > 60) {
						linkage = linkage_undriven;
					}

					if (!res) {
						linkage = 0.0;
					}

					ind = 0;
					send_buffer[ind++] = COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP;
					buffer_append_float32(send_buffer, linkage, 1e7, &ind);
					reply_func(send_buffer, ind);
				} break;

				case COMM_DETECT_APPLY_ALL_FOC:
				case COMM_PING_CAN:
				case COMM_BM_CONNECT:
				case COMM_BM_ERASE_FLASH_ALL:
				case COMM_BM_WRITE_FLASH_LZO:
				case COMM_BM_WRITE_FLASH:
				case COMM_BM_REBOOT:
				case COMM_BM_DISCONNECT:
				case COMM_BM_MAP_PINS_DEFAULT:
				case COMM_BM_MAP_PINS_NRF5X:
				case COMM_BM_MEM_READ:
				case COMM_GET_IMU_CALIBRATION:
					/*
					if (!is_blocking) {
						memcpy(blocking_thread_cmd_buffer, data - 1, len + 1);
						blocking_thread_cmd_len = len + 1;
						is_blocking = true;
						blocking_thread_motor = mc_interface_get_motor_thread();
						send_func_blocking = reply_func;
						chEvtSignal(blocking_tp, (eventmask_t)1);
					}*/
					break;

				default:
					break;
				}

}
