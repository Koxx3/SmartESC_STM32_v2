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
#include "cli_common.h"
#include "tune.h"
#include "conf_general.h"


static void(* volatile send_func)(unsigned char *data, unsigned int len) = 0;
static volatile int fw_version_sent_cnt = 0;
static uint8_t send_buffer_global[PACKET_MAX_PL_LEN];
static disp_pos_mode display_position_mode;


qd_t currComp;


void timeout_reset(){


};

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




//Current (digit) = [Current(Amp) * 65536 * Rshunt * Aop] / Vdd micro.


int32_t current_to_torque(int32_t curr_ma){
	float ret = curr_ma * CURRENT_FACTOR_mA;
	return ret;

}



const char test[13] = "1234567890123";

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

		memcpy(send_buffer + ind, test, 12);
		ind += 12;

		send_buffer[ind++] = 1;
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
				//buffer_append_float16(send_buffer, mc_interface_temp_fet_filtered(), 1e1, &ind);
				buffer_append_float16(send_buffer, NTC_GetAvTemp_C(pMCT[M1]->pTemperatureSensor) , 1e1, &ind);
			}
			if (mask & ((uint32_t)1 << 1)) {
				//buffer_append_float16(send_buffer, mc_interface_temp_motor_filtered(), 1e1, &ind);
				buffer_append_float16(send_buffer, 0 , 1e1, &ind);
			}
			if (mask & ((uint32_t)1 << 2)) {
				//buffer_append_float32(send_buffer, mc_interface_read_reset_avg_motor_current(), 1e2, &ind);
				buffer_append_float32(send_buffer, MCI_GetPhaseCurrentAmplitude(pMCI[M1])/CURRENT_FACTOR, 1e2, &ind);
			}
			if (mask & ((uint32_t)1 << 3)) {
				//buffer_append_float32(send_buffer, mc_interface_read_reset_avg_input_current(), 1e2, &ind);
				buffer_append_float32(send_buffer, (float)pMPM[M1]->_super.hAvrgElMotorPowerW/(float)VBS_GetAvBusVoltage_V(pMCT[M1]->pBusVoltageSensor), 1e2, &ind);
			}
			if (mask & ((uint32_t)1 << 4)) {
				//buffer_append_float32(send_buffer, mc_interface_read_reset_avg_id(), 1e2, &ind);
				buffer_append_float32(send_buffer, pMCI[M1]->pFOCVars->Iqd.d / CURRENT_FACTOR, 1e2, &ind);
			}
			if (mask & ((uint32_t)1 << 5)) {
				//buffer_append_float32(send_buffer, mc_interface_read_reset_avg_iq(), 1e2, &ind);
				buffer_append_float32(send_buffer, pMCI[M1]->pFOCVars->Iqd.q / CURRENT_FACTOR, 1e2, &ind);
			}
			if (mask & ((uint32_t)1 << 6)) {
				//buffer_append_float16(send_buffer, mc_interface_get_duty_cycle_now(), 1e3, &ind);
				buffer_append_float16(send_buffer, 0, 1e3, &ind);
			}
			if (mask & ((uint32_t)1 << 7)) {
				//buffer_append_float32(send_buffer, mc_interface_get_rpm(), 1e0, &ind);
				buffer_append_float32(send_buffer, MCI_GetAvrgMecSpeedUnit( pMCI[M1] ) * HALL_M1._Super.bElToMecRatio, 1e0, &ind);
			}
			if (mask & ((uint32_t)1 << 8)) {
				//buffer_append_float16(send_buffer, GET_INPUT_VOLTAGE(), 1e1, &ind);
				buffer_append_float16(send_buffer, VBS_GetAvBusVoltage_V(pMCT[M1]->pBusVoltageSensor), 1e1, &ind);
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
				//buffer_append_int32(send_buffer, mc_interface_get_tachometer_value(false), &ind);
				buffer_append_int32(send_buffer, 0, &ind);
			}
			if (mask & ((uint32_t)1 << 14)) {
				//buffer_append_int32(send_buffer, mc_interface_get_tachometer_abs_value(false), &ind);
				buffer_append_int32(send_buffer, 0, &ind);
			}
			if (mask & ((uint32_t)1 << 15)) {
				//send_buffer[ind++] = mc_interface_get_fault();
				send_buffer[ind++] = 0;
			}
			if (mask & ((uint32_t)1 << 16)) {
				//buffer_append_float32(send_buffer, mc_interface_get_pid_pos_now(), 1e6, &ind);
				buffer_append_float32(send_buffer, 0, 1e6, &ind);
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
				//buffer_append_float32(send_buffer, mc_interface_read_reset_avg_vd(), 1e3, &ind);
				buffer_append_float32(send_buffer, pMCI[M1]->pFOCVars->Vqd.d / VOLT_SCALING, 1e3, &ind);
			}
			if (mask & ((uint32_t)1 << 20)) {
				//buffer_append_float32(send_buffer, mc_interface_read_reset_avg_vq(), 1e3, &ind);
				buffer_append_float32(send_buffer, pMCI[M1]->pFOCVars->Vqd.q / VOLT_SCALING, 1e3, &ind);
			}

			reply_func(send_buffer, ind);
		} break;

			case COMM_SET_DUTY: {
				//int32_t ind = 0;
				//mc_interface_set_duty((float)buffer_get_int32(data, &ind) / 100000.0);
				timeout_reset();
			} break;

			case COMM_SET_CURRENT: {
				int32_t ind = 0;
				if(MCI_GetControlMode(pMCI[M1]) != STC_TORQUE_MODE){
					MCI_ExecTorqueRamp(pMCI[M1], MCI_GetTeref(pMCI[M1]),0);
				}
				int16_t q = current_to_torque(buffer_get_int32(data, &ind));

				if(q != currComp.q){
					if(q > SpeednTorqCtrlM1.MaxPositiveTorque){
						q = SpeednTorqCtrlM1.MaxPositiveTorque;
					}else if (q < SpeednTorqCtrlM1.MinNegativeTorque){
						q = SpeednTorqCtrlM1.MinNegativeTorque;
					}
					currComp.q = q;
					MCI_SetCurrentReferences(pMCI[M1],currComp);
				}
				timeout_reset();
			} break;

			case COMM_SET_CURRENT_BRAKE: {
				int32_t ind = 0;
				int32_t q = current_to_torque(buffer_get_int32(data, &ind))*-1;
				if(q != currComp.q){
					if(q > SpeednTorqCtrlM1.MaxPositiveTorque){
						q = SpeednTorqCtrlM1.MaxPositiveTorque;
					}else if (q < SpeednTorqCtrlM1.MinNegativeTorque){
						q = SpeednTorqCtrlM1.MinNegativeTorque;
					}
					currComp.q = q;
					MCI_SetCurrentReferences(pMCI[M1],currComp);
				}
				timeout_reset();
			} break;

			case COMM_SET_RPM: {
				int32_t ind = 0;
				MCI_ExecSpeedRamp(pMCI[M1], buffer_get_int32(data, &ind) , 100);
				timeout_reset();
			} break;

			case COMM_SET_POS: {
				//int32_t ind = 0;
				//CMD_tune();
				//mc_interface_set_pid_pos((float)buffer_get_int32(data, &ind) / 1000000.0);
				timeout_reset();
			} break;

			case COMM_SET_HANDBRAKE: {
				//int32_t ind = 0;
				//mc_interface_set_handbrake(buffer_get_float32(data, 1e3, &ind));
				timeout_reset();
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

				timeout_reset();
			} break;

			case COMM_SET_SERVO_POS: {

				} break;

				case COMM_SET_MCCONF: {
					mc_configuration *mcconf = pvPortMalloc(sizeof(mc_configuration));
					*mcconf = *mc_interface_get_configuration();


					if (confgenerator_deserialize_mcconf(data, mcconf)) {
						utils_truncate_number(&mcconf->l_current_max_scale , 0.0, 1.0);
						utils_truncate_number(&mcconf->l_current_min_scale , 0.0, 1.0);


						mcconf->lo_current_max = mcconf->l_current_max * mcconf->l_current_max_scale;
						mcconf->lo_current_min = mcconf->l_current_min * mcconf->l_current_min_scale;
						mcconf->lo_in_current_max = mcconf->l_in_current_max;
						mcconf->lo_in_current_min = mcconf->l_in_current_min;
						mcconf->lo_current_motor_max_now = mcconf->lo_current_max;
						mcconf->lo_current_motor_min_now = mcconf->lo_current_min;

						conf_general_setup_mc(mcconf);

						//commands_apply_mcconf_hw_limits(mcconf);
						//conf_general_store_mc_configuration(mcconf, mc_interface_get_motor_thread() == 2);
						//mc_interface_set_configuration(mcconf);
						//chThdSleepMilliseconds(200);

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

				case COMM_SET_APPCONF: {

				} break;

				case COMM_GET_APPCONF:
				case COMM_GET_APPCONF_DEFAULT: {

				} break;

				case COMM_SAMPLE_PRINT: {
					/*
					uint16_t sample_len;
					uint8_t decimation;
					debug_sampling_mode mode;

					int32_t ind = 0;
					mode = data[ind++];
					sample_len = buffer_get_uint16(data, &ind);
					decimation = data[ind++];
					mc_interface_sample_print_data(mode, sample_len, decimation);*/
				} break;

				case COMM_REBOOT:
					NVIC_SystemReset();
					break;

				case COMM_ALIVE:
					//SHUTDOWN_RESET();
					timeout_reset();
					break;

				case COMM_GET_DECODED_PPM: {
					int32_t ind = 0;
					uint8_t send_buffer[50];
					send_buffer[ind++] = COMM_GET_DECODED_PPM;
					//buffer_append_int32(send_buffer, (int32_t)(app_ppm_get_decoded_level() * 1000000.0), &ind);
					buffer_append_int32(send_buffer, (int32_t)(0 * 1000000.0), &ind);
					//buffer_append_int32(send_buffer, (int32_t)(servodec_get_last_pulse_len(0) * 1000000.0), &ind);
					buffer_append_int32(send_buffer, (int32_t)(0 * 1000000.0), &ind);
					reply_func(send_buffer, ind);

				} break;

				case COMM_GET_DECODED_ADC: {

					int32_t ind = 0;
					uint8_t send_buffer[50];
					send_buffer[ind++] = COMM_GET_DECODED_ADC;
					//buffer_append_int32(send_buffer, (int32_t)(app_adc_get_decoded_level() * 1000000.0), &ind);
					buffer_append_int32(send_buffer, (int32_t)(100 * 1000000.0), &ind);
					//buffer_append_int32(send_buffer, (int32_t)(app_adc_get_voltage() * 1000000.0), &ind);
					buffer_append_int32(send_buffer, (int32_t)(VBS_GetAvBusVoltage_V(pMCT[M1]->pBusVoltageSensor) * 1000000.0), &ind);
					//buffer_append_int32(send_buffer, (int32_t)(app_adc_get_decoded_level2() * 1000000.0), &ind);
					buffer_append_int32(send_buffer, (int32_t)(0 * 1000000.0), &ind);
					//buffer_append_int32(send_buffer, (int32_t)(app_adc_get_voltage2() * 1000000.0), &ind);
					buffer_append_int32(send_buffer, (int32_t)(0 * 1000000.0), &ind);
					reply_func(send_buffer, ind);
				} break;

				case COMM_GET_DECODED_CHUK: {
					int32_t ind = 0;
					uint8_t send_buffer[50];
					send_buffer[ind++] = COMM_GET_DECODED_CHUK;
					//buffer_append_int32(send_buffer, (int32_t)(app_nunchuk_get_decoded_chuk() * 1000000.0), &ind);
					buffer_append_int32(send_buffer, (int32_t)(0 * 1000000.0), &ind);
					reply_func(send_buffer, ind);
				} break;

				case COMM_GET_DECODED_BALANCE: {
					int32_t ind = 0;
					uint8_t send_buffer[50];
					send_buffer[ind++] = COMM_GET_DECODED_BALANCE;
					buffer_append_int32(send_buffer, 0, &ind);
					buffer_append_int32(send_buffer, 0, &ind);
					buffer_append_int32(send_buffer, 0, &ind);
					buffer_append_uint32(send_buffer, 0, &ind);
					buffer_append_int32(send_buffer, 0, &ind);
					buffer_append_int32(send_buffer, 0, &ind);
					buffer_append_uint16(send_buffer, 0, &ind);
					buffer_append_uint16(send_buffer, 0, &ind);
					buffer_append_int32(send_buffer, 0, &ind);
					buffer_append_int32(send_buffer, 0, &ind);
					reply_func(send_buffer, ind);
				} break;

				case COMM_FORWARD_CAN: {

				} break;

				case COMM_SET_CHUCK_DATA: {

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

				case COMM_GPD_SET_FSW: {
					timeout_reset();
					//int32_t ind = 0;
					//gpdrive_set_switching_frequency((float)buffer_get_int32(data, &ind));
				} break;

				case COMM_GPD_BUFFER_SIZE_LEFT: {
					int32_t ind = 0;
					uint8_t send_buffer[50];
					send_buffer[ind++] = COMM_GPD_BUFFER_SIZE_LEFT;
					//buffer_append_int32(send_buffer, gpdrive_buffer_size_left(), &ind);
					buffer_append_int32(send_buffer, 128, &ind);
					reply_func(send_buffer, ind);
				} break;

				case COMM_GPD_FILL_BUFFER: {
					timeout_reset();
					//int32_t ind = 0;
					//while (ind < (int)len) {
						//gpdrive_add_buffer_sample(buffer_get_float32_auto(data, &ind));
					//}
				} break;

				case COMM_GPD_OUTPUT_SAMPLE: {
					timeout_reset();
					//int32_t ind = 0;
					//gpdrive_output_sample(buffer_get_float32_auto(data, &ind));
				} break;

				case COMM_GPD_SET_MODE: {
					timeout_reset();
					//int32_t ind = 0;
					//gpdrive_set_mode(data[ind++]);
				} break;

				case COMM_GPD_FILL_BUFFER_INT8: {
					timeout_reset();
					//int32_t ind = 0;
					//while (ind < (int)len) {
						//gpdrive_add_buffer_sample_int((int8_t)data[ind++]);
					//}
				} break;

				case COMM_GPD_FILL_BUFFER_INT16: {
					timeout_reset();
					//int32_t ind = 0;
					//while (ind < (int)len) {
						//gpdrive_add_buffer_sample_int(buffer_get_int16(data, &ind));
					//}
				} break;

				case COMM_GPD_SET_BUFFER_INT_SCALE: {
					//int32_t ind = 0;
					//gpdrive_set_buffer_int_scale(buffer_get_float32_auto(data, &ind));
				} break;

				case COMM_GET_VALUES_SETUP:
				case COMM_GET_VALUES_SETUP_SELECTIVE: {
					//setup_values val = mc_interface_get_setup_values();
					setup_values val;

					float wh_batt_left = 0.0;
					//float battery_level = mc_interface_get_battery_level(&wh_batt_left);
					float battery_level = 100;

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
						//buffer_append_float16(send_buffer, mc_interface_temp_fet_filtered(), 1e1, &ind);
						buffer_append_float16(send_buffer, 0, 1e1, &ind);
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
						//buffer_append_float16(send_buffer, mc_interface_get_duty_cycle_now(), 1e3, &ind);
						buffer_append_float16(send_buffer, 0, 1e3, &ind);
					}
					if (mask & ((uint32_t)1 << 5)) {
						//buffer_append_float32(send_buffer, mc_interface_get_rpm(), 1e0, &ind);
						buffer_append_float32(send_buffer, 0, 1e0, &ind);
					}
					if (mask & ((uint32_t)1 << 6)) {
						//buffer_append_float32(send_buffer, mc_interface_get_speed(), 1e3, &ind);
						buffer_append_float32(send_buffer, 0, 1e3, &ind);
					}
					if (mask & ((uint32_t)1 << 7)) {
						//buffer_append_float16(send_buffer, GET_INPUT_VOLTAGE(), 1e1, &ind);
						buffer_append_float16(send_buffer, 0, 1e1, &ind);
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
						//buffer_append_float32(send_buffer, mc_interface_get_distance(), 1e3, &ind);
						buffer_append_float32(send_buffer, 0, 1e3, &ind);
					}
					if (mask & ((uint32_t)1 << 14)) {
						//buffer_append_float32(send_buffer, mc_interface_get_distance_abs(), 1e3, &ind);
						buffer_append_float32(send_buffer, 0, 1e3, &ind);
					}
					if (mask & ((uint32_t)1 << 15)) {
						//buffer_append_float32(send_buffer, mc_interface_get_pid_pos_now(), 1e6, &ind);
						buffer_append_float32(send_buffer, 0, 1e6, &ind);
					}
					if (mask & ((uint32_t)1 << 16)) {
						//send_buffer[ind++] = mc_interface_get_fault();
						send_buffer[ind++] = 0;
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
						//buffer_append_uint32(send_buffer, mc_interface_get_odometer(), &ind);
						buffer_append_uint32(send_buffer, 0, &ind);
					}

					reply_func(send_buffer, ind);
					//chMtxUnlock(&send_buffer_mutex);
				    } break;

				case COMM_SET_ODOMETER: {
					//int32_t ind = 0;
					//mc_interface_set_odometer(buffer_get_uint32(data, &ind));
					timeout_reset();
				} break;

				case COMM_SET_MCCONF_TEMP:
				case COMM_SET_MCCONF_TEMP_SETUP: {
					mc_configuration *mcconf = pvPortMalloc(sizeof(mc_configuration));
					*mcconf = *mc_interface_get_configuration();

					int32_t ind = 0;
					bool store = data[ind++];
					bool forward_can = data[ind++];
					bool ack = data[ind++];
					bool divide_by_controllers = data[ind++];

					float controller_num = 1.0;

					/*if (divide_by_controllers) {
						for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
							can_status_msg *msg = comm_can_get_status_msg_index(i);
							if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < 0.1) {
								controller_num += 1.0;
							}
						}
					}*/

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

					//commands_apply_mcconf_hw_limits(mcconf);

					//if (store) {
					//	conf_general_store_mc_configuration(mcconf, mc_interface_get_motor_thread() == 2);
					//}

					//mc_interface_set_configuration(mcconf);

					if (forward_can) {
						data[-1] = COMM_SET_MCCONF_TEMP;
						data[1] = 0; // No more forward
						data[2] = 0; // No ack
						data[3] = 0; // No dividing, see comment above

						// TODO: Maybe broadcast on CAN-bus?
						/*for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
							can_status_msg *msg = comm_can_get_status_msg_index(i);
							if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < 0.1) {
								comm_can_send_buffer(msg->id, data - 1, len + 1, 0);
							}
						}*/
					}

					if (ack) {
						ind = 0;
						uint8_t send_buffer[50];
						send_buffer[ind++] = packet_id;
						reply_func(send_buffer, ind);
					}

					vPortFree(mcconf);
				} break;

				case COMM_GET_MCCONF_TEMP: {
					mc_configuration *mcconf = pvPortMalloc(sizeof(mc_configuration));
					*mcconf = *mc_interface_get_configuration();
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

					vPortFree(mcconf);
					reply_func(send_buffer, ind);
				} break;

				case COMM_EXT_NRF_PRESENT: {


				} break;

				case COMM_EXT_NRF_ESB_RX_DATA: {

				} break;

				case COMM_APP_DISABLE_OUTPUT: {

				} break;

				case COMM_TERMINAL_CMD_SYNC:
					data[len] = '\0';
					//chMtxLock(&terminal_mutex);
					//terminal_process_string((char*)data);
					//chMtxUnlock(&terminal_mutex);
					break;

				case COMM_GET_IMU_DATA: {
					int32_t ind = 0;
					uint8_t send_buffer[70];
					send_buffer[ind++] = packet_id;

					int32_t ind2 = 0;
					uint32_t mask = buffer_get_uint16(data, &ind2);

					float rpy[3], acc[3], gyro[3], mag[3], q[4];
					//imu_get_rpy(rpy);
					//imu_get_accel(acc);
					//imu_get_gyro(gyro);
					//imu_get_mag(mag);
					//imu_get_quaternions(q);

					buffer_append_uint16(send_buffer, mask, &ind);

					if (mask & ((uint32_t)1 << 0)) {
						buffer_append_float32_auto(send_buffer, rpy[0], &ind);
					}
					if (mask & ((uint32_t)1 << 1)) {
						buffer_append_float32_auto(send_buffer, rpy[1], &ind);
					}
					if (mask & ((uint32_t)1 << 2)) {
						buffer_append_float32_auto(send_buffer, rpy[2], &ind);
					}

					if (mask & ((uint32_t)1 << 3)) {
						buffer_append_float32_auto(send_buffer, acc[0], &ind);
					}
					if (mask & ((uint32_t)1 << 4)) {
						buffer_append_float32_auto(send_buffer, acc[1], &ind);
					}
					if (mask & ((uint32_t)1 << 5)) {
						buffer_append_float32_auto(send_buffer, acc[2], &ind);
					}

					if (mask & ((uint32_t)1 << 6)) {
						buffer_append_float32_auto(send_buffer, gyro[0], &ind);
					}
					if (mask & ((uint32_t)1 << 7)) {
						buffer_append_float32_auto(send_buffer, gyro[1], &ind);
					}
					if (mask & ((uint32_t)1 << 8)) {
						buffer_append_float32_auto(send_buffer, gyro[2], &ind);
					}

					if (mask & ((uint32_t)1 << 9)) {
						buffer_append_float32_auto(send_buffer, mag[0], &ind);
					}
					if (mask & ((uint32_t)1 << 10)) {
						buffer_append_float32_auto(send_buffer, mag[1], &ind);
					}
					if (mask & ((uint32_t)1 << 11)) {
						buffer_append_float32_auto(send_buffer, mag[2], &ind);
					}

					if (mask & ((uint32_t)1 << 12)) {
						buffer_append_float32_auto(send_buffer, q[0], &ind);
					}
					if (mask & ((uint32_t)1 << 13)) {
						buffer_append_float32_auto(send_buffer, q[1], &ind);
					}
					if (mask & ((uint32_t)1 << 14)) {
						buffer_append_float32_auto(send_buffer, q[2], &ind);
					}
					if (mask & ((uint32_t)1 << 15)) {
						buffer_append_float32_auto(send_buffer, q[3], &ind);
					}

					reply_func(send_buffer, ind);
				} break;

				case COMM_ERASE_BOOTLOADER_ALL_CAN:

					/* Falls through. */
					/* no break */
				case COMM_ERASE_BOOTLOADER: {
					int32_t ind = 0;

					ind = 0;
					uint8_t send_buffer[50];
					send_buffer[ind++] = COMM_ERASE_BOOTLOADER;
					send_buffer[ind++] = 1;
					reply_func(send_buffer, ind);
				} break;

				case COMM_SET_CURRENT_REL: {
					//int32_t ind = 0;
					//mc_interface_set_current_rel(buffer_get_float32(data, 1e5, &ind));
					timeout_reset();
				} break;

				case COMM_CAN_FWD_FRAME: {

				} break;

				case COMM_SET_BATTERY_CUT: {
					int32_t ind = 0;
					float start = buffer_get_float32(data, 1e3, &ind);
					float end = buffer_get_float32(data, 1e3, &ind);
					bool store = data[ind++];
					bool fwd_can = data[ind++];

					/*if (fwd_can) {
						comm_can_conf_battery_cut(255, store, start, end);
					}*/

					mc_configuration *mcconf = pvPortMalloc(sizeof(mc_configuration));
					//*mcconf = *mc_interface_get_configuration();
					memset(mcconf,0,sizeof(mc_configuration));

					if (mcconf->l_battery_cut_start != start || mcconf->l_battery_cut_end != end) {
						mcconf->l_battery_cut_start = start;
						mcconf->l_battery_cut_end = end;

						/*if (store) {
							conf_general_store_mc_configuration(mcconf,
									mc_interface_get_motor_thread() == 2);
						}*/

						//mc_interface_set_configuration(mcconf);
					}

					vPortFree(mcconf);

					// Send ack
					ind = 0;
					uint8_t send_buffer[50];
					send_buffer[ind++] = packet_id;
					reply_func(send_buffer, ind);
				} break;

				case COMM_SET_CAN_MODE: {
					int32_t ind = 0;
					bool store = data[ind++];
					bool ack = data[ind++];
					int mode = data[ind++];

					/*app_configuration *appconf = mempools_alloc_appconf();
					*appconf = *app_get_configuration();
					appconf->can_mode = mode;

					if (store) {
						conf_general_store_app_configuration(appconf);
					}

					app_set_configuration(appconf);

					mempools_free_appconf(appconf);*/

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
				case COMM_DETECT_MOTOR_PARAM:
				case COMM_DETECT_MOTOR_R_L:
				case COMM_DETECT_MOTOR_FLUX_LINKAGE:
				case COMM_DETECT_ENCODER:
					break;
				case COMM_DETECT_HALL_FOC:{
					int32_t ind = 0;
					uint8_t hall_tab[8];
					uint8_t send_buffer[50];
					bool res = tune_hall_detect(buffer_get_int32(data, &ind), hall_tab);
					ind=0;
					send_buffer[ind++] = COMM_DETECT_HALL_FOC;
					memcpy(send_buffer + ind, hall_tab, 8);
					ind += 8;
					send_buffer[ind++] = res ? 0 : 1;
					reply_func(send_buffer, ind);

				} break;
				case COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP:
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

