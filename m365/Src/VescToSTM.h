

#ifndef VESCTOSTM_H_
#define VESCTOSTM_H_

#include <stdint.h>


void VescToSTM_set_torque(int32_t current);
void VescToSTM_set_brake(int32_t current);
void VescToSTM_handle_brake();
void VescToSTM_set_speed(int32_t rpm);
float VescToSTM_get_temperature();
float VescToSTM_get_phase_current();
float VescToSTM_get_input_current();
float VescToSTM_get_id();
float VescToSTM_get_iq();
float VescToSTM_get_Vd();
float VescToSTM_get_Vq();
float VescToSTM_get_bus_voltage();
int32_t VescToSTM_get_erpm();
int32_t VescToSTM_get_rpm();

void VescToSTM_timeout_reset();
void VescToSTM_handle_timeout();

void VescToSTM_stop_motor();
void VescToSTM_start_motor();

#endif /* VESCTOSTM_H_ */
