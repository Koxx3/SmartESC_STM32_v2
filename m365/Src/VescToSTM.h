

#ifndef VESCTOSTM_H_
#define VESCTOSTM_H_

#include <stdint.h>
#include "VescDatatypes.h"


void VescToSTM_set_torque(int32_t current);
void VescToSTM_set_brake(int32_t current);
void VescToSTM_set_handbrake(float current);
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
float VescToSTM_get_pid_pos_now();

void VescToSTM_timeout_reset();
void VescToSTM_handle_timeout();

void VescToSTM_stop_motor();
void VescToSTM_start_motor();
void VescToSTM_init_odometer(mc_configuration* mcconf);
void VescToSTM_set_odometer(uint32_t meters);
uint32_t VescToSTM_get_odometer();
float VescToSTM_get_distance(void);
float VescToSTM_get_distance_abs(void);
int32_t VescToSTM_get_tachometer_value(bool reset);
int32_t VescToSTM_get_tachometer_abs_value(bool reset);
float VescToSTM_get_battery_level(float *wh_left);
void VescToSTM_set_current_rel(float val);
float VescToSTM_get_duty_cycle_now(void);
mc_fault_code VescToSTM_get_fault(void);
uint8_t VescToSTM_get_uid(uint8_t * ptr, uint8_t size);


#endif /* VESCTOSTM_H_ */
