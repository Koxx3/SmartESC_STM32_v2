

#ifndef VESCTOSTM_H_
#define VESCTOSTM_H_

#include <stdint.h>
#include "VescDatatypes.h"

#define DIR_MUL   (mc_conf.m_invert_direction ? -1 : 1)

typedef enum {
   STM_STATE_IDLE = 0,
   STM_STATE_SPEED,
   STM_STATE_TORQUE,
   STM_STATE_BRAKE,
   STM_STATE_HANDBRAKE,
   STM_STATE_OPENLOOP,
   STM_STATE_NUNCHUCK
} stm_state;

void VescToStm_nunchuk_update_output(chuck_data * chuck_d);
void VescToStm_nunchuk_update_erpm();
float VescToStm_nunchuk_get_decoded_chuk(void);
void VescToSTM_set_torque(int32_t current);
void VescToSTM_set_brake(int32_t current);
void VescToSTM_set_brake_rel_int(int32_t val);
void VescToSTM_set_handbrake(float current);
void VescToSTM_set_speed(int32_t rpm);
float VescToSTM_get_temperature();
float VescToSTM_get_temperature2();
float VescToSTM_get_phase_current();
float VescToSTM_get_input_current();
float VescToSTM_get_id();
float VescToSTM_get_iq();
float VescToSTM_get_Vd();
float VescToSTM_get_Vq();
float VescToSTM_get_bus_voltage();
int32_t VescToSTM_get_erpm();
int32_t VescToSTM_get_erpm_fast();
int32_t VescToSTM_get_rpm();
float VescToSTM_get_pid_pos_now();
int32_t VescToSTM_rpm_to_speed(int32_t rpm);
int32_t VescToSTM_erpm_to_speed(int32_t erpm);
int32_t VescToSTM_speed_to_rpm(int32_t speed);
int32_t VescToSTM_speed_to_erpm(int32_t speed);
int16_t VescToSTM_Iq_lim_hook(int16_t iq);

void VescToSTM_pwm_stop(void);
void VescToSTM_pwm_start(void);
void VescToSTM_set_minimum_current(float current);
void VescToSTM_pwm_force(bool force, bool update);
void VescToSTM_timeout_reset();
void VescToSTM_handle_timeout();
void VescToSTM_set_battery_cut(float start, float end);
void VescToSTM_set_temp_cut(float start, float end);

void VescToSTM_stop_motor();
void VescToSTM_start_motor();
void VescToSTM_init_odometer(mc_configuration* mcconf);
void VescToSTM_set_odometer(uint32_t meters);
uint32_t VescToSTM_get_odometer();
float VescToSTM_get_distance(void);
float VescToSTM_get_distance_abs(void);
int32_t VescToSTM_get_tachometer_value(bool reset);
int32_t VescToSTM_get_tachometer_abs_value(bool reset);
float VescToSTM_get_speed(void);
float VescToSTM_get_battery_level(float *wh_left);
void VescToSTM_set_current_rel(float val);
void VescToSTM_set_current_rel_int(int32_t val);
float VescToSTM_get_duty_cycle_now(void);
float VescToSTM_get_duty_cycle_now_fast(void);
float VescToSTM_get_ADC1();
float VescToSTM_get_ADC2();
void VescToSTM_set_ADC1(float val);
void VescToSTM_set_ADC2(float val);
mc_fault_code VescToSTM_get_fault(void);
uint8_t VescToSTM_get_uid(uint8_t * ptr, uint8_t size);
void VescToSTM_enable_timeout(bool enbale);
void VescToSTM_set_open_loop(bool enabled, int16_t init_angle, int16_t erpm);
void VescToSTM_set_open_loop_erpm(int16_t erpm);
void VescToSTM_ramp_current(float iq, float id);
void VescToSTM_set_current(float iq, float id);
void VescToSTM_set_brake_current_rel(float val);
void VescToSTM_update_torque(int32_t q, int32_t min_erpm, int32_t max_erpm);
#endif /* VESCTOSTM_H_ */
