

#ifndef TUNE_H_
#define TUNE_H_

#include <stdint.h>
#include <stdbool.h>

bool tune_mcpwm_foc_hall_detect(float current, uint8_t *hall_table);
float tune_foc_measure_resistance(float current, int samples);
float tune_foc_measure_inductance(float voltage, float * used_current, uint32_t samples);
float tune_foc_measure_inductance_current(float curr_goal, int samples);
bool tune_foc_measure_res_ind(float *res, float *ind);
bool tune_foc_measure_flux_linkage_openloop(float current, float duty, float erpm_per_sec, float res, float ind, float *linkage,	float *linkage_undriven, float *undriven_samples);
bool tune_foc_measure_r_l_imax(float current_min, float current_max, float max_power_loss, float *r, float *l, float *i_max);
//extern const uint8_t hall_arr[8];

#endif /* TUNE_H_ */
