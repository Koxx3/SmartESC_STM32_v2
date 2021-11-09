

#ifndef TUNE_H_
#define TUNE_H_

#include <stdint.h>
#include <stdbool.h>

bool tune_mcpwm_foc_hall_detect(float current, uint8_t *hall_table);
float tune_foc_measure_resistance(float current, int samples);
float tune_foc_measure_inductance(float voltage, float * used_current, uint32_t samples);
float tune_foc_measure_inductance_current(float curr_goal, int samples);
bool tune_foc_measure_res_ind(float *res, float *ind);
bool tune_foc_measure_flux_linkage(float current, float duty, float min_erpm, float res, float *linkage);

//extern const uint8_t hall_arr[8];

#endif /* TUNE_H_ */
