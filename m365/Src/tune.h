

#ifndef TUNE_H_
#define TUNE_H_

#include <stdint.h>
#include <stdbool.h>

bool tune_mcpwm_foc_hall_detect(float current, uint8_t *hall_table);

//extern const uint8_t hall_arr[8];

#endif /* TUNE_H_ */
