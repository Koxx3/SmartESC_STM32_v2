

#ifndef TUNE_H_
#define TUNE_H_

#include <stdint.h>
#include <stdbool.h>

uint8_t tune_hall_detect(uint32_t current, uint8_t *hall_table, int32_t *offset);
bool tune_mcpwm_foc_hall_detect(float current, uint8_t *hall_table);

extern const uint8_t hall_arr[8];

#endif /* TUNE_H_ */
