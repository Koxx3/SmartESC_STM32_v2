

#ifndef TERMINAL_H_
#define TERMINAL_H_

#include <stdint.h>
#include <stdbool.h>
#include "packet.h"

void terminal_process_string(char *str, PACKET_STATE_t * phandle);


#endif /* TUNE_H_ */
