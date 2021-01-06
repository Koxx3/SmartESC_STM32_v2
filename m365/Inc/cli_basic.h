#ifndef CLI_BASIC_H
#define CLI_BASIC_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "TTerm.h"
#include "main.h"
    
#define ESC_STR "\x1b"

#define portM   ((port_str*)handle->port)
   
    
#define set_bit(var, bit) ((var) |= (1 << (bit)))

/* Bit löschen */
#define clear_bit(var, bit) ((var) &= (unsigned)~(1 << (bit)))

/* Bit togglen */
#define toggle_bit(var,bit) ((var) ^= (1 << (bit)))

/* Bit abfragen */
#define bit_is_set(var, bit) ((var) & (1 << (bit)))
#define bit_is_clear(var, bit) !bit_is_set(var, bit)

#define PARAM_SIZE(param) sizeof(param) / sizeof(parameter_entry)

enum cli_types{
    TYPE_UNSIGNED,
    TYPE_SIGNED,
    TYPE_FLOAT,
    TYPE_CHAR,
    TYPE_STRING,
    TYPE_BUFFER
};

    
#define typename(x) _Generic((x), \
    uint8_t:    TYPE_UNSIGNED, \
    uint16_t:   TYPE_UNSIGNED, \
    uint32_t:   TYPE_UNSIGNED, \
    int8_t:     TYPE_SIGNED, \
    int16_t:    TYPE_SIGNED, \
    int32_t:    TYPE_SIGNED, \
    float:      TYPE_FLOAT, \
    uint16_t*:  TYPE_BUFFER, \
    char:       TYPE_CHAR, \
    char*:      TYPE_STRING)

#define SIZEP(x) ((char*)(&(x) + 1) - (char*)&(x))
#define ADD_PARAM(text, value_var, min, max, div, update_func, help_text) {text, &value_var, SIZEP(value_var), typename(value_var),min, max, div, update_func, help_text},
#define ADD_COMMAND(command, command_func, help_text) {command, command_func, help_text},

    
#define PARAM_DEFAULT   0
#define PARAM_CONFIG    1

#define ROW_SIZE 16
#define DATASET_BYTES 5

#define CYDEV_EEPROM_ROW_SIZE 4u
#define CY_EEPROM_SIZEOF_ROW        (CYDEV_EEPROM_ROW_SIZE)

#define CYDEV_EE_SIZE 1024
#define CY_EEPROM_SIZE              (CYDEV_EE_SIZE)

#define ADDR_FLASH_PAGE_63    ((uint32_t)0x0800FC00) /* Base @ of Page 63, 1 Kbytes */
   
typedef struct parameter_entry_struct parameter_entry;
struct parameter_entry_struct {
	const char *name;
	void *value;
	const uint8_t size;
	const uint8_t type;
	const int32_t min;
	const int32_t max;
    const uint16_t div;
	uint8_t (*callback_function)(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);
	const char *help;
};

uint8_t updateDefaultFunction(parameter_entry * params, char * newValue, uint8_t index, TERMINAL_HANDLE * handle);
void EEPROM_check_hash(parameter_entry * params, uint8_t param_size, TERMINAL_HANDLE * handle);
void EEPROM_write_conf(parameter_entry * params, uint8_t param_size, uint16_t eeprom_offset ,TERMINAL_HANDLE * handle);
void EEPROM_read_conf(parameter_entry * params, uint8_t param_size, uint16_t eeprom_offset ,TERMINAL_HANDLE * handle);
void print_param_help(parameter_entry * params, uint8_t param_size, TERMINAL_HANDLE * handle);
void print_param(parameter_entry * params, uint8_t index, TERMINAL_HANDLE * handle);

uint8_t getch(TERMINAL_HANDLE * handle, TickType_t xTicksToWait);
uint8_t getche(TERMINAL_HANDLE * handle, TickType_t xTicksToWait);
uint8_t kbhit(TERMINAL_HANDLE * handle);
uint8_t Term_check_break(TERMINAL_HANDLE * handle, uint32_t ms_to_wait);

uint32_t djb_hash(const char* cp);

#endif
