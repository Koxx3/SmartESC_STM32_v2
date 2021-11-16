#include "app.h"
#include "crc.h"

static app_configuration appconf;
static bool output_vt_init_done = false;
static volatile bool output_disabled_now = false;

// Private functions
static void output_vt_cb(void *arg);

const app_configuration* app_get_configuration(void) {
	return &appconf;
}

/**
 * Reconfigure and restart all apps. Some apps don't have any configuration options.
 *
 * @param conf
 * The new configuration to use.
 */
void app_set_configuration(app_configuration *conf) {
	appconf = *conf;

	switch (appconf.app_to_use) {
		case APP_UART:
			task_app_init();
			break;
		default:
			break;
	}
}

/**
 * Disable output on apps
 *
 * @param time_ms
 * The amount of time to disable output in ms
 * 0: Enable output now
 * -1: Disable forever
 * >0: Amount of milliseconds to disable output
 */
void app_disable_output(int time_ms) {
	if (!output_vt_init_done) {
		//chVTObjectInit(&output_vt);
		output_vt_init_done = true;
	}

	if (time_ms == 0) {
		output_disabled_now = false;
	} else if (time_ms == -1) {
		output_disabled_now = true;
		//chVTReset(&output_vt);
	} else {
		output_disabled_now = true;
		//chVTSet(&output_vt, MS2ST(time_ms), output_vt_cb, 0);
	}
}

bool app_is_output_disabled(void) {
	return output_disabled_now;
}

static void output_vt_cb(void *arg) {
	(void)arg;
	output_disabled_now = false;
}

/**
 * Get app_configuration CRC
 *
 * @param conf
 * Pointer to app_configuration or NULL for current appconf
 *
 * @return
 * CRC16 (with crc field in struct temporarily set to zero).
 */
unsigned app_calc_crc(app_configuration* conf) {
	//if(null == conf)
	//	conf = &appconf;

	unsigned crc_old = conf->crc;
	conf->crc = 0;
	unsigned crc_new = crc16((uint8_t*)conf, sizeof(app_configuration));
	conf->crc = crc_old;
	return crc_new;
}
