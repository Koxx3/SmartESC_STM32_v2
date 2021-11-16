#include "motorcontrol.h"

#ifndef TEMPERATURE_H_
#define TEMPERATURE_H_

#define Q15		                       						32768L
#define Q16		                            				65536L

#define COOLING_CONSTANT                                    (int32_t)20
#define HEATING_CONSTANT                                    (int32_t)600
#define HARD_BRAKE_HEATING_CONSTANT                         (int32_t)300

#define FILTER_FREQUENCY_F32_005HZ     						(float)(0.9968633)    // 0.05 Hertz
#define FILTER_FREQUENCY_Q15_005HZ     						(int16_t)(Q15*FILTER_FREQUENCY_F32_005HZ)    // 0.05 Hertz

#define ABS(A) (((A) < 0) ? (-(A)) : (A))
#define IIR_FILTER_Q31(signal_in, signal_out, filter_constant) signal_out = signal_in + ((filter_constant * (signal_out - signal_in)) >> 15); if((!signal_in) && (ABS(signal_out) < 20)) { signal_out = 0; }

extern PWMC_Handle_t * pwmcHandle[1];

typedef struct{
	int32_t MosfetEstimatedTemperature[6];
	int32_t HottestMosfetTempersture;
	int32_t Temperature;
	int32_t temperature;
	int16_t HottestMosfetTemperstureDifference;
}TEMPERATURE_SENSOR_DATA;

int32_t GetTemperatureSensorData(TEMPERATURE_SENSOR_DATA* temperature_sensor_data);

#endif /* APP_PRODUCT_H_ */
