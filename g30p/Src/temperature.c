#include "temperature.h"
#include "mc_interface.h"
#include "defines.h"
#include "drive_parameters.h"
#include "mc_config.h"

void get_mosfet_on_time(int32_t on_time[6]) {
	uint16_t min_duty_cycle, max_duty_cycle;
	uint8_t i, min_duty_cycle_phase, max_duty_cycle_phase, mid_duty_cycle_phase;
	uint16_t duty_cylce[3];

	if(true)//Check if motor is on
	{
		min_duty_cycle = (uint16_t)(Q16-1);
		max_duty_cycle = 0;

		for(i = 0; i < 3; i++)
		{
			if(i == 0){
				duty_cylce[i] = pwmcHandle[0]->CntPhA;
			} else if(i == 1){
				duty_cylce[i] = pwmcHandle[0]->CntPhB;
			} else {
				duty_cylce[i] = pwmcHandle[0]->CntPhC;
			}

			if(duty_cylce[i] < min_duty_cycle)
			{
				min_duty_cycle = duty_cylce[i];
				min_duty_cycle_phase = i;
			}
			if(duty_cylce[i] > max_duty_cycle)
			{
				max_duty_cycle = duty_cylce[i];
				max_duty_cycle_phase = i;
			}
		}
		if(min_duty_cycle_phase == max_duty_cycle_phase)
		{
			min_duty_cycle_phase = 0;
			max_duty_cycle_phase = 1;
		}
		if((min_duty_cycle_phase + max_duty_cycle_phase) == 1)
		{
			mid_duty_cycle_phase = 2;
		}
		else if((min_duty_cycle_phase + max_duty_cycle_phase) == 2)
		{
			mid_duty_cycle_phase = 1;
		}
		else
		{
			mid_duty_cycle_phase = 0;
		}
		for(i = 0; i < 3; i++)
		{
			on_time[i] = (int32_t)((duty_cylce[i] - duty_cylce[min_duty_cycle_phase]))/72;
		}

		on_time[3 + max_duty_cycle_phase] = 0;
		on_time[3 + min_duty_cycle_phase] = (int32_t) ((duty_cylce[max_duty_cycle_phase] - duty_cylce[min_duty_cycle_phase]))/72;
		on_time[3 + mid_duty_cycle_phase] = (int32_t) ((duty_cylce[max_duty_cycle_phase] - duty_cylce[mid_duty_cycle_phase]))/72;
	} else {
		for(i = 0; i < 6; i++)
		{
			on_time[i] = 0;
		}
	}
}

int32_t GetTemperatureSensorData(TEMPERATURE_SENSOR_DATA* temperature_sensor_data)
{
	int32_t deltaT, maxT = 0;
	uint8_t mosfet, index;
	int32_t mosfet_on_time[6];
	int32_t total_power = (float)pMPM[M1]->_super.hAvrgElMotorPowerW/(float)VBS_GetAvBusVoltage_V(pMCT[M1]->pBusVoltageSensor);
	int32_t motor_rpm = ((int32_t)pMCI[M1]->pSTC->SPD->open_speed);

	temperature_sensor_data->temperature = 25 << 9;
	IIR_FILTER_Q31(temperature_sensor_data->temperature, temperature_sensor_data->Temperature, FILTER_FREQUENCY_Q15_005HZ);

	get_mosfet_on_time(mosfet_on_time);
	total_power >>= 9;

	for(mosfet = 0; mosfet < 6; mosfet++)
	{
		index = mosfet;
		deltaT = temperature_sensor_data->Temperature - temperature_sensor_data->MosfetEstimatedTemperature[index];
		if(deltaT < 0)
		{
			temperature_sensor_data->MosfetEstimatedTemperature[index] += (((deltaT * COOLING_CONSTANT) - ((int32_t)Q15)) >> 15);
		}
		else if(deltaT > 0)
		{
			temperature_sensor_data->MosfetEstimatedTemperature[index] += (((deltaT * COOLING_CONSTANT) + ((int32_t)Q15)) >> 15);
		}

		/*if(GlobalData.CmdMap.device_state_h & MOTOR_HARD_BRAKE)
		{
			temperature_sensor_data->MosfetEstimatedTemperature[index] += ((((ABS(total_power)) * mosfet_on_time[motor][mosfet]) * HARD_BRAKE_HEATING_CONSTANT) >> SHIFT_15);
		}
		else
		{*/
			temperature_sensor_data->MosfetEstimatedTemperature[index] += ((((ABS(total_power/(motor_rpm+1))) * mosfet_on_time[mosfet]) * HEATING_CONSTANT) >> 15);
		/*}*/

		if(temperature_sensor_data->MosfetEstimatedTemperature[index] > maxT)
		{
			maxT = temperature_sensor_data->MosfetEstimatedTemperature[index];
		}
	}

	temperature_sensor_data->HottestMosfetTempersture = maxT;
	temperature_sensor_data->HottestMosfetTemperstureDifference = (int16_t)(maxT - temperature_sensor_data->Temperature);

	return 1;
}
