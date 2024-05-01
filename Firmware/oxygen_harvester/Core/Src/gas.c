/*
 * gas.c
 *
 *  Created on: Mar 23, 2024
 *      Author: Donatus
 */


#include "main.h"
#include "gas.h"
#include "ssd1306_funcs.h"
#include "stdio.h"

int gas_counter = 0;
int gas_counter2 = 0;
uint8_t gasRxBuffer[GASRXBUFSIZE];
uint32_t gasPressureBitBuffer[1];
GAS_t gas1;
char oled_buf2[100];

int check_buf(void)
{
	for (int i = 0; i < GASRXBUFSIZE; i++)
	{
		if ((gasRxBuffer[i] == 0x16) && (gasRxBuffer[i + 1] == 0x09) && (gasRxBuffer[i + 2] == 0x01))
			return 1;
		if (i > 0)
			break;
	}

	return 0;
}


void get_oxygen_params(void)
{
	gas_counter = 0;
	gas_counter2 = 3;

	if (check_buf())
	{
		while (gas_counter < 3)
		{
			gas1.gas_params[gas_counter] = (gasRxBuffer[gas_counter2 + gas_counter] * 256 + gasRxBuffer[(++gas_counter2) + gas_counter])/10.0;
			gas_counter++;
		}
	}

	else
	{
		while (gas_counter < 3)
		{
			gas1.gas_params[gas_counter] = 0;
			gas_counter++;
		}
	}
	gas_bit_to_bar();
}


void gas_bit_to_bar(void)
{
	/* pressure sensor output is 4-20mA and takes input of 10-30VDC
	 * pressure sensor can measure 0-16bar
	 * atmospheric pressure is 1.01325bar
	 * 4mA is 1.01325bar and we can say 20mA is 16bar
	 * We can assume the pressure-current relationship is linear and use a formula
	 * y = mx + c
	 * m = (20-4)/(16-1) = (1/3)[mA/bar]
	 * with a defined resistance r, m can be (r/3)[mV/bar]
	 * voltage read from the drop across the resistor can be converted to pressure in bar
	 * x = y/m
	 * adc is 12bit thus has values from 0 to 4096
	 * adc max input voltage is 3.3V thus 0int=0V and 4096int=3.3V
	 * adc int to voltage -> voltage = int * 3.3/4096
	 * Using a resistance r:
	 * 	0bar is 4ma*r -> 4ma * 150R = 0.6V
	 * 	16bar is 20ma*r -> 20ma * 150R = 3V
	 */
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	*gasPressureBitBuffer = HAL_ADC_GetValue(&hadc1);
	gas1.gas_pressure = ((*gasPressureBitBuffer) * (3.3 / 4096.0)) / (PRESSURE_R_DROP / 3.0);
}

void power_electrodes(int power_direction, int *electrode_power_status)
{
	HAL_GPIO_WritePin(Electrode1_Output_GPIO_Port, Electrode1_Output_Pin, power_direction);
	HAL_GPIO_WritePin(Electrode2_Output_GPIO_Port, Electrode2_Output_Pin, power_direction);
	HAL_GPIO_WritePin(Electrode3_Output_GPIO_Port, Electrode3_Output_Pin, power_direction);
	HAL_GPIO_WritePin(Electrode4_Output_GPIO_Port, Electrode4_Output_Pin, power_direction);
	*electrode_power_status = 1;
}

void manage_chambers(void)
{
	if (gas1.gas_pressure >= PRESSURE_THRESH && *electrode_power_status)
	{
		// stop operation as gas is full in reservoir
		display_message_overwrite("Pressure limit reached");
		power_electrodes(0, electrode_power_status);
		return;
	}

	if (gas1.gas_concentration < CONCENTRATION_THRESH && *electrode_power_status)
	{
		// possible leakage
		display_message_overwrite("Possible GAS leakage");
		power_electrodes(0, electrode_power_status);
		return;
	}

	if (gas1.gas_temperature >= TEMPERATURE_THRESH && *electrode_power_status)
	{
		// possible overheating
		display_message_overwrite("Possible Overheating");
		power_electrodes(0, electrode_power_status);
		return;
	}

	if (gas1.gas_pressure < PRESSURE_THRESH &&
			gas1.gas_temperature < TEMPERATURE_THRESH &&
			!(*electrode_power_status))
	{
		// gas parameters within threshold
		display_message_overwrite("Powering Electrodes");
		power_electrodes(1, electrode_power_status);
	}

}

void display_gas_parameters(void)
{
	sprintf(oled_buf2, "P: %.4fbar", gas1.gas_pressure);
	display_message_overwrite(oled_buf2);
	display1.cur_y += 10;
	sprintf(oled_buf2, "Conc: %.2f%%", gas1.gas_concentration);
	display_message(oled_buf2);
	display1.cur_y += 10;
	sprintf(oled_buf2, "Flow: %.2fL/min", gas1.gas_flowrate);
	display_message(oled_buf2);
	display1.cur_y += 10;
	sprintf(oled_buf2, "Temp: %.2fdegC", gas1.gas_temperature);
	display_message(oled_buf2);
}

