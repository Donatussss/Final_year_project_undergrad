/*
 * gas.c
 *
 *  Created on: Mar 23, 2024
 *      Author: Donatus
 */


#include "main.h"
#include "gas.h"


int check_main_buf(uint8_t *rx_buf, uint8_t *buf_p, uint16_t buf_size)
{
    int buf_ok = -1;
    for (int i = 0; i < buf_size * 2; i++)
    {
        if ((rx_buf[i % buf_size] == 0x16) && (rx_buf[(i + 1) % buf_size] == 0x09) && (rx_buf[(i + 2) % buf_size] == 0x01))
        {
            buf_ok = i % buf_size;
            break;
        }
    }

    return (buf_ok);
}


int modder(int num)
{
    return (num % MAINBUFSIZE);
}


void get_oxygen_params(uint8_t *rx_buf, uint8_t *buf_p, uint16_t *oxygen_params)
{
    int buf_ok = check_main_buf(rx_buf, buf_p, MAINBUFSIZE);

    if (buf_ok > 0)
    {
        oxygen_params[0] = buf_p[modder(buf_ok + 3)] * 256 + buf_p[modder(buf_ok + 4)];
        oxygen_params[1] = buf_p[modder(buf_ok + 5)] * 256 + buf_p[modder(buf_ok + 6)];
        oxygen_params[2] = buf_p[modder(buf_ok + 7)] * 256 + buf_p[modder(buf_ok + 8)];
    }

    else
    {
        oxygen_params[0] = 0;
        oxygen_params[1] = 0;
        oxygen_params[2] = 0;
    }
}


void gas_bit_to_bar(uint32_t gas_bit, uint32_t *gas_pressure)
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
     * adc max input voltage is 3.6V thus 0int=0V and 4096int=3.6V
     * adc int to voltage -> voltage = int * 3.6/4096
     * Using a resistance r:
     * 	0bar is 4ma*r -> 4ma * 150R = 0.6V
     * 	16bar is 20ma*r -> 20ma * 150R = 3V
     */

    *gas_pressure = (gas_bit * (3.6 / 4096)) / (PRESSURE_R_DROP / 3);
}


void power_electrodes(int power_direction, int *electrode_power_status)
{
	HAL_GPIO_WritePin(Electrode1_Output_GPIO_Port, Electrode1_Output_Pin, power_direction);
	HAL_GPIO_WritePin(Electrode2_Output_GPIO_Port, Electrode2_Output_Pin, power_direction);
	HAL_GPIO_WritePin(Electrode3_Output_GPIO_Port, Electrode3_Output_Pin, power_direction);
	HAL_GPIO_WritePin(Electrode4_Output_GPIO_Port, Electrode4_Output_Pin, power_direction);
	*electrode_power_status = 1;
}
