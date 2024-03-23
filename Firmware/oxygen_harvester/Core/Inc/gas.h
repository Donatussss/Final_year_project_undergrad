/*
 * gas.h
 *
 *  Created on: Mar 23, 2024
 *      Author: Donatus
 */

#ifndef INC_GAS_H_
#define INC_GAS_H_

#define RXBUFSIZE 12                 /*uart*/
#define MAINBUFSIZE 12               /*uart*/
#define PRESSURE_RXBUFSIZE 12        /*uart*/
#define PRESSURE_R_DROP 150          /*oxygen_pressure resistor dropper*/
#define CONCENTRATION_THRESH 20 * 10 /*percentage lower limit*/
#define FLOWRATE_UPPER_THRESH 5 * 10 /*litres per minute*/
#define FLOWRATE_LOWER_THRESH 1      /*0.1 litres per minute*/
#define TEMPERATURE_THRESH 60 * 10   /*degrees celsius upper limit*/
#define PRESSURE_THRESH 5            /*bar*/
#define o2conc_ind 0
#define o2flow_ind 1
#define o2temp_ind 2

int check_main_buf(uint8_t *rx_buf, uint8_t *buf, uint16_t size);
int modder(int num);
void get_oxygen_params(uint8_t *rx_buf, uint8_t *buf_p, uint16_t *oxygen_params);
void gas_bit_to_bar(uint32_t gas_bit, uint32_t *gas_pressure);
void power_electrodes(int power_direction, int *electrode_power_status);

#endif /* INC_GAS_H_ */
