/*
 * gas.h
 *
 *  Created on: Mar 23, 2024
 *      Author: Donatus
 */

#ifndef INC_GAS_H_
#define INC_GAS_H_

#define GASRXBUFSIZE 12              /*uart*/
#define PRESSURE_R_DROP 150.0        /*oxygen_pressure resistor dropper*/
#define CONCENTRATION_THRESH 20.0	 /*percentage lower limit*/
#define TEMPERATURE_THRESH 60.0	     /*degrees celsius upper limit*/
#define PRESSURE_THRESH 5.0          /*bar*/

typedef union
{
	struct
	{
		float gas_concentration;
		float gas_flowrate;
		float gas_temperature;
		float gas_pressure;
	};
	float gas_params[4];
}GAS_t;

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern I2C_HandleTypeDef hi2c2;
extern ADC_HandleTypeDef hadc1;
extern GAS_t gas1;
extern uint8_t gasRxBuffer[GASRXBUFSIZE];

int check_buf(void);
void get_oxygen_params(void);
void gas_bit_to_bar(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void power_electrodes(int, int *);
void manage_chambers(void);
void display_gas_parameters(void);

#endif /* INC_GAS_H_ */
