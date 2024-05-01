#include "main.h"
#include "water.h"
#include "gas.h"
#include "ssd1306_funcs.h"

int *electrode_power_status;

void waterInitialization(){
	display_message_overwrite("Starting");
	display1.cur_y += 10;
	display_message("Initialization...");
	while(1){
		int initial_min_1 = HAL_GPIO_ReadPin(Float_Chamber1_min_GPIO_Port, Float_Chamber1_min_Pin);
		int initial_max_1 = HAL_GPIO_ReadPin(Float_Chamber1_max_GPIO_Port, Float_Chamber1_max_Pin);
		int initial_min_2 = HAL_GPIO_ReadPin(Float_Chamber2_min_GPIO_Port, Float_Chamber2_min_Pin);
		int initial_max_2 = HAL_GPIO_ReadPin(Float_Chamber2_max_GPIO_Port, Float_Chamber2_max_Pin);
		//		int initial_min_3 = HAL_GPIO_ReadPin(Float_Chamber3_min_GPIO_Port, Float_Chamber3_min_Pin);
		//		int initial_max_3 = HAL_GPIO_ReadPin(Float_Chamber3_max_GPIO_Port, Float_Chamber3_max_Pin);
		int initial_min_4 = HAL_GPIO_ReadPin(Float_Chamber4_min_GPIO_Port, Float_Chamber4_min_Pin);
		int initial_max_4 = HAL_GPIO_ReadPin(Float_Chamber4_max_GPIO_Port, Float_Chamber4_max_Pin);

		waterLevel_Chamber_1(initial_min_1, initial_max_1);
		waterLevel_Chamber_2(initial_min_2, initial_max_2);
		//		waterLevel_Chamber_3(initial_min_3, initial_max_3);
		waterLevel_Chamber_4(initial_min_4, initial_max_4);

		if (initial_max_1 == 1 && initial_max_2 == 1 && initial_max_4 == 1) {
			display_message_overwrite("Done Initialization...");
			break;
		}
	}
	power_electrodes(1, electrode_power_status);
}

void waterManagement(){
	int current_min_1 = HAL_GPIO_ReadPin(Float_Chamber1_min_GPIO_Port, Float_Chamber1_min_Pin);
	int current_max_1 = HAL_GPIO_ReadPin(Float_Chamber1_max_GPIO_Port, Float_Chamber1_max_Pin);
	int current_min_2 = HAL_GPIO_ReadPin(Float_Chamber2_min_GPIO_Port, Float_Chamber2_min_Pin);
	int current_max_2 = HAL_GPIO_ReadPin(Float_Chamber2_max_GPIO_Port, Float_Chamber2_max_Pin);
	//		int current_min_3 = HAL_GPIO_ReadPin(Float_Chamber3_min_GPIO_Port, Float_Chamber3_min_Pin);
	//		int current_max_3 = HAL_GPIO_ReadPin(Float_Chamber3_max_GPIO_Port, Float_Chamber3_max_Pin);
	int current_min_4 = HAL_GPIO_ReadPin(Float_Chamber4_min_GPIO_Port, Float_Chamber4_min_Pin);
	int current_max_4 = HAL_GPIO_ReadPin(Float_Chamber4_max_GPIO_Port, Float_Chamber4_max_Pin);

	waterLevel_Chamber_1(current_min_1, current_max_1);
	waterLevel_Chamber_2(current_min_2, current_max_2);
	//		waterLevel_Chamber_3(current_min_3, current_max_3);
	waterLevel_Chamber_4(current_min_4, current_max_4);
}

void waterLevel_Chamber_1(int chamberMin, int chamberMax){
	if(chamberMin == 0){
		HAL_GPIO_WritePin(Solenoid1_Output_GPIO_Port, Solenoid1_Output_Pin, 1);
	}
	if(chamberMax == 1){
		//digitalWrite(waterValve, LOW);
		HAL_GPIO_WritePin(Solenoid1_Output_GPIO_Port, Solenoid1_Output_Pin, 0);
	}
}

void waterLevel_Chamber_2(int chamberMin, int chamberMax){
	if(chamberMin == 0){
		HAL_GPIO_WritePin(Solenoid2_Output_GPIO_Port, Solenoid2_Output_Pin, 1);
	}
	if(chamberMax == 1){
		//digitalWrite(waterValve, LOW);
		HAL_GPIO_WritePin(Solenoid2_Output_GPIO_Port, Solenoid2_Output_Pin, 0);
	}
}

void waterLevel_Chamber_3(int chamberMin, int chamberMax){
	if(chamberMin == 0){
		HAL_GPIO_WritePin(Solenoid3_Output_GPIO_Port, Solenoid3_Output_Pin, 1);
	}
	if(chamberMax == 1){
		//digitalWrite(waterValve, LOW);
		HAL_GPIO_WritePin(Solenoid3_Output_GPIO_Port, Solenoid3_Output_Pin, 0);
	}
}

void waterLevel_Chamber_4(int chamberMin, int chamberMax){
	if(chamberMin == 0){
		HAL_GPIO_WritePin(Solenoid4_Output_GPIO_Port, Solenoid4_Output_Pin, 1);
	}
	if(chamberMax == 1){
		//digitalWrite(waterValve, LOW);
		HAL_GPIO_WritePin(Solenoid4_Output_GPIO_Port, Solenoid4_Output_Pin, 0);
	}
}
