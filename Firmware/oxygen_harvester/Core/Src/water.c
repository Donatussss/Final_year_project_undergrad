#include "main.h"
#include "water.h"
#include "gas.h"

void waterInitialization(){
	while(1){
		int initial_min_1 = HAL_GPIO_ReadPin(Float_Chamber1_min_GPIO_Port, Float_Chamber1_min_Pin);
		int initial_max_1 = HAL_GPIO_ReadPin(Float_Chamber1_max_GPIO_Port, Float_Chamber1_max_Pin);
		int initial_min_2 = HAL_GPIO_ReadPin(Float_Chamber2_min_GPIO_Port, Float_Chamber2_min_Pin);
		int initial_max_2 = HAL_GPIO_ReadPin(Float_Chamber2_max_GPIO_Port, Float_Chamber2_max_Pin);
		int initial_min_3 = HAL_GPIO_ReadPin(Float_Chamber3_min_GPIO_Port, Float_Chamber3_min_Pin);
		int initial_max_3 = HAL_GPIO_ReadPin(Float_Chamber3_max_GPIO_Port, Float_Chamber3_max_Pin);
		int initial_min_4 = HAL_GPIO_ReadPin(Float_Chamber4_min_GPIO_Port, Float_Chamber4_min_Pin);
		int initial_max_4 = HAL_GPIO_ReadPin(Float_Chamber4_max_GPIO_Port, Float_Chamber4_max_Pin);

		waterLevel_Chamber_1(initial_min_1, initial_max_1);
		waterLevel_Chamber_2(initial_min_2, initial_max_2);
		waterLevel_Chamber_3(initial_min_3, initial_max_3);
		waterLevel_Chamber_4(initial_min_4, initial_max_4);

		  if (initial_max_1 == 1 && initial_max_2 == 1 && initial_max_3 == 1 && initial_max_4 == 1) {
			break;
		  }
	}
	power_electrodes(1, electrode_power_status);
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
