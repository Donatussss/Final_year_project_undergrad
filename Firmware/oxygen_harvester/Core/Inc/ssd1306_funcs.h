/*
 * ssd1306_funcs.h
 *
 *  Created on: Apr 19, 2024
 *      Author: user
 */

#ifndef INC_SSD1306_FUNCS_H_
#define INC_SSD1306_FUNCS_H_

#include <stdint.h>

typedef struct
{
	uint8_t cur_y;
	uint8_t cur_x;
	char oled_buf[100];
}Display_t;

extern Display_t display1;

void display_message(char *);
void display_message_overwrite(char *);

#endif /* INC_SSD1306_FUNCS_H_ */
