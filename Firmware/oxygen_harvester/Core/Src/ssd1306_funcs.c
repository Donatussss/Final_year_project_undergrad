/*
 * ssd1306_funcs.c
 *
 *  Created on: Apr 19, 2024
 *      Author: user
 */

#ifndef SRC_SSD1306_FUNCS_C_
#define SRC_SSD1306_FUNCS_C_

// max oled characters in 7x10 font: 18
#include "ssd1306_funcs.h"
#include "stdio.h"
#include "ssd1306.h"

Display_t display1;

void display_message(char *message)
{
	ssd1306_SetCursor(display1.cur_x, display1.cur_y);
	sprintf(display1.oled_buf, "%s", message);
    ssd1306_WriteString(display1.oled_buf, Font_7x10, White);
    ssd1306_UpdateScreen();
}

void display_message_overwrite(char *message)
{
	ssd1306_Fill(Black);
	display1.cur_y = 0;
	display1.cur_x = 0;
	ssd1306_SetCursor(display1.cur_x, display1.cur_y);
	sprintf(display1.oled_buf, "%s", message);
    ssd1306_WriteString(display1.oled_buf, Font_7x10, White);
    ssd1306_UpdateScreen();
}

#endif /* SRC_SSD1306_FUNCS_C_ */
