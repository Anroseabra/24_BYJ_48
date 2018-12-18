/*
 * serial.h
 *  Redirection of the printf stream to the AVR serial port
 *  Header file
 *  Created on: 13/09/2016
 *      Author: jpsousa@fe.up.pt
 */

#ifndef SERIAL_PRINTF_H_
#define SERIAL_PRINTF_H_

#include <stdio.h>

int usart_putchar(char c, FILE *stream);
void printf_init(void);

#endif /* SERIAL_H_ */
