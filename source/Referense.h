/*
 * Referense.h
 *
 * Created: 22.02.2017 15:32:42
 *  Author: VLAD
 */ 

#ifndef REFERENSE_H_
#define REFERENSE_H_

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

#include <alloca.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "lcd_lib_2.h"


/*инициализация*/
 void Referense(void);
 void clear_array(char *str);
 void coppy_array(char *str1, const char *str2);

 int StrCompare(const char *str1, const char *str2);

 void UART_Send(const uint8_t data);
 void UART_SendStr(const char *str);
 char Command_token(char *str);

 void SPI_MassiveRead(uint8_t num, uint8_t chip_select, uint8_t *data);

#endif /* REFERENSE_H_ */