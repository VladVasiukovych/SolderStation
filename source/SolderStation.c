/*
* SolderStation.c
*
* Created: 22.02.2017 15:36:08
*  Author: VLAD
*/
#include "Referense.h"
#define PIN_MASK 0b00000011        // Маска для сравнения с PINC
#define PIN_MASK2 0b00001100       // Маска для сравнения с PINC
/*==============================================================================================================*/
double EEMEM dt_solder_eeprom = 30.0;		//адрес в eeprom для коэффициента dt_solder
double EEMEM dt_fan_eeprom = 0.0;			//адрес в eeprom для коэффициента dt_solder
double dt_solder, dt_fan;

double EEMEM S_Kp_eeprom = 20.0;			//адрес в eeprom для коэффициента
double EEMEM S_Ki_eeprom = 0.0;			//адрес в eeprom для коэффициента
double EEMEM S_Kd_eeprom = 20.0;			//адрес в eeprom для коэффициента
double S_Kp, S_Ki, S_Kd;

double EEMEM F_Kp_eeprom = 20.0;			//адрес в eeprom для коэффициента
double EEMEM F_Ki_eeprom = 0.0;			//адрес в eeprom для коэффициента
double EEMEM F_Kd_eeprom = 20.0;			//адрес в eeprom для коэффициента
double F_Kp, F_Ki, F_Kd;

uint16_t EEMEM T_solder_eeprom = 0;			//адрес в eeprom для температуры паяльника
uint16_t EEMEM T_fan_eeprom = 0;
int16_t T_solder_set, T_fan_set;
/*==============================================================================================================*/
volatile uint8_t rx_flag = 0, rx_byte_num = 0;
volatile uint16_t fan_counter = 0, display_refresh = 0, pid_refresh = 0;

char command_buff[24];

uint16_t adc_result;
int16_t T_solder_set = 0, T_fan_set = 0;


/*==============================================================================================================*/
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/

/*=======================================================================================================================================================================================*/
ISR(INT0_vect)
{
	GICR &= !(1<<INT0);		//Запретить внешние прерывние на выводе INT0
	GIFR &= !(1<<INTF0);	//Сброс флага прерывания
	TCNT1 = 0;
	GICR |= (1<<INT0);		//Разрешить внешние прерывние на выводе INT0
}
/*=======================================================================================================================================================================================*/
ISR(ADC_vect)	//прерывание по завершению преобразования АЦП
{
	static uint8_t adc_counter = 0;
	static uint32_t adc_temp = 0;

	adc_temp = adc_temp + ADC;
	adc_counter++;

	if (adc_counter >= 150)
	{
		adc_result = adc_temp / adc_counter;
		adc_counter = 0;
		adc_temp = 0;
	}

}
/*=======================================================================================================================================================================================*/
ISR(TIMER0_OVF_vect)	//переполнение каждые 0.000256 сек	делитель /8
{
	display_refresh++;		//увеличение счетчика для обновления дисплея
	pid_refresh++;
	fan_counter++;
	//-----------------------------------------------------------------------------------------------------------
	if (rx_flag)
	{
		rx_flag++;
	}
	//rx_flag = (rx_flag > 0) ? 1 : 0;			//тоже самое что и выше, но удобнее в записи
	//rx_flag = (rx_flag) ? rx_flag++ : NULL;
	//-----------------------------------------------------------------------------------------------------------
	/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
	static uint8_t next_state = 0, prev_state = 0, up_state = 0, down_state = 0;

	next_state = PINC & PIN_MASK; // Считываем текущее значение битов
	if (next_state != prev_state)
	{
		switch (prev_state)
		{
			case 2:
			{
				if (next_state == 3) up_state++;
				if (next_state == 0) down_state++;
				break;
			}
			case 0:
			{
				if (next_state == 2) up_state++;
				if (next_state == 1) down_state++;
				break;
			}
			case 1:
			{
				if (next_state == 0) up_state++;
				if (next_state == 3) down_state++;
				break;
			}
			case 3:
			{
				if (next_state == 1) up_state++;
				if (next_state == 2) down_state++;
				break;
			}
			default:
			{
				break;
			}
		}
		prev_state = next_state;    // Текущее состояние становится предыдущим
		//-----------------------------------------------------------------------------------------------------------
		if (up_state >= 4)            // 1 раз за 4 импульса изменяем состояние передачи (поменять на 2 для другого типа)
		{
			T_solder_set += 5;
			if (T_solder_set > 420)
			{
				T_solder_set = 420;
			}
			up_state = 0;
			eeprom_write_word (&T_solder_eeprom, T_solder_set);
		}
		if (down_state >= 4)
		{
			T_solder_set -= 5;
			if (T_solder_set < 0)
			{
				T_solder_set = 0;
			}
			down_state = 0;
			eeprom_write_word (&T_solder_eeprom, T_solder_set);
		}
	}
	/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
	static uint8_t next_state2, prev_state2, up_state2, down_state2;

	next_state2 = PINC & PIN_MASK2; // Считываем текущее значение битов
	if (next_state2 != prev_state2)
	{
		switch (prev_state2)
		{
			case 8:
			{
				if (next_state2 == 12) up_state2++;
				if (next_state2 == 0) down_state2++;
				break;
			}
			case 0:
			{
				if (next_state2 == 8) up_state2++;
				if (next_state2 == 4) down_state2++;
				break;
			}
			case 4:
			{
				if (next_state2 == 0) up_state2++;
				if (next_state2 == 12) down_state2++;
				break;
			}
			case 12:
			{
				if (next_state2 == 4) up_state2++;
				if (next_state2 == 8) down_state2++;
				break;
			}
			default:
			{
				break;
			}
		}
		prev_state2 = next_state2;    // Текущее состояние становится предыдущим
	}
	//-----------------------------------------------------------------------------------------------------------
	if (up_state2 >= 4)            // 1 раз за 4 импульса изменяем состояние передачи (поменять на 2 для другого типа)
	{
		T_fan_set += 5;
		if (T_fan_set > 500)
		{
			T_fan_set = 500;
		}
		up_state2 = 0;
		eeprom_write_word (&T_fan_eeprom, T_fan_set);
	}
	if (down_state2 >= 4)
	{
		T_fan_set -= 5;
		if (T_fan_set < 0)
		{
			T_fan_set = 0;
		}
		down_state2 = 0;
		eeprom_write_word (&T_fan_eeprom, T_fan_set);
	}
}
/*=======================================================================================================================================================================================*/
//*** Прием данных по UART ***//
ISR(USART_RXC_vect)
{
	if (rx_flag == 0){rx_flag = 1;}
	command_buff[rx_byte_num] = UDR;
	rx_byte_num++;
}

//*** Распознование команд ***//
char Command_token(char *str)
{
	char *pch;
	pch = strtok (str," ,.-");
	char string[100];
	

	if (strcmp(pch, "dt_solder") == 0)
	{
		sprintf(string, "dt_solder save!\t\t old_dt_solder= %4.2f\t", dt_solder);		UART_SendStr(string);
		
		pch = strtok (NULL, " ");
		dt_solder =  atof(pch);													//перевод строки в double
		eeprom_write_block(&dt_solder, &dt_solder_eeprom, sizeof(double));		//запсиь коэффициента dt_solder в ПЗУ

		sprintf(string, "new_dt_solder= %4.2F\t\r", dt_solder);			UART_SendStr(string);
	}
	else if (strcmp(pch, "dt_fan") == 0)
	{
		sprintf(string, "dt_fan save!\t\t dt_fan= %4.2f\t", dt_fan);		UART_SendStr(string);
		
		pch = strtok (NULL, " ");
		dt_fan =  atof(pch);													//перевод строки в double
		eeprom_write_block(&dt_fan, &dt_fan_eeprom, sizeof(double));		//запсиь коэффициента dt_solder в ПЗУ

		sprintf(string, "new_dt_fan= %4.2F\t\r", dt_fan);			UART_SendStr(string);
	}
	else if (strcmp(pch, "S_Kp") == 0)
	{
		sprintf(string, "S_Kp save!\t\t old_S_Kp= %4.4f\t\t\t", S_Kp);		UART_SendStr(string);
		
		pch = strtok (NULL, " ");
		S_Kp =  atof(pch);													//перевод строки в double
		eeprom_write_block(&S_Kp, &S_Kp_eeprom, sizeof(double));		//запсиь коэффициента S_Kp в ПЗУ

		sprintf(string, "new_S_Kp= %4.4F\t\r", S_Kp);			UART_SendStr(string);
	}
	else if (strcmp(pch, "S_Ki") == 0)
	{
		sprintf(string, "S_Ki save!\t\t old_S_Ki= %4.4f\t\t\t", S_Ki);		UART_SendStr(string);
		
		pch = strtok (NULL, " ");
		S_Ki =  atof(pch);													//перевод строки в double
		eeprom_write_block(&S_Ki, &S_Ki_eeprom, sizeof(double));		//запсиь коэффициента S_Ki в ПЗУ

		sprintf(string, "new_S_Ki= %4.4F\t\r", S_Ki);			UART_SendStr(string);
	}
	else if (strcmp(pch, "S_Kd") == 0)
	{
		sprintf(string, "S_Kd save!\t\t old_S_Kd= %4.4f\t\t\t", S_Kd);		UART_SendStr(string);
		
		pch = strtok (NULL, " ");
		S_Kd =  atof(pch);													//перевод строки в double
		eeprom_write_block(&S_Kd, &S_Kd_eeprom, sizeof(double));		//запсиь коэффициента S_Kd в ПЗУ

		sprintf(string, "new_S_Kd= %4.4F\t\r", S_Kd);			UART_SendStr(string);
	}
	else if (strcmp(pch, "F_Kp") == 0)
	{
		sprintf(string, "F_Kp save!\t\t old_F_Kp= %4.4f\t\t\t", F_Kp);		UART_SendStr(string);
		
		pch = strtok (NULL, " ");
		F_Kp =  atof(pch);													//перевод строки в double
		eeprom_write_block(&F_Kp, &F_Kp_eeprom, sizeof(double));		//запсиь коэффициента S_Kp в ПЗУ

		sprintf(string, "new_F_Kp= %4.4F\t\r", F_Kp);			UART_SendStr(string);
	}
	else if (strcmp(pch, "F_Ki") == 0)
	{
		sprintf(string, "F_Ki save!\t\t old_F_Ki= %4.4f\t\t\t", F_Ki);		UART_SendStr(string);
		
		pch = strtok (NULL, " ");
		F_Ki =  atof(pch);													//перевод строки в double
		eeprom_write_block(&F_Ki, &F_Ki_eeprom, sizeof(double));		//запсиь коэффициента S_Ki в ПЗУ

		sprintf(string, "new_F_Ki= %4.4F\t\r", F_Ki);			UART_SendStr(string);
	}
	else if (strcmp(pch, "F_Kd") == 0)
	{
		sprintf(string, "F_Kd save!\t\t old_F_Kd= %4.4f\t\t\t", F_Kd);		UART_SendStr(string);
		
		pch = strtok (NULL, " ");
		F_Kd =  atof(pch);													//перевод строки в double
		eeprom_write_block(&F_Kd, &F_Kd_eeprom, sizeof(double));		//запсиь коэффициента S_Kd в ПЗУ

		sprintf(string, "new_SF_Kd= %4.4F\t\r", F_Kd);			UART_SendStr(string);
	}
	
	return 0;
}
/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
void SolderPid (double t_current, int16_t t_set)
{
	int16_t PID;
	double error;
	double U;
	static double last_error = 0.0;
	static double I = 0.0;


	error = t_set - t_current;		//получение текущей ошибки


	U = I + S_Ki * error;
	
	I = U;
	if (I > 1000){I = 1000;}
	if (I < -1000){I = -1000;}


	U += S_Kp * error;


	U += S_Kd * (error - last_error);
	last_error = error;


	PID = 1023 - U;


	if (PID > 1023){PID = 1023;}
	else if (PID < 0){PID = 0;}
	OCR1B = PID;
}


void FanPid (double t_current, int16_t t_set)
{
	int16_t PID;
	double error;
	double U;
	static double last_error = 0.0;
	static double I = 0.0;


	error = t_set - t_current;		//получение текущей ошибки


	U = I + S_Ki * error;
	
	I = U;
	if (I > 1000){I = 1000;}
	if (I < -1000){I = -1000;}


	U += S_Kp * error;


	U += S_Kd * (error - last_error);
	last_error = error;


	PID = 1023 - U;


	if (PID > 1023){PID = 1023;}
	else if (PID < 500){PID = 500;}
	OCR1A = PID;
}


/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
int main(void)
{
	/*
	Вызов из разных обработчиков прерываний одной функции, 
	в которой при помощи if определяется прерывание и выполняется нужные действия.
	*/
	
	OCR1B = 1023;
	OCR1A = 1023;
	OCR2 = 255;
	
	Referense();
	LCD_Init();
	//=================================================================
	eeprom_read_block(&dt_solder, &dt_solder_eeprom, sizeof(double));
	eeprom_read_block(&dt_fan, &dt_fan_eeprom, sizeof(double));
	
	eeprom_read_block(&S_Kp, &S_Kp_eeprom, sizeof(double));		//чтение коэффициента S_Kp из ПЗУ
	eeprom_read_block(&S_Ki, &S_Ki_eeprom, sizeof(double));		//чтение коэффициента S_Ki из ПЗУ
	eeprom_read_block(&S_Kd, &S_Kd_eeprom, sizeof(double));		//чтение коэффициента S_Kd из ПЗУ
	
	eeprom_read_block(&F_Kp, &F_Kp_eeprom, sizeof(double));		//чтение коэффициента S_Kp из ПЗУ
	eeprom_read_block(&F_Ki, &F_Ki_eeprom, sizeof(double));		//чтение коэффициента S_Ki из ПЗУ
	eeprom_read_block(&F_Kd, &F_Kd_eeprom, sizeof(double));		//чтение коэффициента S_Kd из ПЗУ
	
	T_solder_set = eeprom_read_word (&T_solder_eeprom);
	T_fan_set = eeprom_read_word (&T_fan_eeprom);
	//=================================================================
	
	char LcdBuffer[16];

	double Rthermistor;
	double Rmin;
	double divider;
	
	double T_solder = 0.0;
	double T_fan = 0.0;
	uint16_t t_set = 0, t_set2 = 0;
	
	uint8_t fan_buf[2];
	uint32_t temp_buff;
	
	//float мантисса 23бит (8 388 608)	порядок 8бит (256)
	#define a 17.50959E-3
	#define b -4.254383E-3
	#define c 40.57705E-6
	
	wdt_enable(WDTO_500MS);		//установка сторожевого таймера на счет в течении 500мс
	
	//int n=344;
	//char * buffer = (char*) malloc (n);
	//free (buffer);
	
//	register d = 56;
	
	while(1)
	{
		wdt_reset();	//сброс сторожевого таймера
		
		//=================================================================		//передача через UART
		if(rx_flag > 160)	//счет, пока не пройдет 41 ms
		{
			//UART_SendStr(command_buff);		UART_SendStr("\r\r");

			rx_flag = 0;
			rx_byte_num = 0;

			Command_token(command_buff);
			memset(command_buff, 0x00, 24);
		}
		//=================================================================		//температура фена
		if (fan_counter > 780)	//ниже паузы в 200 мсек - зависает
		{
			fan_counter = 0;
			//----------------------------------------------------------------------------------------------			
			for(uint8_t i=0; i < 128; i++)	//	(< 128) - выполнение 128 раз,		(<= 128) - выполнение 129 раз
			{
				SPI_MassiveRead(2, 4, fan_buf);
				temp_buff = temp_buff + ((fan_buf[0] << 5) | (fan_buf[1] >> 3));
			}
			T_fan =  ((temp_buff >> 7) * 0.25) - dt_fan;
			temp_buff = 0;
			//----------------------------------------------------------------------------------------------			
			//if ((fan_buf[1] & (1 << 2)))	//Проверка на отсутствие термопары (2 бит=1, если её нет)
			if (T_fan > 1020)
			{
				T_fan = 0.0;
			}
			//----------------------------------------------------------------------------------------------
			if ((PINB & (1 << PB2)) == 0)	//FanSW1 (тумблер вверх)
			{
				PORTC |= (1<<PC4);
				t_set2 = T_fan_set;
				//----------------------------------------------------------------------------------------------
				ADCSRA = (0<<ADEN)|(0<<ADSC)|(0<<ADATE)|(0<<ADIF)|(0<<ADIE)|(0<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);	//делитель частоты на 8
				ADMUX = (0<<REFS1)|(0<<REFS0)|(1<<ADLAR)|(0<<MUX4)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(1<<MUX0);
				_delay_us(10);
				ADCSRA |= (1<<ADEN)|(1<<ADSC);
				while(!(ADCSRA & (1<<ADIF)));
				OCR2 = ~ADCH;
				
				ADMUX = (0<<REFS1)|(0<<REFS0)|(0<<ADLAR)|(0<<MUX4)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0);
				_delay_us(10);
				ADCSRA = (1<<ADEN)|(1<<ADSC)|(1<<ADATE)|(0<<ADIF)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0);	//делитель частоты на 64
				//----------------------------------------------------------------------------------------------
				FanPid(T_fan, t_set2);
			}
			else if ((PINB & (1 << PB3)) == 0)	//FanSW2
			{
				t_set2 = 0;
				OCR1A = 1023;
				
				if (T_fan > 100)
				{
					OCR2 = 25;
					PORTC |= (1<<PC4);
				}
				else if (T_fan < 65)
				{
					OCR2 = 255;
					PORTC &= ~(1<<PC4);
				}
			}
			else if ((PINB & (1 << PB2)) && (PINB & (1 << PB3)))
			{
				OCR2 = 200;
				PORTC |= (1<<PC4);
				t_set2 = 200;				

				FanPid(T_fan, t_set2);
			}
			//----------------------------------------------------------------------------------------------
			if (T_fan > 510){OCR1A = 1023;}
		}
		//=================================================================		//пид паяльник
		if (pid_refresh > 390) //обновление каждые 50 ms
		{
			pid_refresh = 0;			
			//----------------------------------------------------------------------------------------------
			divider = (3.788 - 0.178) / 100;
			Rmin = 50 - (0.178 / divider);
			Rthermistor = (adc_result * 4) / (1023 * divider) + Rmin;			
			//----------------------------------------------------------------------------------------------
			T_solder = log(Rthermistor);
			T_solder = 1/(a + b*T_solder + c*T_solder*T_solder*T_solder);
			T_solder = T_solder - 273.15 - dt_solder;			
			//----------------------------------------------------------------------------------------------			
			if ((PINA & (1 << PA3)) == 0)	//SolderSW1 (тумблер вверх)
			{
				t_set = T_solder_set;
				SolderPid(T_solder, t_set);
			}
			else if ((PINB & (1 << PB0)) == 0)	//SolderSW2
			{
				t_set = 0;
				OCR1B = 1023;
			}
			else
			{
				t_set =200;
				SolderPid(T_solder, t_set);
			}
			//----------------------------------------------------------------------------------------------
			if(T_solder > 430)
			{
				OCR1B = 1023;
			}
			//OCR1B = (T_solder > 430) ? 1023 : OCR1B;
		}
		//=================================================================		//Обновление экрана
		if (display_refresh > 900) //обновление экрана каждые 230 ms
		{
			display_refresh = 0;
			LCD_Clear();
			
			LCD_Goto(0,0);
			sprintf(LcdBuffer, "%i  %.1f  %i", t_set, T_solder, OCR1B);
			LCD_SendStr(LcdBuffer);
			//==============================================================
			
			LCD_Goto(0,1);
			sprintf(LcdBuffer, "%i  %.1f  %i", t_set2, T_fan, OCR1A);
			LCD_SendStr(LcdBuffer);
		}
		//=================================================================
		_delay_us(100);
	}
	return 1;
}


/*

strcat(string, string2);		//объединение строк
UART_Send(0x0D);				//перевод на новую строку



T_solder = ((adc_result * 4) / (1023 * divider)) * 0.205 + Rmin - dt_solder;


if(strstr(reciv_buf, "vkld13") != NULL)




//PORTD ^= 0b01000000;	//произвести XOR с 7 битом порта C


void SolderPid (double t_current, int16_t t_set)
{
int16_t PID;
double error;
double U;
static double last_error = 0.0;
//static double I = 0.0;


error = t_set - t_current;		//получение текущей ошибки


U = sI + S_Ki * error;

sI = U;
if (sI > 1000){sI = 1000;}
if (sI < -1000){sI = -1000;}


U += S_Kp * error;


U += S_Kd * (error - last_error);
last_error = error;


PID = 1023 - U;


if (PID > 1023){PID = 1023;}
else if (PID < 0){PID = 0;}
OCR1B = PID;
}


*/



