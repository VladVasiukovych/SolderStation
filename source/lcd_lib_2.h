//***************************************************************************
//  File........: lcd_lib.h
//
//  Author(s)...: Pashgan    chipenable.ru
//
//  Target(s)...: ATMega...
//
//  Compiler....: IAR, CodeVision, GNU GCC
//
//  Description.: Драйвер знакосинтезирующего жк дисплея
//
//  Data........: 20.07.13  
//
//***************************************************************************

#ifndef LCD_LIB_H
#define LCD_LIB_H

#include "compilers_4.h"

#define VIRT_PORT
#ifdef VIRT_PORT

//здесь я определил виртуальный порт

//шина данных LCD
#define LCD_PORT  LCD_DATA, F, _VIRT

#define LCD_DATA_0  D, 0, _NONE
#define LCD_DATA_1  D, 1, _NONE
#define LCD_DATA_2  D, 2, _NONE
#define LCD_DATA_3  D, 5, _NONE
#define LCD_DATA_4  A, 4, _HI
#define LCD_DATA_5  A, 5, _HI
#define LCD_DATA_6  A, 6, _HI
#define LCD_DATA_7  A, 7, _HI

//управляющие выводы LCD
#define LCD_RS  C, 7, _HI
#define LCD_RW  C, 6, _HI
#define LCD_EN  C, 5, _HI  

#else

//а здесь я определил реальный порт

//шина данных LCD
#define LCD_PORT  LCD_DATA, A, _REAL

#define LCD_DATA_0  A, 0, _HI
#define LCD_DATA_1  A, 1, _HI
#define LCD_DATA_2  A, 2, _HI
#define LCD_DATA_3  A, 3, _HI
#define LCD_DATA_4  A, 4, _HI
#define LCD_DATA_5  A, 5, _HI
#define LCD_DATA_6  A, 6, _HI
#define LCD_DATA_7  A, 7, _HI

//управляющие выводы LCD
#define LCD_RS  D, 5, _HI
#define LCD_RW  D, 6, _HI
#define LCD_EN  D, 7, _HI

#endif

/*____________________________________________________________________*/

//глобальные настройки драйвера
#define LCD_CHECK_FL_BF             1	//проверять флаг BF или использовать программную задержку. 0 - задержка, 1 - проверка флага
#define LCD_BUS_4_8_BIT             0	//используемая шина данных. 0 - 4 разрядная шина, 1 - 8-ми разрядная

//настройки инициализации дисплея
#define LCD_ONE_TWO_LINE            1	//количество отображаемых строк. 0 - 1 строка; 1 - 2 строки
#define LCD_FONT58_FONT511          0	//тип шрифта. 0 - 5х8 точек; 1 - 5х11 точек
#define LCD_DEC_INC_DDRAM           1	//изменения адреса ОЗУ при выводе на дисплей. 0 - курсор движется влево, адрес уменьшается на 1 (текст получается задом наперед) ; 1 - курсор движется вправо, адрес увеличивается на 1
#define LCD_SHIFT_RIGHT_LEFT        0	//сдвиг всего дисплея. 0 - при чтении ОЗУ сдвиг не выполняется, 1 - при записи в ОЗУ сдвиг дисплея выполняется согласно установке LCD_DEC_INC_DDRAM (0 - сдвиг вправо, 1 - сдвиг влево)
#define LCD_DISPLAY_OFF_ON          1	//включение / выключение дисплея. 0 - дисплей выключен, но данные в ОЗУ остаются; 1 - дисплей включен
#define LCD_CURSOR_OFF_ON           0	//тображение подчеркивающего курсора. 0 - курсор не отображается, 1 - курсор отображается
#define LCD_CURSOR_BLINK_OFF_ON     0	//отображение мигающего курсора. 0 - мигающий курсор не отображается; 1 - мигающий курсор отображается
#define LCD_CURSOR_DISPLAY_SHIFT    0	//команда сдвига вправо/влево курсора или дисплея без записи на дисплей. В библиотеке не используется и ни на что не влияет.

/*_____________________макро функции________________________________*/

//команды
#define LCD_CLEAR_DISPLAY  0x01
#define LCD_RETURN_HOME    0x02

//очистка дисплея
#define LCD_Clear() do{LCD_WriteCom(LCD_CLEAR_DISPLAY); _delay_ms(2);}while(0)   

//возврат курсора в начальное положение
#define LCD_ReturnHome()  do{LCD_WriteCom(LCD_RETURN_HOME);}while(0)

//позиционирование курсора
#define LCD_Goto(x, y)    LCD_WriteCom(((((y)& 1)*0x40)+((x)& 15))|128)  

/*___________________пользовательские функции_______________________*/

void LCD_Init(void);                                  //инициализация портов и жкд по умолчанию
void LCD_WriteCom(uint8_t data);                      //посылает команду жкд
void LCD_WriteData(char data);                        //выводит символ на жкд
void LCD_SendStr(char *str);                          //выводит строку из ОЗУ

#ifdef __GNUC__
   void LCD_SendStrFl(char const *str);                     //выводит строку из флэш памяти     
   void LCD_SetUserChar(uint8_t const *sym, uint8_t adr);   //загрузить пользовательский символ
#else
   void LCD_SendStrFl(char __flash *str);                   //выводит строку из флэш памяти
   void LCD_SetUserChar(uint8_t __flash *sym, uint8_t adr); //загрузить пользовательский символ
#endif

#endif