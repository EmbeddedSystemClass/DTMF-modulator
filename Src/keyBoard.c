/*
 * keyBoard.c
 *
 *  Created on: 17 sty 2019
 *      Author: Witek
 */
#include "keyBoard.h"
#include "main.h"




void SetRows(GPIO_TypeDef *_GPIO_R0,  uint16_t _Pin_R0, GPIO_TypeDef *_GPIO_R1,  uint16_t _Pin_R1, GPIO_TypeDef *_GPIO_R2,  uint16_t _Pin_R2, GPIO_TypeDef *_GPIO_R3,  uint16_t _Pin_R3)
{
	GPIO_ROWS[0] = _GPIO_R0;
	GPIO_ROWS[1] = _GPIO_R1;
	GPIO_ROWS[2] = _GPIO_R2;
	GPIO_ROWS[3] = _GPIO_R3;

	Pin_ROWS[0] = _Pin_R0;
	Pin_ROWS[1] = _Pin_R1;
	Pin_ROWS[2] = _Pin_R2;
	Pin_ROWS[3] = _Pin_R3;

	procesSPrawdzaniaPrzyciskow = 0;
}

void RowDown(uint8_t nrRow)
{
	HAL_GPIO_WritePin(GPIO_ROWS[nrRow], Pin_ROWS[nrRow], GPIO_PIN_RESET);
}

void RowUp(uint8_t nrRow)
{
	HAL_GPIO_WritePin(GPIO_ROWS[nrRow], Pin_ROWS[nrRow], GPIO_PIN_SET);
}

void SetColumns(GPIO_TypeDef *_GPIO_C0,  uint16_t _Pin_C0, GPIO_TypeDef *_GPIO_C1,  uint16_t _Pin_C1, GPIO_TypeDef *_GPIO_C2,  uint16_t _Pin_C2, GPIO_TypeDef *_GPIO_C3,  uint16_t _Pin_C3)
{
	GPIO_COLUMNS[0] = _GPIO_C0;
	GPIO_COLUMNS[1] = _GPIO_C1;
	GPIO_COLUMNS[2] = _GPIO_C2;
	GPIO_COLUMNS[3] = _GPIO_C3;

	Pin_COLUMNS[0] = _Pin_C0;
	Pin_COLUMNS[1] = _Pin_C1;
	Pin_COLUMNS[2] = _Pin_C2;
	Pin_COLUMNS[3] = _Pin_C3;
}

bool ReadColumn(uint8_t nrCol)
{
	return HAL_GPIO_ReadPin(GPIO_COLUMNS[nrCol], Pin_COLUMNS[nrCol]) == 0;
}


void eachInterrupt()
{
	sprawdzPrzyciski();

}

void sprawdzPrzyciski()
{
	if (procesSPrawdzaniaPrzyciskow == 0)
	{
		RowDown(0);
		RowUp(1);
		RowUp(2);
		RowUp(3);
		procesSPrawdzaniaPrzyciskow = 1;
	}
	else if (procesSPrawdzaniaPrzyciskow == 1)
	{
		if (ReadColumn(0)) UstawPrzycisk(1);
		if (ReadColumn(1)) UstawPrzycisk(2);
		if (ReadColumn(2)) UstawPrzycisk(3);
		if (ReadColumn(3)) UstawPrzycisk(4);
		procesSPrawdzaniaPrzyciskow = 2;
	}
	else if (procesSPrawdzaniaPrzyciskow == 2)
	{
		RowUp(0);
		RowDown(1);
		RowUp(2);
		RowUp(3);
		procesSPrawdzaniaPrzyciskow = 3;
	}
	else if (procesSPrawdzaniaPrzyciskow == 3)
	{
		if (ReadColumn(0)) UstawPrzycisk(5);
		if (ReadColumn(1)) UstawPrzycisk(6);
		if (ReadColumn(2)) UstawPrzycisk(7);
		if (ReadColumn(3)) UstawPrzycisk(8);
		procesSPrawdzaniaPrzyciskow = 4;
	}
	else if (procesSPrawdzaniaPrzyciskow == 4)
	{
		RowUp(0);
		RowUp(1);
		RowDown(2);
		RowUp(3);
		procesSPrawdzaniaPrzyciskow = 5;
	}
	else if (procesSPrawdzaniaPrzyciskow == 5)
	{
		if (ReadColumn(0)) UstawPrzycisk(9);
		if (ReadColumn(1)) UstawPrzycisk(10);
		if (ReadColumn(2)) UstawPrzycisk(11);
		if (ReadColumn(3)) UstawPrzycisk(12);
		procesSPrawdzaniaPrzyciskow = 6;
	}
	else if (procesSPrawdzaniaPrzyciskow == 6)
	{
		RowUp(0);
		RowUp(1);
		RowUp(2);
		RowDown(3);
		procesSPrawdzaniaPrzyciskow = 7;
	}
	else if (procesSPrawdzaniaPrzyciskow == 7)
	{
		if (ReadColumn(0)) UstawPrzycisk(13);
		if (ReadColumn(1)) UstawPrzycisk(14);
		if (ReadColumn(2)) UstawPrzycisk(15);
		if (ReadColumn(3)) UstawPrzycisk(16);
		procesSPrawdzaniaPrzyciskow = 0;
	}
	UstawPrzycisk(0);
}


bool pushed = false;
void UstawPrzycisk(uint8_t przycisk)
{
	//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	//static bool pushed = false;

	if (przycisk != 0 /*&& */)
	{
		czasyPrzyciskow[przycisk] = 40;
		if (pushed == false)
		{
			WcisnietoPrzycisk(przycisk);
			pushed = true;
		}
		//
		//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);


	}

	for(uint8_t i = 1; i<=16;i++)
	{
		if (czasyPrzyciskow[i] > 0) czasyPrzyciskow[i] --;
		if (czasyPrzyciskow[i] == 1)
		{
			pushed = false;
			PuszczonoPrzycisk(i);
		}
	}
}


void SetCallbacks(void(*__WcisnietoPrzyciskCallback)(uint8_t), void(*__PuszczonoPrzyciskCallback)(uint8_t))
{
	_WcisnietoPrzyciskCallback = __WcisnietoPrzyciskCallback;
	_PuszczonoPrzyciskCallback = __PuszczonoPrzyciskCallback;
}
void WcisnietoPrzycisk(uint8_t nrPrzycisku)
{
	_WcisnietoPrzyciskCallback(nrPrzycisku);
}

void PuszczonoPrzycisk(uint8_t nrPrzycisku)
{
	_PuszczonoPrzyciskCallback(nrPrzycisku);
}


