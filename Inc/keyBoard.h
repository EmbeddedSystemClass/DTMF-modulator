/*
 * keyBoard.h
 *
 *  Created on: 16 sty 2019
 *      Author: Witek
 */

#ifndef KEYBOARD_H_
#define KEYBOARD_H_

#include "main.h"
#include <stdbool.h>


void SetRows(GPIO_TypeDef *_GPIO_R0,  uint16_t _Pin_R0, GPIO_TypeDef *_GPIO_R1,  uint16_t _Pin_R1, GPIO_TypeDef *_GPIO_R2,  uint16_t _Pin_R2, GPIO_TypeDef *_GPIO_R3,  uint16_t _Pin_R3);
void RowDown(uint8_t nrRow);
void RowUp(uint8_t nrRow);
void SetColumns(GPIO_TypeDef *_GPIO_C0,  uint16_t _Pin_C0, GPIO_TypeDef *_GPIO_C1,  uint16_t _Pin_C1, GPIO_TypeDef *_GPIO_C2,  uint16_t _Pin_C2, GPIO_TypeDef *_GPIO_C3,  uint16_t _Pin_C3);
bool ReadColumn(uint8_t nrCol);
void eachInterrupt();
void sprawdzPrzyciski();
void UstawPrzycisk(uint8_t przycisk);
void SetCallbacks(void(*__WcisnietoPrzyciskCallback)(uint8_t), void(*__PuszczonoPrzyciskCallback)(uint8_t));
void WcisnietoPrzycisk(uint8_t nrPrzycisku);
void PuszczonoPrzycisk(uint8_t nrPrzycisku);


GPIO_TypeDef *GPIO_ROWS[4];
uint16_t Pin_ROWS[4];

GPIO_TypeDef *GPIO_COLUMNS[4];
uint16_t Pin_COLUMNS[4];

uint8_t procesSPrawdzaniaPrzyciskow;
uint8_t czasyPrzyciskow[17];
void (*_WcisnietoPrzyciskCallback)(uint8_t);
void (*_PuszczonoPrzyciskCallback)(uint8_t);



#endif /* KEYBOARD_H_ */
