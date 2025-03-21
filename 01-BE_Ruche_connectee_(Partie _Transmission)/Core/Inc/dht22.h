/*
 * dht22.h
 *
 *  Created on: Jan 29, 2025
 *      Author:
 */

#ifndef INC_DHT22_H_
#define INC_DHT22_H_

#include "stm32l4xx_hal.h"

// Définition du port et de la broche utilisés pour le DHT22
#define DHT22_PORT GPIOA
#define DHT22_PIN GPIO_PIN_1

// Prototypes des fonctions
void DHT22_Init(void);
void DHT22_Start(void);
uint8_t DHT22_Check_Response(void);
uint8_t DHT22_Read(void);
void DHT22_Get_Data(float *temperature, float *humidity);
void delay (uint16_t time);

// Fonctions utilitaires
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);



#endif /* INC_DHT22_H_ */
