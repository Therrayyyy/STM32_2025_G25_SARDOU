/*
 * hx711.h
 *
 *  Created on: Mar 4, 2025
 *      Author: rehali
 */

#ifndef INC_HX711_H_
#define INC_HX711_H_

#include "stm32l4xx_hal.h"

#define DT_PIN GPIO_PIN_5
#define DT_PORT GPIOB
#define SCK_PIN GPIO_PIN_6
#define SCK_PORT GPIOB

void HX711_Start(void);
int32_t getHX711(void);
int32_t getAverageReading(uint16_t samples);
void calibrate(float *kHX711, uint32_t *taree);
int weigh(float kOriginal, float kHX711, uint32_t taree);


#endif /* INC_HX711_H_ */
