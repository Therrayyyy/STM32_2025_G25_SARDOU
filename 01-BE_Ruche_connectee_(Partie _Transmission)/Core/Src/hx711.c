/*
 * hx711.c
 *
 *  Created on: Mar 4, 2025
 *      Author: rehali
 */

#include "hx711.h"
#include "tim.h"
#include "dht22.h"
#include "i2c.h"
#include "lcd.h"
#include "main.h"
#include <stdio.h>

// Fonction d'initialisation du HX711
void HX711_Start(void)
{
  HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET); // Met SCK à 1 pour réveiller le capteur
  HAL_Delay(10); // Attente de 10 ms
  HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET); // Remet SCK à 0 pour initialiser la communication
  HAL_Delay(10); // Attente de 10 ms
}

// Fonction pour lire une valeur brute de 24 bits depuis le HX711
int32_t getHX711(void)
{
  uint32_t data = 0; // Stocke la donnée lue
  uint32_t startTime = HAL_GetTick(); // Stocke le temps de départ pour éviter un blocage

  // Attente de la disponibilité des données (DOUT passe à 0)
  while(HAL_GPIO_ReadPin(DT_PORT, DT_PIN) == GPIO_PIN_SET)
  {
    if(HAL_GetTick() - startTime > 200) // Si l'attente dépasse 200 ms, annule la lecture
      return 0;
  }

  // Lecture des 24 bits de données
  for(int8_t len=0; len<24 ; len++)
  {
    HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET); // Génère une impulsion d'horloge SCK (montée)
    delay(1); // Petite pause pour garantir la stabilité
    data = data << 1; // Décale la donnée vers la gauche pour insérer un nouveau bit
    HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET); // Génère une impulsion d'horloge SCK (descente)
    delay(1); // Petite pause
    if(HAL_GPIO_ReadPin(DT_PORT, DT_PIN) == GPIO_PIN_SET) // Lit la valeur de DOUT
      data ++; // Ajoute 1 si le bit lu est haut
  }

  data = data ^ 0x800000; // Applique un XOR avec 0x800000 pour convertir en valeur signée 24 bits

  // Impulsion supplémentaire pour configurer le gain à 128 et canal A
  HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
  delay(1);
  HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
  delay(1);

  return data; // Retourne la donnée lue
}

int32_t getAverageReading(uint16_t samples)
{
  int32_t total = 0;
  for(uint16_t i=0 ; i<samples ; i++)
  {
      total += getHX711();
  }
  return (int32_t)(total / samples);
}

void calibrate(float *kHX711, uint32_t *taree)
{
  // Étape 1 : Mesure de la tare
  *taree = getAverageReading(50);
  HAL_Delay(2000);

  clearlcd();
  lcd_position(&hi2c1, 0, 0);
  lcd_print(&hi2c1, "Veuillez deposer");
  lcd_position(&hi2c1, 2, 1);
  lcd_print(&hi2c1, "votre objet");
  printf("Veuillez deposer un objet pour commencer la procedure de calibrage\r\n");
  HAL_Delay(5000);

  clearlcd();
  lcd_position(&hi2c1, 4, 0);
  lcd_print(&hi2c1, "Calibrage");
  lcd_position(&hi2c1, 3, 1);
  lcd_print(&hi2c1, "En cours...");
  printf("Le calibrage est en cours...\r\n");
  HAL_Delay(3000);

  // Étape 2 : Mesure avec l'objet connu
  *kHX711 = getAverageReading(50) - *taree;
}

// Fonction pour mesurer le poids d'un objet
int weigh(float kOriginal, float kHX711, uint32_t taree)
{
  int32_t  measuredValue = getAverageReading(50); // Mesure la valeur actuelle du capteur
  float coefficient = kOriginal / kHX711; // Calcule le coefficient de conversion
  int milligram = (int)(measuredValue - taree) * coefficient; // Convertit la valeur brute en mg
  return milligram; // Retourne le poids en mg
}



