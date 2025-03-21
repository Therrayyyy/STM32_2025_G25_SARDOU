/*
 * dht22.c
 *
 *  Created on: Jan 29, 2025
 *      Author:
 */

#include "dht22.h"
#include "tim.h" // Assurez-vous que le timer est configuré pour gérer les délais

// Fonction pour configurer une broche en sortie
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Mode sortie push-pull
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Vitesse basse pour minimiser le bruit
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

// Fonction pour configurer une broche en entrée
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // Mode entrée
    GPIO_InitStruct.Pull = GPIO_NOPULL; // Pas de pull-up ou pull-down
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

// Fonction d'initialisation du DHT22
void DHT22_Start(void)
{
    Set_Pin_Output(DHT22_PORT, DHT22_PIN); // Configurer la broche en sortie
    HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_RESET); // Mettre la ligne à LOW
    HAL_Delay(1);  // Maintenir LOW pendant >1ms (réinitialisation du capteur)

    HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_SET); // Relâcher la ligne
    delay(30);  // Attendre 30µs avant d'écouter la réponse du capteur

    Set_Pin_Input(DHT22_PORT, DHT22_PIN); // Passer en entrée pour lire la réponse du DHT22
}

// Vérifier la réponse du DHT22
uint8_t DHT22_Check_Response(void)
{
    uint8_t Response = 0;
    delay(40); // Attendre 40µs

    if (!(HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))) // Vérifier si la ligne est LOW
    {
        delay(80); // Attendre encore 80µs
        if ((HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))) // Vérifier si la ligne est HIGH
            Response = 1; // Réponse correcte du DHT22
        else
            Response = 0; // Pas de réponse
    }

    while ((HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))); // Attendre que la ligne repasse à LOW
    return Response; // Retourner l'état de la réponse
}

// Lire un octet de données du DHT22
uint8_t DHT22_Read(void)
{
    uint8_t i = 0, j;

    for (j = 0; j < 8; j++) // Lire 8 bits
    {
        while (!(HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))); // Attendre le passage à HIGH
        delay(40); // Attendre 40 µs

        if (!(HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))) // Si la ligne est LOW après 40µs
            i &= ~(1 << (7 - j));  // Écrire 0 dans le bit correspondant
        else
            i |= (1 << (7 - j));   // Écrire 1 dans le bit correspondant

        while ((HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))); // Attendre le retour à LOW
    }
    return i; // Retourner l’octet lu
}

// Fonction pour récupérer la température et l'humidité du DHT22
void DHT22_Get_Data(float *temperature, float *humidity)
{
    uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, SUM;
    uint16_t RH, TEMP;

    DHT22_Start(); // Démarrer le protocole de communication avec le capteur
    if (DHT22_Check_Response()) // Vérifier si le capteur répond
    {
        Rh_byte1 = DHT22_Read();  // Lire la première partie de l'humidité
        Rh_byte2 = DHT22_Read();  // Lire la seconde partie de l'humidité
        Temp_byte1 = DHT22_Read(); // Lire la première partie de la température
        Temp_byte2 = DHT22_Read(); // Lire la seconde partie de la température
        SUM = DHT22_Read(); // Lire le checksum (somme de contrôle)

        // Convertir les valeurs brutes en valeurs exploitables
        RH = ((Rh_byte1 << 8) | Rh_byte2); // Combiner les deux octets pour l'humidité
        TEMP = ((Temp_byte1 << 8) | Temp_byte2); // Combiner les deux octets pour la température

        *humidity = (float)(RH / 10.0); // Convertir l'humidité en pourcentage
        *temperature = (float)(TEMP / 10.0); // Convertir la température en degrés Celsius
    }
}

// Fonction de délai en microsecondes (utilise TIM6)
void delay (uint16_t time)
{
    __HAL_TIM_SET_COUNTER(&htim6, 0); // Réinitialiser le compteur du timer
    while ((__HAL_TIM_GET_COUNTER(&htim6)) < time); // Attendre que le timer atteigne la valeur demandée
}



