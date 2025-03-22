# Cube IDE Project

## Description

Ce projet utilise STM32Cube pour gérer plusieurs capteurs et modules de communication sur une carte STM32.

## Fonctionnalités

- **Capteurs** : SHT31, DHT22, HX711.
- **Communication** : LoRa, UART, I2C.
- **Affichage** : Écran LCD pour les données des capteurs.

## Configuration

- Initialisation des capteurs et modules.
- Collecte et affichage des données de température, humidité et poids.
- Transmission des données via LoRa.

## Utilisation

- Le programme initialise les capteurs et affiche les données sur un écran LCD.
- Les données sont transmises périodiquement via LoRa.