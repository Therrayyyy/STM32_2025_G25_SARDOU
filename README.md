# TP-Réalisation des systèmes BE

## Introduction

Ce projet vise à créer une ruche connectée pour surveiller en temps réel la température, l'humidité et le poids. Les capteurs SHT31, DHT22 et HX711 collectent les données, qui sont ensuite transmises via un module LoRa-E5. Un microcontrôleur STM32 gère l'ensemble du système et affiche les informations sur un écran LCD.

## Câblage du Projet

- **Capteur SHT31** : Connecté via I2C.
- **Capteur DHT22** : Connecté via un fil de données.
- **Capteur HX711** : Connecté via une interface série.
- **Module LoRa-E5** : Connecté via UART.
- **Afficheur LCD** : Connecté via I2C.

## Instructions d'Utilisation

1. **Câblage** : Suivez les instructions de câblage pour chaque composant.
2. **Alimentation** : Utilisez une source de 3.3V.
3. **Démarrage** : Le système démarre automatiquement et affiche les données sur l'écran LCD tout en les transmettant via LoRa.

## Contributeurs

- **Rayan Sardou**
- **Amayas Kaci**

## Conclusion

Ce projet démontre la capacité de surveiller et de transmettre des données environnementales en temps réel, ouvrant la voie à des applications pratiques dans divers domaines.
