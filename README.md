Ce projet présente un robot mobile autonome piloté par reconnaissance vocale. Conçu autour d'une architecture Raspberry Pi, il intègre des capacités de traitement du langage naturel (NLP) simple pour exécuter des commandes complexes et un système de sécurité par ultrasons pour l'évitement d'obstacles.

 Fonctionnalités Clés
Contrôle Vocal Bilingue : Reconnaissance des commandes en Français et Anglais via l'API Google Speech Recognition.

Interprétation de Commandes Complexes : Capacité à enchaîner des actions (ex: "Avance pendant 3 secondes puis tourne à droite").

Sécurité Active : Surveillance en temps réel via un capteur ultrason (HC-SR04) avec arrêt d'urgence automatique si un obstacle est détecté à moins de 20cm.

Gestion Multithread : Le capteur de distance tourne sur un thread séparé pour garantir une réactivité maximale sans bloquer l'écoute vocale.

Pilotage PWM : Contrôle précis de la vitesse des moteurs DC.





Architecture Matérielle
Composants utilisés :
Microcontrôleur : Raspberry Pi (3/4/5)

Contrôleur Moteur : L298N Dual H-Bridge

Moteurs : 2x Moteurs DC (6V-12V)

Capteur de distance : HC-SR04 (Ultrason)

Audio : Microphone USB ou carte son externe

Alimentation : Batterie externe (Powerbank) pour la Pi + Support piles pour les moteurs





Installation & Configuration
1. Prérequis
Assurez-vous d'avoir Python 3 installé sur votre Raspberry Pi et une connexion internet (pour la reconnaissance vocale Google).

2. Installation des dépendances
Bash

sudo apt-get update
sudo apt-get install portaudio19-dev python3-pyaudio
pip3 install SpeechRecognition RPi.GPIO
3. Lancement
Bash

python3 main.py





Guide des Commandes VocalesLe robot supporte des instructions simples et composées grâce à des mots-clés de liaison (puis, ensuite, and, then).ActionExemples de commandesDéplacement"Avance", "Recule", "Gauche", "Droite"Durée"Avance 5 secondes"Vitesse"Plus vite", "Ralentir"Séquences"Avancer puis tourner à gauche"Arrêt"Stop", "Arrête", "Éteindre" (Quitte le programme)





Structure du Code
Le projet est organisé de manière modulaire :

class DistanceSensor : Gère les impulsions sonores et le calcul de distance en cm.

class Robot : Gère les états des moteurs, le PWM et les directions.

monitor_obstacles() : Fonction exécutée en arrière-plan (Threading) pour la sécurité.

process_command() : Parseur de texte utilisant des expressions régulières (Regex) pour extraire les durées et les actions.


À propos
Ce projet a été réalisé dans un cadre académique pour démontrer l'intégration de l'IoT, de la robotique et de l'intelligence artificielle (traitement de la voix).

Auteur : Sarah , Myriam , Rosa et Bilal
