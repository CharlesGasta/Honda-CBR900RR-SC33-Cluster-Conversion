# Honda CBR900RR SC33 — 1996–1997 Cluster Conversion on 1998 Harness

<p align="center">
  <img src="AFFICHE%20GITHUB.png" alt="Honda CBR900RR SC33 cluster conversion" width="100%">
</p>

Open-source documentation and ESP8266 firmware for installing a **1996–1997 Honda CBR900RR SC33 instrument cluster on a 1998 motorcycle / wiring harness** while retaining the 1998 temperature sender.

> **Current firmware:** V20.4 — Smooth RPM  
> **3D enclosure:** [Thingiverse — thing:7411537](https://www.thingiverse.com/thing:7411537)

## Quick download

| File | Download |
|---|---|
| Latest ESP8266 firmware — V20.4 | [Download .ino](https://raw.githubusercontent.com/CharlesGasta/Honda-CBR900RR-SC33-Cluster-Conversion/main/firmware/CBR900RR_V20_4_SMOOTH_RPM.ino) |
| Complete workshop wiring PDF | [Open / download PDF](Tableau_conversion_complet_CBR900RR_SC33_COMPTEUR.pdf) |
| 3D enclosure | [Download from Thingiverse](https://www.thingiverse.com/thing:7411537) |

<p align="center">
  <img src="INTERFACE%20CBR900RR%20WIFI.png" alt="CBR900RR Wi-Fi dashboard" width="420">
</p>

---

# Français

## Présentation

Ce projet documente l'adaptation d'un **compteur de CBR900RR SC33 1996–1997 sur une CBR900RR SC33 1998**.

Le faisceau principal de la moto n'a pas besoin d'être modifié : l'adaptation est réalisée par un faisceau intermédiaire. Le projet traite également les deux différences qui ne peuvent pas être résolues par un simple changement de broches :

- adaptation du **signal de vitesse** avec un **SpeedoHealer V4** ;
- conservation de la **sonde de température 1998** avec un **ESP8266 NodeMCU** qui pilote la jauge 1996–1997.

Le firmware ajoute en complément un **shift-light**, un **tableau de bord Wi-Fi**, un thermomètre numérique, le diagnostic en temps réel et des fonctions de calibration.

## Compatibilité visée

- Moto / faisceau : **Honda CBR900RR Fireblade SC33 1998**
- Compteur : **Honda CBR900RR Fireblade SC33 1996–1997**
- Contrôleur : **ESP8266 NodeMCU**
- Firmware actuel : **V20.4**

Cette documentation correspond au montage développé et testé dans le cadre de ce projet. Vérifiez toujours votre propre faisceau, les couleurs de fils et les références de pièces avant branchement.

## Ce que fait le NodeMCU

Le firmware V20.4 assure :

- lecture de la sonde de température d'origine 1998 ;
- conversion résistance → température ;
- pilotage de la jauge analogique 1996–1997 ;
- lecture du signal compte-tours via optocoupleur ;
- calibration du compte-tours au ralenti ;
- shift-light 12 V via MOSFET ;
- seuil de shift spécifique **moteur froid** ;
- seuils fixe et clignotant **moteur chaud** ;
- seuil de température froid → chaud réglable ;
- point d'accès Wi-Fi autonome ;
- interface téléphone avec compte-tours analogique ;
- affichage RPM filtré et animation d'aiguille fluide ;
- calibration de température à froid et au déclenchement du ventilateur ;
- diagnostic complet et journal d'événements ;
- sauvegarde des réglages en EEPROM ;
- restauration des paramètres par défaut ;
- redémarrage logiciel depuis l'interface.

### Wi-Fi

```text
SSID : CBR900RR
Mot de passe : 00365412
Adresse : 192.168.4.1
```

Le téléphone se connecte directement au réseau créé par le NodeMCU. Aucune box, connexion Internet ou application dédiée n'est nécessaire.

## Shift-light froid / chaud

Valeurs par défaut :

```text
Moteur < 60 °C
→ limite à froid : 8000 tr/min
→ shift-light clignotant

Moteur ≥ 60 °C
→ fixe : 9000 tr/min
→ clignotant : 10000 tr/min
```

Les trois seuils sont réglables depuis l'interface Wi-Fi.

## Compte-tours V20.4

Le signal est lu sur le NodeMCU via le PC817. La V20.4 utilise **2 impulsions par tour** pour le calcul de base.

Deux valeurs sont volontairement séparées :

- **RPM contrôle** : valeur réactive utilisée par le shift-light ;
- **RPM affichage** : valeur filtrée utilisée par le Dashboard.

L'aiguille du Dashboard ne se téléporte plus entre les mesures : le navigateur interpole continuellement sa position pour produire un mouvement plus naturel, sans ralentir la logique du shift-light.

Le menu de calibration permet d'indiquer au système que le moteur est stabilisé à **1100 tr/min** afin de calculer et sauvegarder le facteur de correction.

## Température moteur

Le montage conserve la **sonde de température du modèle 1998**. Le NodeMCU mesure sa résistance, estime la température et recrée la commande nécessaire à la jauge du compteur 1996–1997.

Repères de jauge par défaut issus des essais du projet :

| Position | PWM |
|---|---:|
| 0 % | 400 |
| 25 % | 650 |
| 50 % | 725 |
| 75 % | 800 |
| 100 % | 880 |
| 110 % / surchauffe | 900 |

Le firmware permet également de calibrer :

- le point froid à partir de la température ambiante réelle ;
- le point chaud au moment exact du déclenchement du ventilateur ;
- la marge d'erreur affichée du thermomètre.

## Matériel principal

- compteur CBR900RR SC33 1996–1997 ;
- connecteurs adaptés au faisceau 1998 et au compteur 1996–1997 ;
- ESP8266 NodeMCU ;
- convertisseur DC 12 V → 5 V pour l'alimentation du NodeMCU ;
- PC817 pour l'isolation du signal compte-tours ;
- MOSFET logique pour la jauge de température ;
- MOSFET logique pour le shift-light 12 V ;
- résistances et condensateurs de filtrage indiqués dans la documentation de câblage ;
- fusible dédié sur l'alimentation du module ;
- SpeedoHealer V4 / faisceau universel pour la correction du signal de vitesse ;
- câblage, connecteurs et gaine thermorétractable.

Pour les valeurs, broches et raccordements, utilisez le **PDF de câblage** plutôt qu'une photo ou un résumé de forum.

## Installation du firmware

1. Installer **Arduino IDE**.
2. Installer le package **esp8266 by ESP8266 Community**.
3. Sélectionner :
   ```text
   NodeMCU 1.0 (ESP-12E Module)
   ```
4. Télécharger et ouvrir :
   ```text
   firmware/CBR900RR_V20_4_SMOOTH_RPM.ino
   ```
5. Compiler puis téléverser sur le NodeMCU.
6. Mettre le contact.
7. Se connecter au Wi-Fi `CBR900RR`.
8. Ouvrir `192.168.4.1` dans le navigateur.
9. Effectuer les calibrations sur la moto si nécessaire.

## Boîtier 3D

Le boîtier du NodeMCU est publié séparément sur Thingiverse avec les fichiers et informations d'impression :

**[Télécharger le boîtier 3D sur Thingiverse](https://www.thingiverse.com/thing:7411537)**

Le dépôt GitHub reste ainsi centré sur le câblage, la documentation et le firmware ; Thingiverse reste la source du modèle 3D.

## Structure du dépôt

```text
/
├── README.md
├── LICENSE
├── CHANGELOG.md
├── AFFICHE GITHUB.png
├── INTERFACE CBR900RR WIFI.png
├── Tableau_conversion_complet_CBR900RR_SC33_COMPTEUR.pdf
└── firmware/
    └── CBR900RR_V20_4_SMOOTH_RPM.ino
```

Seule la **dernière version du firmware** est conservée dans la branche principale afin d'éviter de flasher par erreur une ancienne version. L'historique Git conserve les versions précédentes.

## Questions / contributions

Si vous avez une question, trouvé une erreur ou amélioré le montage, vous pouvez :

- ouvrir une **Issue** sur ce dépôt ;
- proposer une **Pull Request** ;
- me contacter via mon profil GitHub : [@CharlesGasta](https://github.com/CharlesGasta).

---

# English

## Overview

This repository documents the installation of a **1996–1997 CBR900RR SC33 instrument cluster on a 1998 CBR900RR SC33 motorcycle / wiring harness**.

The motorcycle's main harness does not need to be modified: the conversion is performed with an intermediate adapter harness. The project also addresses the two functions that cannot be solved by pin reassignment alone:

- **speed signal correction** using a **SpeedoHealer V4**;
- retaining the **1998 temperature sender** using an **ESP8266 NodeMCU** to drive the 1996–1997 gauge.

The ESP8266 firmware also provides a shift light, local Wi-Fi dashboard, digital coolant-temperature display, live diagnostics and calibration tools.

## Main V20.4 features

- original 1998 temperature-sender reading;
- resistance-to-temperature conversion;
- 1996–1997 analog temperature-gauge control;
- opto-isolated tachometer input;
- 1100 RPM idle calibration;
- 12 V shift-light output through a MOSFET;
- configurable **cold-engine RPM limit**;
- configurable normal and flashing **hot-engine shift thresholds**;
- configurable cold → hot temperature threshold;
- standalone Wi-Fi access point;
- mobile analog tachometer dashboard;
- filtered display RPM with smooth needle animation;
- cold and fan-trigger temperature calibration;
- live diagnostic page and event log;
- EEPROM settings storage;
- factory-default restore;
- remote software restart.

### Default Wi-Fi

```text
SSID: CBR900RR
Password: 00365412
Address: 192.168.4.1
```

No Internet connection or dedicated mobile application is required.

## Default cold / hot shift logic

```text
Engine < 60 °C
→ cold limit: 8000 RPM
→ flashing shift light

Engine ≥ 60 °C
→ solid: 9000 RPM
→ flashing: 10000 RPM
```

All thresholds are configurable from the Wi-Fi interface.

## RPM display

V20.4 uses **2 pulses per revolution** for the base tachometer calculation.

The firmware intentionally separates:

- **control RPM** for responsive shift-light operation;
- **display RPM** with additional filtering for the phone Dashboard.

The browser continuously interpolates the analog needle between new RPM targets, producing smoother motion without delaying the shift-light logic.

## 3D enclosure

The enclosure files and print information are hosted on Thingiverse:

**[Download the 3D enclosure on Thingiverse](https://www.thingiverse.com/thing:7411537)**

## Installation

1. Install Arduino IDE.
2. Install **esp8266 by ESP8266 Community**.
3. Select **NodeMCU 1.0 (ESP-12E Module)**.
4. Download `firmware/CBR900RR_V20_4_SMOOTH_RPM.ino`.
5. Compile and upload.
6. Switch the motorcycle ignition on.
7. Connect to Wi-Fi `CBR900RR`.
8. Open `192.168.4.1`.
9. Perform the required on-bike calibrations.

## Support and contributions

Questions, corrections and improvements are welcome through GitHub **Issues** and **Pull Requests**, or via [@CharlesGasta](https://github.com/CharlesGasta).

---

## License

The source code in this repository is released under the **MIT License**. See [LICENSE](LICENSE).

## Disclaimer

This is an independent open-source project and is **not affiliated with, sponsored by, approved by or endorsed by Honda Motor Co., Ltd., HealTech Electronics, Thingiverse or their respective owners**. Product and company names are used only to identify compatibility.

Motorcycle electrical systems can be damaged by incorrect wiring. A wiring error may also cause loss of instrumentation or other unsafe behavior. Verify connector orientation, pin numbering, voltage, polarity and continuity on your own motorcycle before applying power.

The software, documentation and wiring information are provided **as-is, without warranty**. You remain responsible for installation, validation and safe operation of the motorcycle.
