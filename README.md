# Additive Random Sampling for Aeronautical Vibration Monitoring

<div align="center">

![STM32](https://img.shields.io/badge/MCU-STM32H723ZG-03234B?style=for-the-badge&logo=stmicroelectronics&logoColor=white)
![Status](https://img.shields.io/badge/Status-Prototype-success?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge)
![School](https://img.shields.io/badge/School-EPT-blue?style=for-the-badge)
![Partner](https://img.shields.io/badge/Partner-Safran-red?style=for-the-badge)

**Prototype embarqué de surveillance vibratoire par échantillonnage aléatoire (ARS)**

*École Polytechnique de Tunisie (EPT) — Collaboration industrielle Safran*  
*Année universitaire 2025 – 2026*

[Rapport complet (PDF)](docs/rapport_projet.pdf) · [Code source](firmware/) · [Script PC](host/)

</div>

---

## Présentation du projet

Ce dépôt contient le **prototype complet** d’un système embarqué de **surveillance vibratoire** pour composants mécaniques aéronautiques (roulements, engrenages, transmissions).

Le système met en œuvre l’**échantillonnage aléatoire additif (Additive Random Sampling – ARS)**.  
Cette technique permet d’acquérir des signaux à une fréquence moyenne **bien inférieure** au critère de Nyquist-Shannon tout en évitant les alias déterministes (le repliement spectral est redistribué sous forme de bruit de fond).

| Caractéristique              | Valeur                          |
|-----------------------------|----------------------------------|
| Microcontrôleur             | **STM32H723ZG** (Nucleo)        |
| Fréquence moyenne ARS       | **500 Hz**                      |
| Paramètres                  | \(a=1200\,\mu s\), \(b=2800\,\mu s\), \(R=0.8\) |
| Résolution temporelle       | **1 µs**                        |
| Nombre d’échantillons/bloc  | **4000**                        |
| Interface PC                | UART 115200 bauds + Python      |

---

## Pourquoi l’échantillonnage aléatoire ?

Dans l’aéronautique, les systèmes **HUMS** (Health and Usage Monitoring Systems) doivent surveiller des signaux vibratoires jusqu’à plusieurs dizaines de kHz.  
L’échantillonnage classique (Nyquist) génère alors des volumes de données incompatibles avec les nœuds IoT embarqués (énergie, mémoire, bande passante radio).

L’**ARS** offre une alternative sub-Nyquist élégante :
- Pas de filtre anti-repliement analogique complexe obligatoire
- Spectre estimable directement (FFT + Welch ou Lomb-Scargle)
- Compatible avec des microcontrôleurs bas coût / basse consommation

---

## Architecture du système

```text
┌──────────────────┐     ┌────────────────────────────┐     ┌─────────────────────┐
│  Accéléromètre   │────▶│      STM32H723ZG           │────▶│   PC / Interface    │
│  MEMS / IEPE     │     │  • Génération ARS (RNG)    │     │  • Réception UART   │
│                  │     │  • ADC 12 bits horodaté    │     │  • DSP (Welch / LS) │
└──────────────────┘     │  • Transmission blocs      │     │  • Visualisation     │
                         └────────────────────────────┘     └─────────────────────┘
```

### Périphériques utilisés

| Périphérique          | Rôle                                              |
|-----------------------|---------------------------------------------------|
| **DAC1 + TIM6 + DMA** | Génération signal de test carré 1 kHz (PA4)      |
| **TIM2**              | Déclenchement aléatoire des conversions (1 µs)   |
| **ADC1**              | Acquisition 12 bits (PF11)                        |
| **RNG matériel**      | Tirage uniforme des intervalles \(\tau_n\)        |
| **USART3**            | Envoi des données vers le PC (ST-Link)            |

---

## Structure du dépôt

```text
.
├── README.md                          ← Vous êtes ici
├── LICENSE                            ← Licence MIT
├── .gitignore
│
├── firmware/                          ← Projet STM32CubeIDE
│   ├── Core/
│   │   ├── Inc/
│   │   └── Src/
│   │       └── main.c                 ← Cœur de l’algorithme ARS
│   ├── Drivers/BSP/                   ← Board Support Package Nucleo
│   ├── arstest.ioc                    ← Configuration CubeMX
│   ├── STM32H723ZGTX_FLASH.ld
│   └── ...
│
├── host/                              ← Scripts côté PC
│   ├── example_receiver.py            ← Réception + visualisation
│   └── requirements.txt
│
├── docs/
│   └── rapport_projet.pdf             ← Rapport complet (~100 pages)
│
└── assets/                            ← Images / figures (à compléter)
```

> **Note importante** : Les bibliothèques HAL et CMSIS (standard ST) ne sont pas incluses dans le dépôt (pratique recommandée).  
> Elles sont **générées automatiquement** en ouvrant le fichier `arstest.ioc` avec STM32CubeMX ou STM32CubeIDE.

---

## Mise en route rapide

### 1. Firmware (carte Nucleo-H723ZG)

1. Ouvrir **STM32CubeIDE**
2. *File → Open Projects from File System* → sélectionner le dossier `firmware/`
3. Ou ouvrir `firmware/arstest.ioc` avec **CubeMX** puis générer le code
4. **Attention** : ne pas écraser les sections `/* USER CODE BEGIN */` … `/* USER CODE END */` dans `main.c`
5. Compiler et flasher la carte

### 2. Connexions de test (boucle fermée)

| Broche   | Fonction                  |
|----------|---------------------------|
| **PA4**  | DAC1_OUT1 (signal carré)  |
| **PF11** | ADC1_IN2 (mesure)         |
| **PD8**  | USART3_TX                 |
| **PD9**  | USART3_RX                 |

→ Relier **PA4 ↔ PF11** avec un fil pour les tests.

### 3. Réception des données sur PC

```bash
cd host
pip install -r requirements.txt

# Linux
python example_receiver.py --port /dev/ttyACM0

# Windows
python example_receiver.py --port COM3
```

Le script affiche le signal temporel et l’histogramme des intervalles \(\tau_n\).

---

## Protocole de communication

Chaque bloc de 4000 points est envoyé en texte ASCII :

```text
START
1234,2048
3456,1024
...
END
```

- 1ʳᵉ colonne : timestamp en **microsecondes**
- 2ᵉ colonne  : valeur ADC **12 bits** (0 – 4095)
- Baudrate    : **115200** (8N1)

---

## Contenu du rapport (docs/rapport_projet.pdf)

Le rapport complet (~100 pages) couvre :

1. **Contexte industriel** et enjeux HUMS aéronautiques
2. **Fondements théoriques** de l’échantillonnage aléatoire (ARS vs JRS)
3. **Dimensionnement** des paramètres (choix de \(R = 0.8\))
4. **Estimation spectrale** non uniforme (NUDFT, Welch + zéro-insertion, Lomb-Scargle)
5. **Évaluation économique** du concept
6. **Architecture embarquée** détaillée
7. **Validation expérimentale** (signal carré 1 kHz + signal bi-tonal)
8. **Interface web** Plotly Dash
9. Perspectives **TinyML**

---

## Résultats clés

- Distribution des intervalles \(\tau_n\) conforme à la loi uniforme \([a, b]\)
- Spectre de Welch après zéro-insertion : pic clair à 1 kHz sans alias déterministe
- Validation réussie sur signal bi-tonal
- Prototype fonctionnel prêt pour intégration capteur réel

---

## Auteurs & Remerciements

Projet réalisé à l’**École Polytechnique de Tunisie (EPT)**.

Nous remercions chaleureusement :
- **Safran** pour avoir proposé cette problématique industrielle stimulante
- L’équipe pédagogique de l’EPT pour la formation en traitement du signal et systèmes embarqués

---

## Licence

Distribué sous licence **MIT**.  
Voir le fichier [LICENSE](LICENSE).

Les fichiers HAL/CMSIS de STMicroelectronics restent soumis à leurs licences respectives.

---

<div align="center">

*Projet académique — Non certifié pour usage aéronautique opérationnel*

</div>
