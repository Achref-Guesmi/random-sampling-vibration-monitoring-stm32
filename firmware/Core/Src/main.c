/* USER CODE BEGIN Header */

/**

******************************************************************************

* @file : main.c

* @brief : ARS (Additive Random Sampling) - STM32H723ZG

*

* Clocks réels (PLLFRACN=3072 pris en compte) :

* SYSCLK = 550 MHz, HCLK = 275 MHz

* APB1 = 137.5 MHz → TIM_CLK = 275 MHz

* TIM2 PSC=274 → tick = 1 µs exactement

* TIM6 PSC=274, ARR=499 → trigger DAC @ 2 kHz → carré @ 1 kHz

*

* Connexions :

* PA4 → DAC1_OUT1 (signal carré 1 kHz)

* PF11 → ADC1_IN2 (fil relié à PA4)

* PD8 → USART3_TX (ST-Link USB vers PC)

* PD9 → USART3_RX

******************************************************************************

*/

/* USER CODE END Header */



/* Includes ------------------------------------------------------------------*/

#include "main.h"



/* Private includes ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */

#include <stdio.h>

#include <string.h>

/* USER CODE END Includes */



/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */



/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */



/* ── Paramètres ARS (Table 1 du papier Hajar et al. 2019) ─────────────────

* TS = 2 ms → fréquence moyenne = 500 Hz

* R = 0.8 → a = 1200 µs, b = 2800 µs

* N = 400 points par bloc envoyé

* Tick TIM2 = 1 µs (PSC=274, TIM_CLK=275 MHz)

* ────────────────────────────────────────────────────────────────────────── */

#define ARS_N 4000u /* Nombre d'échantillons par bloc */

#define ARS_A_TICKS 1200u /* Borne inférieure en ticks (= µs) */

#define ARS_B_TICKS 2800u /* Borne supérieure en ticks (= µs) */

#define ARS_RANGE (ARS_B_TICKS - ARS_A_TICKS) /* 1600 ticks */



/* Valeurs DAC pour signal carré 12 bits pleine échelle */

#define DAC_LOW 0u

#define DAC_HIGH 4095u



/* USER CODE END PD */



/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */



/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

DMA_HandleTypeDef hdma_dac1_ch1;

RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim2;

TIM_HandleTypeDef htim6;



/* USER CODE BEGIN PV */



/* ── USART3 handle (déclaré ici, initialisé dans MX_USART3_UART_Init) ───── */

UART_HandleTypeDef huart3;



/* ── Buffers ARS ─────────────────────────────────────────────────────────── */

static uint32_t adc_samples[ARS_N]; /* valeurs ADC brutes 12 bits */

static uint32_t time_stamps[ARS_N]; /* instants en µs depuis début du bloc */



/* ── Variables de contrôle (modifiées en ISR → volatile) ────────────────── */

static volatile uint8_t adc_ready = 0; /* 1 = conversion terminée */

static volatile uint32_t adc_value = 0; /* valeur lue dans le callback */



/* ── Compteur de temps courant et index buffer ───────────────────────────── */

static uint32_t t_current_us = 0;

static uint16_t buf_index = 0;



/* ── Tableau DMA pour le signal carré DAC (2 points) ────────────────────── */

static uint16_t dac_wave[2] = {DAC_LOW, DAC_HIGH};



/* ── Tampon de formatage UART ────────────────────────────────────────────── */

static char uart_buf[40];



/* USER CODE END PV */



/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

static void MPU_Config(void);

static void MX_GPIO_Init(void);

static void MX_DMA_Init(void);

static void MX_DAC1_Init(void);

static void MX_TIM6_Init(void);

static void MX_ADC1_Init(void);

static void MX_RNG_Init(void);

static void MX_TIM2_Init(void);



/* USER CODE BEGIN PFP */

static void MX_USART3_UART_Init(void); /* ajouté manuellement */

static uint32_t ARS_GenerateTau(void); /* tirage uniforme [a, b] en ticks */

/* USER CODE END PFP */



/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */



/**

* @brief Callback HAL appelé automatiquement quand l'ADC finit une conversion.

* TIM2 déclenche l'ADC → ici on récupère la valeur et on lève le flag.

*/

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)

{

if (hadc->Instance == ADC1)

{

adc_value = HAL_ADC_GetValue(&hadc1);

adc_ready = 1;

}

}



/**

* @brief Génère un intervalle tau_n uniforme sur [ARS_A_TICKS, ARS_B_TICKS].

*

* On utilise le RNG matériel 32 bits.

* Modulo biaisé acceptable : RANGE=1600 << 2^32, biais < 4e-8.

*

* @retval tau en ticks TIM2 (= µs)

*/

static uint32_t ARS_GenerateTau(void)

{

uint32_t rng_val = 0;

if (HAL_RNG_GenerateRandomNumber(&hrng, &rng_val) != HAL_OK)

{

return (ARS_A_TICKS + ARS_B_TICKS) / 2u; /* valeur de repli = TS */

}

return ARS_A_TICKS + (rng_val % (ARS_RANGE + 1u));

}



/* USER CODE END 0 */



/**

* @brief The application entry point.

*/

int main(void)

{

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */



/* MPU Configuration -------------------------------------------------------*/

MPU_Config();



/* MCU Configuration -------------------------------------------------------*/

HAL_Init();



/* USER CODE BEGIN Init */

/* USER CODE END Init */



/* Configure the system clock */

SystemClock_Config();



/* USER CODE BEGIN SysInit */

/* USER CODE END SysInit */



/* Initialize all configured peripherals */

MX_GPIO_Init();

MX_DMA_Init();

MX_DAC1_Init();

MX_TIM6_Init();

MX_ADC1_Init();

MX_RNG_Init();

MX_TIM2_Init();



/* USER CODE BEGIN 2 */

MX_USART3_UART_Init(); /* USART3 initialisé ici (hors CubeMX) */



/* ── Initialiser les LEDs BSP (debug visuel) ───────────────────────── */

BSP_LED_Init(LED_GREEN);

BSP_LED_Init(LED_YELLOW);

BSP_LED_Init(LED_RED);



/* ── 1. Lancer le signal carré DAC + DMA en boucle infinie ─────────────

* TIM6 (ARR=499, PSC=274) → trigger @ 2 kHz

* DMA envoie {0, 4095} en circulaire → carré 1 kHz sur PA4

* ─────────────────────────────────────────────────────────────────── */

if (HAL_DAC_Start_DMA(&hdac1,

DAC_CHANNEL_1,

(uint32_t *)dac_wave,

2u,

DAC_ALIGN_12B_R) != HAL_OK)

{

Error_Handler();

}

HAL_TIM_Base_Start(&htim6);



/* ── 2. Lancer l'ADC en mode interruption (attend trigger TIM2) ─────── */

if (HAL_ADC_Start_IT(&hadc1) != HAL_OK)

{

Error_Handler();

}



/* ── 3. Petite pause : le DAC se stabilise avant le premier échantillon */

HAL_Delay(20);



/* Allumer LED verte = système prêt */

BSP_LED_On(LED_GREEN);



/* USER CODE END 2 */



/* Infinite loop -----------------------------------------------------------*/

/* USER CODE BEGIN WHILE */

while (1)

{

/* ══════════════════════════════════════════════════════════════════

* BOUCLE ARS :

* A. Tirer tau_n ∈ [a, b] (RNG matériel)

* B. Charger tau_n dans TIM2->ARR

* C. Démarrer TIM2 → trigger automatique de l'ADC à l'expiration

* D. Attendre le flag ADC (levé dans HAL_ADC_ConvCpltCallback)

* E. Sauvegarder t_n = t_(n-1) + tau_n et x_n = ADC

* F. Si buffer plein → envoyer par USART3 et recommencer

* ══════════════════════════════════════════════════════════════════ */



/* A. Tirage aléatoire */

uint32_t tau = ARS_GenerateTau();



/* B. Programmer TIM2 :

* ARR = tau - 1 car le timer compte 0 … ARR puis génère TRGO

* On remet aussi le compteur à 0 pour partir proprement */

__HAL_TIM_SET_AUTORELOAD(&htim2, tau - 1u);

__HAL_TIM_SET_COUNTER(&htim2, 0u);

adc_ready = 0; /* réinitialiser AVANT de démarrer (évite fausse détection) */



/* C. Démarrer TIM2 (one-shot géré logiquement : on l'arrête après le flag) */

HAL_TIM_Base_Start(&htim2);



/* D. Attendre conversion ADC — timeout sécurité 50 ms */

uint32_t t_deadline = HAL_GetTick() + 50u;

while (adc_ready == 0)

{

if (HAL_GetTick() >= t_deadline)

{

/* Timeout : point raté, on éteint TIM2 et on recommence */

HAL_TIM_Base_Stop(&htim2);

BSP_LED_Toggle(LED_RED); /* clignoter rouge = anomalie */

goto next_sample;

}

}



/* E. Enregistrer l'échantillon */

HAL_TIM_Base_Stop(&htim2);



t_current_us += tau;

time_stamps[buf_index] = t_current_us;

adc_samples[buf_index] = adc_value;

buf_index++;



/* F. Buffer plein → envoi UART ─────────────────────────────────── */

if (buf_index >= ARS_N)

{

BSP_LED_Toggle(LED_YELLOW); /* clignoter jaune = bloc envoyé */



/* Marqueur de début — Python attend "START\n" */

HAL_UART_Transmit(&huart3,

(uint8_t *)"START\n", 6u,

HAL_MAX_DELAY);



/* Envoyer les N points : "instant_us,valeur_adc\n" */

for (uint16_t i = 0u; i < ARS_N; i++)

{

int len = snprintf(uart_buf, sizeof(uart_buf),

"%lu,%lu\n",

(unsigned long)time_stamps[i],

(unsigned long)adc_samples[i]);

HAL_UART_Transmit(&huart3,

(uint8_t *)uart_buf,

(uint16_t)len,

HAL_MAX_DELAY);

}



/* Marqueur de fin */

HAL_UART_Transmit(&huart3,

(uint8_t *)"END\n", 4u,

HAL_MAX_DELAY);



/* Reset pour le prochain bloc */

buf_index = 0u;

t_current_us = 0u;



/* Relancer l'ADC (HAL_ADC_Start_IT ne se relance pas seul

* après une conversion en mode single + trigger externe) */

HAL_ADC_Start_IT(&hadc1);

}



next_sample:; /* étiquette goto pour le cas de timeout */



/* USER CODE END WHILE */

/* USER CODE BEGIN 3 */

}

/* USER CODE END 3 */

}



/* ============================================================================

* USART3 — Initialisation manuelle

* PD8 = TX (AF7), PD9 = RX (AF7)

* Relié au ST-Link → port COM virtuel sur le PC

* ========================================================================== */

/* USER CODE BEGIN 4 */

static void MX_USART3_UART_Init(void)

{

/* ── Activer les horloges ─────────────────────────────────────────── */

__HAL_RCC_USART3_CLK_ENABLE();

__HAL_RCC_GPIOD_CLK_ENABLE();



/* ── Configurer PD8 (TX) et PD9 (RX) en mode alternatif AF7 ─────── */

GPIO_InitTypeDef GPIO_InitStruct = {0};

GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;

GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;

GPIO_InitStruct.Pull = GPIO_NOPULL;

GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

GPIO_InitStruct.Alternate = GPIO_AF7_USART3;

HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);



/* ── Configurer USART3 ───────────────────────────────────────────── */

huart3.Instance = USART3;

huart3.Init.BaudRate = 115200;

huart3.Init.WordLength = UART_WORDLENGTH_8B;

huart3.Init.StopBits = UART_STOPBITS_1;

huart3.Init.Parity = UART_PARITY_NONE;

huart3.Init.Mode = UART_MODE_TX_RX;

huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;

huart3.Init.OverSampling = UART_OVERSAMPLING_16;

huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;

huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;

huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;



if (HAL_UART_Init(&huart3) != HAL_OK)

{

Error_Handler();

}



/* Prescaler FIFO (obligatoire sur H7) */

if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)

{

Error_Handler();

}

if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)

{

Error_Handler();

}

if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)

{

Error_Handler();

}

}

/* USER CODE END 4 */



/**

* @brief System Clock Configuration

*/

void SystemClock_Config(void)

{

RCC_OscInitTypeDef RCC_OscInitStruct = {0};

RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};



HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}



RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSI;

RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;

RCC_OscInitStruct.HSICalibrationValue = 64;

RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;

RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;

RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;

RCC_OscInitStruct.PLL.PLLM = 4;

RCC_OscInitStruct.PLL.PLLN = 34;

RCC_OscInitStruct.PLL.PLLP = 1;

RCC_OscInitStruct.PLL.PLLQ = 4;

RCC_OscInitStruct.PLL.PLLR = 2;

RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;

RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;

RCC_OscInitStruct.PLL.PLLFRACN = 3072;

if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }



RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK

| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2

| RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;

RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;

RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;

RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;

RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;

RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;

RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) { Error_Handler(); }

}



/* ── Initialisations périphériques (générées par CubeMX, corrigées) ─────── */



static void MX_ADC1_Init(void)

{

ADC_MultiModeTypeDef multimode = {0};

ADC_ChannelConfTypeDef sConfig = {0};



hadc1.Instance = ADC1;

hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;

hadc1.Init.Resolution = ADC_RESOLUTION_12B;

hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;

hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

hadc1.Init.LowPowerAutoWait = DISABLE;

hadc1.Init.ContinuousConvMode = DISABLE;

hadc1.Init.NbrOfConversion = 1;

hadc1.Init.DiscontinuousConvMode = DISABLE;

hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;

hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;

hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;

hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;

hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;

hadc1.Init.OversamplingMode = DISABLE;

if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }



multimode.Mode = ADC_MODE_INDEPENDENT;

if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) { Error_Handler(); }



sConfig.Channel = ADC_CHANNEL_2; /* PF11 = ADC1_IN2 */

sConfig.Rank = ADC_REGULAR_RANK_1;

sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;

sConfig.SingleDiff = ADC_SINGLE_ENDED;

sConfig.OffsetNumber = ADC_OFFSET_NONE;

sConfig.Offset = 0;

sConfig.OffsetSignedSaturation = DISABLE;

if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

}



static void MX_DAC1_Init(void)

{

DAC_ChannelConfTypeDef sConfig = {0};



hdac1.Instance = DAC1;

if (HAL_DAC_Init(&hdac1) != HAL_OK) { Error_Handler(); }



sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;

sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;

sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;

sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;

sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;

if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) { Error_Handler(); }

}



static void MX_RNG_Init(void)

{

hrng.Instance = RNG;

hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;

if (HAL_RNG_Init(&hrng) != HAL_OK) { Error_Handler(); }

}



static void MX_TIM2_Init(void)

{

/* TIM2 : timer ARS rechargé dynamiquement

* PSC = 274 → tick = 1 µs (TIM_CLK = 275 MHz)

* ARR sera écrasé à chaque itération dans la boucle ARS

* MasterOutput = UPDATE → trigger l'ADC1 */

TIM_ClockConfigTypeDef sClockSourceConfig = {0};

TIM_MasterConfigTypeDef sMasterConfig = {0};



htim2.Instance = TIM2;

htim2.Init.Prescaler = 274;

htim2.Init.CounterMode = TIM_COUNTERMODE_UP;

htim2.Init.Period = ARS_B_TICKS - 1u; /* valeur initiale */

htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }



sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }



sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;

sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }

}



static void MX_TIM6_Init(void)

{

/* TIM6 : horloge du DAC

* PSC = 274, ARR = 499 → trigger @ 275 MHz / 275 / 500 = 2000 Hz

* DMA envoie {0, 4095} → signal carré 1 kHz sur PA4

*

* CORRECTION par rapport au code CubeMX d'origine (ARR était 1374) :

* 1374 → trigger @ 727 Hz → carré @ 363 Hz ← FAUX

* 499 → trigger @ 2000 Hz → carré @ 1000 Hz ← CORRECT */

TIM_MasterConfigTypeDef sMasterConfig = {0};



htim6.Instance = TIM6;

htim6.Init.Prescaler = 274;

htim6.Init.CounterMode = TIM_COUNTERMODE_UP;

htim6.Init.Period = 499; /* ← valeur corrigée */

htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

if (HAL_TIM_Base_Init(&htim6) != HAL_OK) { Error_Handler(); }



sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;

sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) { Error_Handler(); }

}



static void MX_DMA_Init(void)

{

__HAL_RCC_DMA1_CLK_ENABLE();

HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);

HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}



static void MX_GPIO_Init(void)

{

__HAL_RCC_GPIOC_CLK_ENABLE();

__HAL_RCC_GPIOA_CLK_ENABLE();

__HAL_RCC_GPIOF_CLK_ENABLE();

/* PD8/PD9 sont initialisés dans MX_USART3_UART_Init */

}



/* MPU Configuration ---------------------------------------------------------*/

void MPU_Config(void)

{

MPU_Region_InitTypeDef MPU_InitStruct = {0};

HAL_MPU_Disable();



MPU_InitStruct.Enable = MPU_REGION_ENABLE;

MPU_InitStruct.Number = MPU_REGION_NUMBER0;

MPU_InitStruct.BaseAddress = 0x0;

MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;

MPU_InitStruct.SubRegionDisable = 0x87;

MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;

MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;

MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;

MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;

MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

HAL_MPU_ConfigRegion(&MPU_InitStruct);



HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}



/* Error Handler -------------------------------------------------------------*/

void Error_Handler(void)

{

/* USER CODE BEGIN Error_Handler_Debug */

__disable_irq();

BSP_LED_On(LED_RED);

while (1) {}

/* USER CODE END Error_Handler_Debug */

}



#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)

{

/* USER CODE BEGIN 6 */

(void)file; (void)line;

/* USER CODE END 6 */

}

#endif /* USE_FULL_ASSERT */
