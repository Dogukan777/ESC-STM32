/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : PPM (TIM2 CH1 PA0) -> ESC PWM (TIM3 CH1 PA6) + OLED display
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include "SH1106.h"
#include "fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* -------------------- PPM settings -------------------- */
#define PPM_CH_MAX          10
#define PPM_SYNC_US         3000   // >3ms -> sync gap
#define PPM_PULSE_MIN_US    800
#define PPM_PULSE_MAX_US    2200

// Kanal indexleri (PPM sırası: CH1=0, CH2=1, CH3=2, CH4=3 ...)
#define ROLL_CH_INDEX       0  // CH1
#define PITCH_CH_INDEX      1  // CH2
#define THROTTLE_CH_INDEX   2  // CH3
#define YAW_CH_INDEX        3  // CH4

/* PPM decoded channels (us) */
volatile uint16_t ppm_ch[PPM_CH_MAX] = {1500};
volatile uint8_t  ppm_ch_count = 0;

/* Timing */
static volatile uint32_t last_cap = 0;
static volatile uint8_t  ch_idx = 0;

/* Failsafe timer */
volatile uint32_t ppm_last_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
static inline void ESC_WriteUs(uint16_t us);
static inline uint16_t map_u16(uint16_t x,
                               uint16_t in_min, uint16_t in_max,
                               uint16_t out_min, uint16_t out_max);
static inline int16_t ppm_to_percent(uint16_t us);
void displayScreen(const char *str);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();

  SH1106_Init();

  /* ESC PWM start */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  /* ESC arming: minimum pulse */
  ESC_WriteUs(1000);
  HAL_Delay(3000);

  /* PPM input capture start */
  ppm_last_ms = HAL_GetTick();
  last_cap = 0;
  ch_idx = 0;
  ppm_ch_count = 0;

  displayScreen("Booting...");
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  static uint16_t thr_f    = 1000;
  static uint16_t last_out = 1000;
  uint32_t t_esc = 0, t_oled = 0;
  while(1){
    uint32_t now = HAL_GetTick();

    // failsafe aynen...

    if (now - t_esc >= 5) {              // 5ms => 200Hz ESC update
      t_esc = now;

      uint16_t thr = ppm_ch[THROTTLE_CH_INDEX];
      if (thr < 1000) thr = 1000;
      if (thr > 2000) thr = 2000;

      thr_f = (uint16_t)((thr_f * 19 + thr) / 20);   // daha yumuşak (0.95)
      uint16_t out = thr_f;
      if ((out > last_out && out - last_out <= 2) || (last_out > out && last_out - out <= 2)) {
        out = last_out; // deadband 2us
      }
      last_out = out;

      ESC_WriteUs(last_out);     // map etme gerek yok, direkt 1000..2000 yolla
    }


     /* int16_t roll_pct  = ppm_to_percent(ppm_ch[ROLL_CH_INDEX]);
      int16_t pitch_pct = ppm_to_percent(ppm_ch[PITCH_CH_INDEX]);
      int16_t yaw_pct   = ppm_to_percent(ppm_ch[YAW_CH_INDEX]);

      char l1[24], l2[24], l3[24], l4[24];

      snprintf(l1, sizeof(l1), "ROLL : %4d%%", roll_pct);
      snprintf(l2, sizeof(l2), "PITCH: %4d%%", pitch_pct);
      snprintf(l3, sizeof(l3), "YAW  : %4d%%", yaw_pct);
      snprintf(l4, sizeof(l4), "THR  : %4uus", last_out); // <-- motora giden

      SH1106_Clear();
      SH1106_GotoXY(0, 0);   SH1106_Puts(l1, &Font_7x10, 1);
      SH1106_GotoXY(0, 12);  SH1106_Puts(l2, &Font_7x10, 1);
      SH1106_GotoXY(0, 24);  SH1106_Puts(l3, &Font_7x10, 1);
      SH1106_GotoXY(0, 36);  SH1106_Puts(l4, &Font_7x10, 1);
      SH1106_UpdateScreen();*/


  }
}


/**
  * @brief TIM2 input capture callback (RISING edge only)
  * diff = time between rising edges (us) -> TIM2 tick MUST be 1us
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    uint32_t cap = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    uint32_t diff;
    if (cap >= last_cap) diff = cap - last_cap;
    else diff = (0xFFFFFFFFu - last_cap + cap + 1u);

    last_cap = cap;
    if (diff == 0) return;

    /* Sync gap -> new frame */
    if (diff > PPM_SYNC_US)
    {
      ch_idx = 0;
      ppm_ch_count = 0;
      return;
    }

    /* Channel pulse width */
    if (diff >= PPM_PULSE_MIN_US && diff <= PPM_PULSE_MAX_US)
    {
      if (ch_idx < PPM_CH_MAX)
      {
        ppm_ch[ch_idx] = (uint16_t)diff;
        ch_idx++;
        ppm_ch_count = ch_idx;
        ppm_last_ms = HAL_GetTick();
      }
    }
  }
}

/* ---- helpers ---- */
static inline void ESC_WriteUs(uint16_t us)
{
  if (us < 1000) us = 1000;
  if (us > 2000) us = 2000;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, us);
}

static inline uint16_t map_u16(uint16_t x,
                               uint16_t in_min, uint16_t in_max,
                               uint16_t out_min, uint16_t out_max)
{
  if (x <= in_min) return out_min;
  if (x >= in_max) return out_max;
  return (uint16_t)(out_min +
                    (uint32_t)(x - in_min) * (out_max - out_min) /
                    (in_max - in_min));
}

/* 1000..2000us -> -100..+100 (%), 1500us -> 0% */
static inline int16_t ppm_to_percent(uint16_t us)
{
  if (us < 1000) us = 1000;
  if (us > 2000) us = 2000;
  return (int16_t)(((int32_t)us - 1500) * 100 / 500);
}

void displayScreen(const char *str)
{
  SH1106_Clear();
  SH1106_GotoXY(0, 0);
  SH1106_Puts((char*)str, &Font_7x10, 1);
  SH1106_UpdateScreen();
}

/* --- CubeMX generated init functions below --- */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim2.Instance = TIM2;

  /* ÖNEMLİ:
     TIM2 tick 1us olmalı. Timer clock'un kaç MHz ise:
     PSC = (timer_clk_MHz - 1)
     Örn 16MHz ise PSC=15.
     Senin ioc'tan ayarlaman daha doğru. */
  htim2.Init.Prescaler = 15; // <-- gerekirse .ioc'a göre değiştir
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
}

static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15; // 1us tick (16MHz/(15+1)=1MHz)
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999; // 20ms
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 800;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

  HAL_TIM_MspPostInit(&htim3);
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif /* USE_FULL_ASSERT */
