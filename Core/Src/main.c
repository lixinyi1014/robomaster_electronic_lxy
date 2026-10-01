/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "iwdg.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct MusicNote {
  uint16_t note;
  uint32_t time;
} MusicNote_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_PSC          1000
#define MAX_BUZZER_PWM   20000
#define MIN_BUZZER_PWM   10000


#define L1 262
#define L1U 277
#define L2 294
#define L2U 311
#define L3 330
#define L4 349
#define L4U 370
#define L5 392
#define L5U 415
#define L6 440
#define L6U 466
#define L7 494

#define M1 523
#define M1U 554
#define M2 587
#define M2U 622
#define M3 659
#define M4 698
#define M4U 740
#define M5 784
#define M5U 831
#define M6 880
#define M6U 932
#define M7 988

#define H1 1046
#define H1U 1109
#define H2 1175
#define H2U 1245
#define H3 1318
#define H4 1397
#define H4U 1480
#define H5 1568
#define H5U 1661
#define H6 1760
#define H6U 1865
#define H7 1976
const MusicNote_t see_you_again[] = {
  {M5, 343},
{0, 38},
   {H2, 343},
{0, 38},
   {H1, 343},
{0, 38},
   {M3, 686},
{0, 76},
   {H1, 171},
{0, 19},
   {H2, 171},
{0, 19},
   {H3, 171},
{0, 19},
   {H2, 171},
{0, 19},
   {H1, 171},
{0, 19},
   {H2, 171},
{0, 19},
   {M5, 343},
{0, 38},
   {H2, 343},
{0, 38},
   {H1, 343},
{0, 38},
   {M5, 686},
{0, 76},
   {H1, 171},
{0, 19},
   {H2, 171},
{0, 19},
   {H3, 171},
{0, 19},
   {H2, 171},
{0, 19},
   {H1, 171},
{0, 19},
   {H2, 171},
{0, 19},
   {M5, 343},
{0, 38},
   {H2, 343},
{0, 38},
   {H1, 343},
{0, 38},
   {M5, 686},
{0, 76},
   {H1, 171},
{0, 19},
   {H2, 171},
{0, 19},
   {H3, 171},
{0, 19},
   {H2, 171},
{0, 19},
   {H1, 171},
{0, 19},
   {H2, 171},
{0, 19},
   {M5, 343},
{0, 38},
   {H2, 343},
{0, 38},
   {H1, 343},
{0, 38},
   {M5, 686},
{0, 76},
   {M1, 171},
  {0,19},
   {M1, 171},
{0, 19},
   {M3, 343},
{0, 38},
   {M5, 343},
  {0, 38},
   {M6, 800},
{0, 90},
   {M5, 500},
{0, 56},
   {M6, 686},
{0, 76},
  {M1, 686},
{0, 76},
  {M2, 343},
{0, 38},
  {M2, 343},
{0, 38},
  {M1, 343},
{0, 38},
  {M3, 800},
{0, 90},
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t psc = 0;
uint16_t pwm = MIN_BUZZER_PWM;
volatile uint8_t requested_mode = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void buzzer_note(uint16_t freq)
{
   if (freq == 0)                                    // 休止符
   {
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
      return;
   }
   uint32_t arr = 1000000 / freq - 1;                // 1 MHz 计数时钟 → 目标频率
   __HAL_TIM_SET_AUTORELOAD(&htim4, arr);            // 改频率（音调）
   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 5);  // 50% 占空比（最响）
   __HAL_TIM_SET_COUNTER(&htim4, 0);                 // 计数器清零，防止切换时卡顿
}
void buzzer_off(void)
{
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);        // 占空比 0，不响
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
   if (GPIO_Pin == KEY_Pin)
   {
      static uint32_t last_key_tick = 0;
      static uint8_t has_last_key_tick = 0;
      uint32_t now = HAL_GetTick();
      if (!has_last_key_tick || (now - last_key_tick) >= 30U)
      {
         has_last_key_tick = 1;
         last_key_tick = now;
         requested_mode = (requested_mode + 1U) % 3U;
         //__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, requested_mode * 450 + 50);
      }
   }
}
#define BREATH_STEP_MS   2      // 呼吸
#define BLINK_HALF_MS    500    // 闪烁

static void LedSet(uint16_t ccr)
{
   __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, ccr);
}

void UpdateLed(uint8_t mode, uint32_t now)
{
   //static uint8_t  current_mode = 0;
   static uint32_t last_tick = 0;
   static uint16_t pulse = 0;            // 呼吸
   static int8_t   dir = 1;              // 呼吸
   static uint8_t  blink_on = 0;         

   switch (mode)
   {
      case 1:
         if (now - last_tick >= BREATH_STEP_MS)
         {
            last_tick = now;
            pulse += dir;
            if (pulse >= 999)  dir = -1;
            else if (pulse <= 0) dir = 1;
            LedSet(pulse);
         }
         break;

      case 2:
         if (now - last_tick >= BLINK_HALF_MS)
         {
            last_tick = now;
            blink_on = !blink_on;
            LedSet(blink_on ? 500 : 0);
         }
         break;

      default:
         LedSet(0);
         break;
   }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
   __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);   // 上电先灭
   HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while (1)
   {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      uint8_t mode = requested_mode;
      UpdateLed(mode, HAL_GetTick());
      HAL_IWDG_Refresh(&hiwdg);
   }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
