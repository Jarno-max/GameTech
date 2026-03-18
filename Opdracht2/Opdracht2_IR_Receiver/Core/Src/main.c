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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rc5_decode.h"
#include <stdio.h>
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

/* USER CODE BEGIN PV */
RC5_Frame_t IR_FRAME;

volatile uint32_t tim2_ch1_edge_count = 0;
volatile uint32_t tim2_ch2_edge_count = 0;

/* Timing-diagnostiek: min/max gemeten tijden (in us ticks) per seconde. */
volatile uint32_t tim2_ch1_min = 0xFFFFFFFFu;
volatile uint32_t tim2_ch1_max = 0;
volatile uint32_t tim2_ch2_min = 0xFFFFFFFFu;
volatile uint32_t tim2_ch2_max = 0;

/* Voor CH1 (falling): bereken high-pulse = period - lastLowPulse */
static volatile uint32_t tim2_last_low_pulse = 0;
static volatile uint8_t tim2_have_low_pulse = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Stuur printf() output naar USART2 (PuTTY) */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* Timer Input Capture callback - verwerkt RC5 flankmetingen */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      /* CH1: dalende flank - periode tussen twee dalende flanken */
      uint32_t period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

      /* Filter softwarematig zeer korte glitches/spikes */
      if (period < 200u)
      {
        return;
      }

      /* De RC5 decoder verwacht T/2T tussen opeenvolgende flanken.
         Period (fall->fall) kan 3T/4T worden. Daarom: highPulse = period - lowPulse. */
      if (tim2_have_low_pulse && (period > tim2_last_low_pulse))
      {
        uint32_t highPulse = period - tim2_last_low_pulse;

        if (highPulse < 200u)
        {
          return;
        }

        tim2_ch1_edge_count++;
        if (highPulse < tim2_ch1_min) tim2_ch1_min = highPulse;
        if (highPulse > tim2_ch1_max) tim2_ch1_max = highPulse;
        /* TSOP4838 output is active-low (inverted): treat actual FALLING as logical RISING */
        RC5_DataSampling(highPulse, 1);
      }
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
      /* CH2: stijgende flank - duur van de lage puls */
      uint32_t lowPulse = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

      /* Filter softwarematig zeer korte glitches/spikes */
      if (lowPulse < 200u)
      {
        return;
      }

      tim2_ch2_edge_count++;
      if (lowPulse < tim2_ch2_min) tim2_ch2_min = lowPulse;
      if (lowPulse > tim2_ch2_max) tim2_ch2_max = lowPulse;

      tim2_last_low_pulse = lowPulse;
      tim2_have_low_pulse = 1;

      /* TSOP4838 output is active-low (inverted): treat actual RISING as logical FALLING */
      RC5_DataSampling(lowPulse, 0);
    }
  }
}

/* Timer overflow callback - geen signaal na ~3.6ms: reset pakket */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    RC5_ResetPacket();
    tim2_have_low_pulse = 0;
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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  
  /* Schakel stdio-buffering uit zodat printf direct zichtbaar is in PuTTY */
  setvbuf(stdout, NULL, _IONBF, 0);

    /* Kleine delay na (re)programmeren/reset zodat de USB VCP en terminal
      tijd hebben om stabiel te worden. Dit voorkomt soms 'rare tekens' bij de
      eerste bytes na flashen. */
    HAL_Delay(100);

  /* Zet URS=1: Update-interrupt wordt alleen gegenereerd bij overflow,
     NIET bij slave-reset trigger. Anders roept elke dalende flank
     RC5_ResetPacket() aan en kan er nooit een volledig frame worden opgebouwd. */
  TIM2->CR1 |= TIM_CR1_URS;

  /* Initialiseer RC5 tijdsdrempelwaarden (1 tick = 1us met prescaler=31 @ 32MHz) */
  RC5_Init_Timing();

  /* Start TIM2 Input Capture interrupts */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

  /* Activeer timer overflow interrupt voor timeout-detectie */
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);

  const char startup_msg[] = "\r\nRC5 IR Receiver klaar. Richt afstandsbediening op TSOP4838.\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)startup_msg, sizeof(startup_msg)-1, HAL_MAX_DELAY);

  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_diag_print_ms = HAL_GetTick();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    HAL_Delay(100); // 100ms knipperen ter controle of main loop draait

    /* Diagnostiek: toon of er IR-flanken binnenkomen op TIM2.
       Verwachting: bij IR-activiteit lopen deze tellers op. */
    if ((HAL_GetTick() - last_diag_print_ms) >= 1000)
    {
      uint32_t ch1 = tim2_ch1_edge_count;
      uint32_t ch2 = tim2_ch2_edge_count;
      tim2_ch1_edge_count = 0;
      tim2_ch2_edge_count = 0;

      uint32_t ch1_min = tim2_ch1_min;
      uint32_t ch1_max = tim2_ch1_max;
      uint32_t ch2_min = tim2_ch2_min;
      uint32_t ch2_max = tim2_ch2_max;
      tim2_ch1_min = 0xFFFFFFFFu;
      tim2_ch1_max = 0;
      tim2_ch2_min = 0xFFFFFFFFu;
      tim2_ch2_max = 0;

      GPIO_PinState pa0_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

      char diag[140];
      int dlen;
      if (ch1 == 0 && ch2 == 0)
      {
        dlen = snprintf(diag, sizeof(diag), "[DIAG] PA0=%d | TIM2 edges/s: CH1=%lu CH2=%lu\r\n",
                        (int)pa0_state, (unsigned long)ch1, (unsigned long)ch2);
      }
      else
      {
        /* Waarden zijn in us-ticks (TIM2 tick = 1us). Verwacht bij RC5 typisch ~889us of ~1778us. */
        dlen = snprintf(diag, sizeof(diag),
                        "[DIAG] PA0=%d | edges/s CH1=%lu CH2=%lu | CH1(us) min=%lu max=%lu | CH2(us) min=%lu max=%lu\r\n",
                        (int)pa0_state,
                        (unsigned long)ch1, (unsigned long)ch2,
                        (unsigned long)ch1_min, (unsigned long)ch1_max,
                        (unsigned long)ch2_min, (unsigned long)ch2_max);
      }
      if (dlen < 0)
      {
        dlen = 0;
      }
      else if ((size_t)dlen > (sizeof(diag) - 1u))
      {
        /* snprintf returns the would-have-been length; clamp to avoid out-of-bounds transmit */
        dlen = (int)(sizeof(diag) - 1u);
      }
      HAL_UART_Transmit(&huart2, (uint8_t*)diag, (uint16_t)dlen, 100);
      last_diag_print_ms = HAL_GetTick();
    }

    if (RC5FrameReceived != NO)
    {
      RC5_Decode(&IR_FRAME);
      
      char buf[64];
      int len = snprintf(buf, sizeof(buf), "[RC5] Adres: 0x%02X | Commando: 0x%02X | Toggle: %d\r\n",
                         IR_FRAME.Address, IR_FRAME.Command, IR_FRAME.ToggleBit);
      if (len < 0)
      {
        len = 0;
      }
      else if ((size_t)len > (sizeof(buf) - 1u))
      {
        len = (int)(sizeof(buf) - 1u);
      }
      HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)len, 100);
      
      RC5FrameReceived = NO; // Vergeet niet de flag te resetten, anders blijf je printen!
    }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  /* Use HSI16 for maximum robustness (no LSE crystal needed, no PLL needed). */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* No MSI auto-calibration needed (HSI-based clock). */
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
  /* Fail-safe: blink LD3 (PB3) even if clocks/UART init failed. */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /* Configure PB3 as push-pull output (no HAL dependency). */
  GPIOB->MODER &= ~(3u << (3u * 2u));
  GPIOB->MODER |=  (1u << (3u * 2u));
  GPIOB->OTYPER &= ~(1u << 3u);
  GPIOB->PUPDR  &= ~(3u << (3u * 2u));
  while (1)
  {
    GPIOB->ODR ^= (1u << 3u);
    for (volatile uint32_t i = 0; i < 200000u; i++)
    {
      __NOP();
    }
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
