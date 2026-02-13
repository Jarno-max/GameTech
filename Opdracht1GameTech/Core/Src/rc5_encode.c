/**
  ******************************************************************************
  * @file    rc5_encode.c
  * @author  MCD Application Team (adapted for STM32L432KC)
  * @brief   RC5 encoder implementation for 38kHz IR transmission
  ******************************************************************************
  */

#include "main.h"
#include "rc5_encode.h"
#include "ir_common.h"

/* RC5 Protocol Definitions */
#define RC5HIGHSTATE          ((uint8_t)0x02)   /* Manchester high level */
#define RC5LOWSTATE           ((uint8_t)0x01)   /* Manchester low level */

/* Private variables */
uint8_t RC5RealFrameLength = 14;              /* RC5 frame is 14 bits */
uint8_t RC5GlobalFrameLength = 64;            /* Total frame including Manchester */
uint16_t RC5BinaryFrameFormat = 0;            /* Binary RC5 frame */
uint32_t RC5ManchesterFrameFormat = 0;        /* Manchester encoded frame */
__IO uint32_t RC5SendOpCompleteFlag = 1;      /* Send complete flag */
__IO uint32_t RC5SendOpReadyFlag = 0;         /* Send ready flag */
uint8_t BitsSentCounter = 0;                  /* Bit counter for transmission */
RC5_Ctrl_t RC5Ctrl = RC5_CTRL_RESET;          /* Control bit state */

/* Timer handles (defined in main.c) */
TIM_HandleTypeDef TimHandleHF;  /* TIM15: 38kHz carrier */
TIM_HandleTypeDef TimHandleLF;  /* TIM16: 889us bit timing */

/* Private function prototypes */
static uint16_t RC5_BinFrameGeneration(uint8_t RC5_Address, uint8_t RC5_Instruction, RC5_Ctrl_t RC5_Ctrl);
static uint32_t RC5_ManchesterConvert(uint16_t RC5_BinaryFrameFormat);

/**
  * @brief  Initialize RC5 encoder - Configure timers for IR transmission
  * @param  None
  * @retval None
  */
void RC5_Encode_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  
  /* HIGH FREQUENCY TIMER (TIM15) - 38kHz Carrier Wave */
  /* This timer generates the 38kHz modulation carrier */
  
  TimHandleHF.Instance = TIM15;
  TimHandleHF.Init.Prescaler = 31;                    /* 32MHz / 32 = 1MHz */
  TimHandleHF.Init.CounterMode = TIM_COUNTERMODE_UP;
  TimHandleHF.Init.Period = IR_ENC_LPERIOD_RC5;       /* 889 for 889us period */
  TimHandleHF.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TimHandleHF.Init.RepetitionCounter = 0;
  TimHandleHF.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  
  if (HAL_TIM_Base_Init(&TimHandleHF) != HAL_OK)
  {
    Error_Handler();
  }
  
  if (HAL_TIM_OC_Init(&TimHandleHF) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Configure Output Compare for timing control */
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = IR_ENC_LPERIOD_RC5;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  
  if (HAL_TIM_OC_ConfigChannel(&TimHandleHF, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Enable TIM15 interrupt */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
  
  /* LOW FREQUENCY TIMER (TIM16) - 38kHz PWM Carrier */
  /* This timer generates the actual 38kHz PWM signal with 25% duty cycle */
  
  TimHandleLF.Instance = TIM16;
  TimHandleLF.Init.Prescaler = 0;                     /* No prescaler: 32MHz */
  TimHandleLF.Init.CounterMode = TIM_COUNTERMODE_UP;
  TimHandleLF.Init.Period = IR_ENC_HPERIOD_RC5 - 1;   /* 841 for 38kHz */
  TimHandleLF.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TimHandleLF.Init.RepetitionCounter = 0;
  TimHandleLF.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  
  if (HAL_TIM_Base_Init(&TimHandleLF) != HAL_OK)
  {
    Error_Handler();
  }
  
  if (HAL_TIM_PWM_Init(&TimHandleLF) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Configure PWM Channel with 25% duty cycle */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = IR_ENC_HPERIOD_RC5 / 4;           /* 25% duty cycle */
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  
  if (HAL_TIM_PWM_ConfigChannel(&TimHandleLF, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Start PWM output (but forced inactive initially) */
  HAL_TIM_PWM_Start(&TimHandleLF, TIM_CHANNEL_1);
  TIM_ForcedOC1Config(TIM_FORCED_INACTIVE);
}

/**
  * @brief  Send RC5 frame
  * @param  RC5_Address: Device address (0-31)
  * @param  RC5_Instruction: Command instruction (0-127)
  * @param  RC5_Ctrl: Control bit state
  * @retval None
  */
void RC5_Encode_SendFrame(uint8_t RC5_Address, uint8_t RC5_Instruction, RC5_Ctrl_t RC5_Ctrl)
{
  /* Wait if previous transmission is still ongoing */
  while (RC5SendOpCompleteFlag == 0)
  {
  }
  
  /* Generate binary format of the frame */
  RC5BinaryFrameFormat = RC5_BinFrameGeneration(RC5_Address, RC5_Instruction, RC5_Ctrl);
  
  /* Convert to Manchester format */
  RC5ManchesterFrameFormat = RC5_ManchesterConvert(RC5BinaryFrameFormat);
  
  /* Prepare for transmission */
  RC5SendOpReadyFlag = 1;
  BitsSentCounter = 0;
  
  /* Reset counter and start timer interrupt */
  __HAL_TIM_SET_COUNTER(&TimHandleHF, 0);
  HAL_TIM_Base_Start_IT(&TimHandleHF);
}

/**
  * @brief  Generate RC5 signal - Called from timer interrupt
  * @param  None
  * @retval None
  */
void RC5_Encode_SignalGenerate(void)
{
  uint32_t bit_msg = 0;
  
  if ((RC5SendOpReadyFlag == 1) && (BitsSentCounter < (RC5GlobalFrameLength * 2)))
  {
    RC5SendOpCompleteFlag = 0;
    
    /* Extract current bit from Manchester frame */
    bit_msg = (RC5ManchesterFrameFormat >> BitsSentCounter) & 1;
    
    /* Set output state based on bit value */
    if (bit_msg == 1)
    {
      TIM_ForcedOC1Config(TIM_FORCED_ACTIVE);    /* Enable 38kHz carrier */
    }
    else
    {
      TIM_ForcedOC1Config(TIM_FORCED_INACTIVE);  /* Disable carrier */
    }
    
    BitsSentCounter++;
  }
  else
  {
    /* Transmission complete */
    RC5SendOpCompleteFlag = 1;
    HAL_TIM_Base_Stop_IT(&TimHandleHF);
    RC5SendOpReadyFlag = 0;
    BitsSentCounter = 0;
    TIM_ForcedOC1Config(TIM_FORCED_INACTIVE);
    __HAL_TIM_DISABLE(&TimHandleHF);
  }
}

/**
  * @brief  Generate binary RC5 frame
  * @param  RC5_Address: Device address
  * @param  RC5_Instruction: Command instruction
  * @param  RC5_Ctrl: Control bit
  * @retval RC5 frame in binary format
  */
static uint16_t RC5_BinFrameGeneration(uint8_t RC5_Address, uint8_t RC5_Instruction, RC5_Ctrl_t RC5_Ctrl)
{
  uint16_t star1 = 0x2000;  /* Start bit 1 (always 1) */
  uint16_t star2 = 0x1000;  /* Start bit 2 (field bit) */
  uint16_t addr = 0;
  
  /* Check if instruction is extended (7-bit) */
  if (RC5_Instruction >= 64)
  {
    star2 = 0;  /* Field bit = 0 for extended commands */
    RC5_Instruction &= 0x003F;  /* Keep only lower 6 bits */
  }
  
  /* Build frame: [S1][S2][C][A4][A3][A2][A1][A0][I5][I4][I3][I2][I1][I0] */
  addr = ((uint16_t)(RC5_Address & 0x1F)) << 6;
  
  return (star1 | star2 | RC5_Ctrl | addr | (RC5_Instruction & 0x3F));
}

/**
  * @brief  Convert binary RC5 frame to Manchester encoding
  * @param  RC5_BinaryFrameFormat: Binary RC5 frame
  * @retval Manchester encoded frame
  * @note   Manchester encoding: 0 = 01, 1 = 10
  */
static uint32_t RC5_ManchesterConvert(uint16_t RC5_BinaryFrameFormat)
{
  uint8_t i = 0;
  uint16_t Mask = 1;
  uint16_t bit_format = 0;
  uint32_t ConvertedMsg = 0;
  
  for (i = 0; i < RC5RealFrameLength; i++)
  {
    bit_format = ((RC5_BinaryFrameFormat >> i) & Mask);
    ConvertedMsg = ConvertedMsg << 2;
    
    if (bit_format != 0)
    {
      ConvertedMsg |= RC5HIGHSTATE;  /* Manchester: 1 = 10 */
    }
    else
    {
      ConvertedMsg |= RC5LOWSTATE;   /* Manchester: 0 = 01 */
    }
  }
  
  return ConvertedMsg;
}

/**
  * @brief  Force TIM16 output compare state
  * @param  action: TIM_FORCED_ACTIVE or TIM_FORCED_INACTIVE
  * @retval None
  */
void TIM_ForcedOC1Config(uint32_t action)
{
  /* Modify CCMR1 register to force output state */
  uint32_t tmpccmr1 = TimHandleLF.Instance->CCMR1;
  
  /* Clear OC1M bits */
  tmpccmr1 &= ~TIM_CCMR1_OC1M;
  
  /* Set forced output mode */
  tmpccmr1 |= action;
  
  /* Write to CCMR1 register */
  TimHandleLF.Instance->CCMR1 = tmpccmr1;
}
