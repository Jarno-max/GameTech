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

/* RC5 frame is 14 bits -> Manchester is 28 half-bits */
#define RC5_REAL_FRAME_LENGTH        ((uint8_t)14)
#define RC5_MANCHESTER_BITS_LENGTH   ((uint8_t)(RC5_REAL_FRAME_LENGTH * 2))

/* Private variables */
static uint16_t RC5BinaryFrameFormat = 0;      /* Binary RC5 frame */
static uint32_t RC5ManchesterFrameFormat = 0;  /* Manchester encoded frame (MSB-first) */
static __IO uint32_t RC5SendOpCompleteFlag = 1;
static __IO uint32_t RC5SendOpReadyFlag = 0;
uint8_t BitsSentCounter = 0;                   /* Used by ISR callback */

/* TIM handles provided by CubeMX (defined in main.c) */
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;

/* Private function prototypes */
static uint16_t RC5_BinFrameGeneration(uint8_t RC5_Address, uint8_t RC5_Instruction, RC5_Ctrl_t RC5_Ctrl);
static uint32_t RC5_ManchesterConvert(uint16_t rc5BinaryFrame);

void RC5_Carrier_Enable(uint8_t enable)
{
  /* Gate TIM16_CH1 without changing frequency/duty cycle:
     - enable: OC1M = PWM1 (carrier visible)
     - disable: OC1M = Forced inactive (pin driven low)
  */
  uint32_t ccmr1 = TIM16->CCMR1;
  ccmr1 &= ~TIM_CCMR1_OC1M;
  ccmr1 |= (enable ? TIM_OCMODE_PWM1 : TIM_OCMODE_FORCED_INACTIVE);
  TIM16->CCMR1 = ccmr1;

  /* Keep channel output enabled */
  TIM16->CCER |= TIM_CCER_CC1E;
}

/**
  * @brief  Initialize RC5 encoder - Configure timers for IR transmission
  * @param  None
  * @retval None
  */
void RC5_Encode_Init(void)
{
  /* CubeMX already configured TIM15 (889us) and TIM16 (38kHz PWM on PA6/A5).
     We only need to make sure the carrier starts disabled (idle low). */
  RC5_Carrier_Enable(0);
  BitsSentCounter = 0;
  RC5SendOpReadyFlag = 0;
  RC5SendOpCompleteFlag = 1;
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
  __HAL_TIM_SET_COUNTER(&htim15, 0);
  HAL_TIM_Base_Start_IT(&htim15);
}

/**
  * @brief  Generate RC5 signal - Called from timer interrupt
  * @param  None
  * @retval None
  */
void RC5_Encode_SignalGenerate(void)
{
  if ((RC5SendOpReadyFlag == 1) && (BitsSentCounter < RC5_MANCHESTER_BITS_LENGTH))
  {
    uint32_t bit_msg = (RC5ManchesterFrameFormat >> (RC5_MANCHESTER_BITS_LENGTH - 1U - BitsSentCounter)) & 1U;
    RC5SendOpCompleteFlag = 0;

    /* Gate the 38kHz carrier: 1 => PWM, 0 => forced low */
    RC5_Carrier_Enable((uint8_t)bit_msg);

    BitsSentCounter++;
    return;
  }

  /* Transmission complete */
  RC5SendOpCompleteFlag = 1;
  HAL_TIM_Base_Stop_IT(&htim15);
  RC5SendOpReadyFlag = 0;
  BitsSentCounter = 0;
  RC5_Carrier_Enable(0);
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
static uint32_t RC5_ManchesterConvert(uint16_t rc5BinaryFrame)
{
  uint32_t convertedMsg = 0;

  /* MSB-first: Start bits first on the wire */
  for (int8_t i = (int8_t)(RC5_REAL_FRAME_LENGTH - 1); i >= 0; i--)
  {
    uint16_t bit = (uint16_t)((rc5BinaryFrame >> i) & 1U);
    convertedMsg <<= 2;
    convertedMsg |= (bit ? RC5HIGHSTATE : RC5LOWSTATE);
  }

  return convertedMsg;
}

/**
  * @brief  Force TIM16 output compare state
  * @param  action: TIM_FORCED_ACTIVE or TIM_FORCED_INACTIVE
  * @retval None
  */
/* TIM_ForcedOC1Config removed: gating is done via RC5_Carrier_Enable() */
