/**
  ******************************************************************************
  * @file    rc5_encode.h
  * @author  MCD Application Team (adapted for L432KC)
  * @brief   RC5 encoder header file
  ******************************************************************************
  */

#ifndef __RC5_ENCODE_H
#define __RC5_ENCODE_H

#include <stdint.h>

/**
  * @brief RC5 Control bit definition
  */
typedef enum
{
  RC5_CTRL_RESET = ((uint16_t)0),
  RC5_CTRL_SET   = ((uint16_t)0x0800)
} RC5_Ctrl_t;

/* Function prototypes */
void RC5_Encode_Init(void);
void RC5_Encode_SendFrame(uint8_t RC5_Address, uint8_t RC5_Instruction, RC5_Ctrl_t RC5_Ctrl);
void RC5_Encode_SignalGenerate(void);

#endif /* __RC5_ENCODE_H */
