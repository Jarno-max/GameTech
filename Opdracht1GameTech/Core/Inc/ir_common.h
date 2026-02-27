/**
  ******************************************************************************
  * @file    ir_common.h
  * @author  Adapted for STM32L432KC
  * @brief   IR Remote Common definitions for L432KC
  ******************************************************************************
  */

#ifndef __IR_COMMON_H
#define __IR_COMMON_H

#include "main.h"
#include "rc5_encode.h"

/* Timer configuration for L432KC @ 32MHz */
/* TIM16: 38kHz carrier wave
 * Calculation: 32MHz / (Prescaler+1) / (Period+1) = 38kHz
 * With Prescaler=0: 32MHz / 842 = 38.005 kHz 
 */
#define IR_ENC_HPERIOD_RC5      ((uint32_t)842)   /* 38kHz Carrier period */

/* TIM15: 889us bit timing (RC5 half-bit period)
 * Calculation: 32MHz / (Prescaler+1) / (Period+1) = 1125 Hz (889us period)
 * With Prescaler=31: 32MHz / 32 = 1MHz
 * With Period=889: 1MHz / 889 = 1124.86 Hz ≈ 889us
 */
#define IR_ENC_LPERIOD_RC5      ((uint32_t)889)   /* 889us bit half-period */

/* External variables */
extern uint8_t BitsSentCounter;

#endif /* __IR_COMMON_H */
