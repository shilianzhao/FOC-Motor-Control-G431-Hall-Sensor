
/**
  ******************************************************************************
  * @file    mc_parameters.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides definitions of HW parameters specific to the
  *          configuration of the subsystem.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
//cstat -MISRAC2012-Rule-21.1
#include "main.h" //cstat !MISRAC2012-Rule-21.1
//cstat +MISRAC2012-Rule-21.1
#include "parameters_conversion.h"

#include "r3_2_g4xx_pwm_curr_fdbk.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

#define FREQ_RATIO 1                /* Dummy value for single drive */
#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

/**
  * @brief  Current sensor parameters Motor 1 - three shunt - G4
  */
R3_2_Params_t R3_2_ParamsM1 =
{
/* Dual MC parameters --------------------------------------------------------*/
  .FreqRatio       = FREQ_RATIO,
  .IsHigherFreqTim = FREQ_RELATION,

/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx_1 = ADC1,
  .ADCx_2 = ADC2,
  /* Motor Control Kit config */
   .ADCConfig1 = { ( uint32_t )( 14<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( 2<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( 2<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( 2<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( 2<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( 14<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 },
   .ADCConfig2 = { ( uint32_t )( 4<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( 4<< ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( 4<< ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( 14<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( 14<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 , ( uint32_t )( 4<<ADC_JSQR_JSQ1_Pos ) | (LL_ADC_INJ_TRIG_EXT_TIM1_TRGO & ~ADC_INJ_TRIG_EXT_EDGE_DEFAULT)
                 },

   .ADCDataReg1 = { ADC1
                 , ADC1
                 , ADC1
                 , ADC1
                 , ADC1
                 , ADC1
                 },
  .ADCDataReg2 =  { ADC2
                 , ADC2
                 , ADC2
                 , ADC2
                 , ADC2
                 , ADC2
                  },
 //cstat +MISRAC2012-Rule-12.1 +MISRAC2012-Rule-10.1_R6

  /* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .Tafter            = TW_AFTER,
  .Tbefore           = TW_BEFORE,
  .Tsampling         = (uint16_t)SAMPLING_TIME,
  .Tcase2            = (uint16_t)SAMPLING_TIME + (uint16_t)TDEAD + (uint16_t)TRISE,
  .Tcase3            = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)SAMPLING_TIME)/2u,
  .TIMx               = TIM1,

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,
 .pwm_en_u_port      = M1_PWM_EN_U_GPIO_Port,
 .pwm_en_u_pin       = M1_PWM_EN_U_Pin,
 .pwm_en_v_port      = M1_PWM_EN_V_GPIO_Port,
 .pwm_en_v_pin       = M1_PWM_EN_V_Pin,
 .pwm_en_w_port      = M1_PWM_EN_W_GPIO_Port,
 .pwm_en_w_pin       = M1_PWM_EN_W_Pin,

/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .BKIN2Mode     = NONE,

/* Internal OPAMP common settings --------------------------------------------*/
  .OPAMPParams     = MC_NULL,
/* Internal COMP settings ----------------------------------------------------*/
  .CompOCPASelection     = MC_NULL,
  .CompOCPAInvInput_MODE = NONE,
  .CompOCPBSelection     = MC_NULL,
  .CompOCPBInvInput_MODE = NONE,
  .CompOCPCSelection     = MC_NULL,
  .CompOCPCInvInput_MODE = NONE,
  .DAC_OCP_ASelection    = MC_NULL,
  .DAC_OCP_BSelection    = MC_NULL,
  .DAC_OCP_CSelection    = MC_NULL,
  .DAC_Channel_OCPA      = (uint32_t) 0,
  .DAC_Channel_OCPB      = (uint32_t) 0,
  .DAC_Channel_OCPC      = (uint32_t) 0,

  .CompOVPSelection      = MC_NULL,
  .CompOVPInvInput_MODE  = NONE,
  .DAC_OVP_Selection     = MC_NULL,
  .DAC_Channel_OVP       = (uint32_t) 0,

/* DAC settings --------------------------------------------------------------*/
  .DAC_OCP_Threshold =  0,
  .DAC_OVP_Threshold =  23830,

};

/* USER CODE BEGIN Additional parameters */

/* USER CODE END Additional parameters */

/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/

