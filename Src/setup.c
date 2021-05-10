/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
tim1 master, enable -> trgo
tim8, gated slave mode, trgo by tim1 trgo. overflow -> trgo
adc1,adc2 triggered by tim8 trgo
adc 1,2 dual mode

ADC1             ADC2
R_Blau PC4 CH14  R_Gelb PC5 CH15
L_Grün PA0 CH01  L_Blau PC3 CH13
R_DC PC1 CH11    L_DC PC0 CH10
BAT   PC2 CH12   L_TX PA2 CH02
BAT   PC2 CH12   L_RX PA3 CH03

pb10 usart3 dma1 channel2/3
*/

#include <stdint.h>
#include "defines.h"
#include "config.h"
#include "at32f4xx.h"

volatile adc_buf_t adc_buffer;

void MX_GPIO_Init(void)
{
  GPIO_InitType GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOC, ENABLE);

  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStruct.GPIO_MaxSpeed = GPIO_MaxSpeed_10MHz;

  GPIO_InitStruct.GPIO_Pins = LEFT_HALL_U_PIN;
  GPIO_Init(LEFT_HALL_U_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = LEFT_HALL_V_PIN;
  GPIO_Init(LEFT_HALL_V_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = LEFT_HALL_W_PIN;
  GPIO_Init(LEFT_HALL_W_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = RIGHT_HALL_U_PIN;
  GPIO_Init(RIGHT_HALL_U_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = RIGHT_HALL_V_PIN;
  GPIO_Init(RIGHT_HALL_V_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = RIGHT_HALL_W_PIN;
  GPIO_Init(RIGHT_HALL_W_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = CHARGER_PIN;
  GPIO_Init(CHARGER_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = BUTTON_PIN;
  GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT_PP;

  GPIO_InitStruct.GPIO_Pins = BUZZER_PIN;
  GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = OFF_PIN;
  GPIO_Init(OFF_PORT, &GPIO_InitStruct);

  // LED Out
  GPIO_InitStruct.GPIO_Pins = LLED_PIN;
  GPIO_Init(LLED_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = RLED_PIN;
  GPIO_Init(RLED_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_ANALOG;

  GPIO_InitStruct.GPIO_Pins = LEFT_DC_CUR_PIN;
  GPIO_Init(LEFT_DC_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = LEFT_U_CUR_PIN;
  GPIO_Init(LEFT_U_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = LEFT_V_CUR_PIN;
  GPIO_Init(LEFT_V_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = RIGHT_DC_CUR_PIN;
  GPIO_Init(RIGHT_DC_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = RIGHT_U_CUR_PIN;
  GPIO_Init(RIGHT_U_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = RIGHT_V_CUR_PIN;
  GPIO_Init(RIGHT_V_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = DCLINK_PIN;
  GPIO_Init(DCLINK_PORT, &GPIO_InitStruct);

  //Analog in
  GPIO_InitStruct.GPIO_Pins = GPIO_Pins_3;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pins = GPIO_Pins_2;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;

  GPIO_InitStruct.GPIO_Pins = LEFT_TIM_UH_PIN;
  GPIO_Init(LEFT_TIM_UH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = LEFT_TIM_VH_PIN;
  GPIO_Init(LEFT_TIM_VH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = LEFT_TIM_WH_PIN;
  GPIO_Init(LEFT_TIM_WH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = LEFT_TIM_UL_PIN;
  GPIO_Init(LEFT_TIM_UL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = LEFT_TIM_VL_PIN;
  GPIO_Init(LEFT_TIM_VL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = LEFT_TIM_WL_PIN;
  GPIO_Init(LEFT_TIM_WL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = RIGHT_TIM_UH_PIN;
  GPIO_Init(RIGHT_TIM_UH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = RIGHT_TIM_VH_PIN;
  GPIO_Init(RIGHT_TIM_VH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = RIGHT_TIM_WH_PIN;
  GPIO_Init(RIGHT_TIM_WH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = RIGHT_TIM_UL_PIN;
  GPIO_Init(RIGHT_TIM_UL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = RIGHT_TIM_VL_PIN;
  GPIO_Init(RIGHT_TIM_VL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pins = RIGHT_TIM_WL_PIN;
  GPIO_Init(RIGHT_TIM_WL_PORT, &GPIO_InitStruct);
}

void MX_TIM_Init(void)
{
  TMR_TimerBaseInitType TMR_TimeBaseInitStruct = {0};
  TMR_OCInitType TMR_OCInitStruct = {0};
  TMR_BRKDTInitType TMR_BRKDTInitStruct = {0};

  RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_TMR1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_TMR8, ENABLE);

  TMR_TimeBaseStructInit(&TMR_TimeBaseInitStruct);
  TMR_TimeBaseInitStruct.TMR_DIV = 0;
  TMR_TimeBaseInitStruct.TMR_CounterMode = TMR_CounterDIR_CenterAligned1;
  TMR_TimeBaseInitStruct.TMR_Period = SystemCoreClock / 2 / PWM_FREQ;
  TMR_TimeBaseInitStruct.TMR_ClockDivision = TMR_CKD_DIV1;
  TMR_TimeBaseInitStruct.TMR_RepetitionCounter = 0;
  TMR_TimeBaseInit(RIGHT_TIM, &TMR_TimeBaseInitStruct);
  TMR_ARPreloadConfig(RIGHT_TIM, DISABLE);

  TMR_SelectOutputTrigger(RIGHT_TIM, TMR_TRGOSource_Enable);
  TMR_SelectMasterSlaveMode(RIGHT_TIM, TMR_MasterSlaveMode_Disable);

  TMR_OCInitStruct.TMR_OCMode = TMR_OCMode_PWM1;
  TMR_OCInitStruct.TMR_Pulse = 0;
  TMR_OCInitStruct.TMR_OCPolarity = TMR_OCPolarity_High;
  TMR_OCInitStruct.TMR_OCNPolarity = TMR_OCNPolarity_Low;
  TMR_OCInitStruct.TMR_OCIdleState = TMR_OCIdleState_Reset;
  TMR_OCInitStruct.TMR_OCNIdleState = TMR_OCNIdleState_Set;
  TMR_OCInitStruct.TMR_OutputState = TMR_OutputState_Disable;
  TMR_OCInitStruct.TMR_OutputNState = TMR_OutputNState_Disable;
  TMR_OC1Init(RIGHT_TIM, &TMR_OCInitStruct);
  TMR_OC2Init(RIGHT_TIM, &TMR_OCInitStruct);
  TMR_OC3Init(RIGHT_TIM, &TMR_OCInitStruct);
  TMR_OC1PreloadConfig(RIGHT_TIM, TMR_OCPreload_Enable);
  TMR_OC2PreloadConfig(RIGHT_TIM, TMR_OCPreload_Enable);
  TMR_OC3PreloadConfig(RIGHT_TIM, TMR_OCPreload_Enable);
  TMR_OC1FastConfig(RIGHT_TIM, TMR_OCFast_Disable);
  TMR_OC2FastConfig(RIGHT_TIM, TMR_OCFast_Disable);
  TMR_OC3FastConfig(RIGHT_TIM, TMR_OCFast_Disable);

  TMR_BRKDTInitStruct.TMR_OSIMRState = TMR_OSIMRState_Enable;
  TMR_BRKDTInitStruct.TMR_OSIMIState = TMR_OSIMIState_Enable;
  TMR_BRKDTInitStruct.TMR_LOCKgrade = TMR_LOCKgrade_OFF;
  TMR_BRKDTInitStruct.TMR_DeadTime = DEAD_TIME;
  TMR_BRKDTInitStruct.TMR_Break = TMR_Break_Disable;
  TMR_BRKDTInitStruct.TMR_BreakPolarity = TMR_BreakPolarity_Low;
  TMR_BRKDTInitStruct.TMR_AutomaticOutput = TMR_AutomaticOutput_Disable;
  TMR_BRKDTConfig(RIGHT_TIM, &TMR_BRKDTInitStruct);

  TMR_TimeBaseStructInit(&TMR_TimeBaseInitStruct);
  TMR_TimeBaseInitStruct.TMR_DIV = 0;
  TMR_TimeBaseInitStruct.TMR_CounterMode = TMR_CounterDIR_CenterAligned1;
  TMR_TimeBaseInitStruct.TMR_Period = SystemCoreClock / 2 / PWM_FREQ;
  TMR_TimeBaseInitStruct.TMR_ClockDivision = TMR_CKD_DIV1;
  TMR_TimeBaseInitStruct.TMR_RepetitionCounter = 0;
  TMR_TimeBaseInit(LEFT_TIM, &TMR_TimeBaseInitStruct);
  TMR_ARPreloadConfig(LEFT_TIM, DISABLE);

  TMR_SelectOutputTrigger(LEFT_TIM, TMR_TRGOSource_Update);
  TMR_SelectMasterSlaveMode(LEFT_TIM, TMR_MasterSlaveMode_Enable);

  TMR_SelectInputTrigger(LEFT_TIM, TMR_TRGSEL_ITR0);
  TMR_SelectSlaveMode(LEFT_TIM, TMR_SlaveMode_Gate);

  TMR_OCInitStruct.TMR_OCMode = TMR_OCMode_PWM1;
  TMR_OCInitStruct.TMR_Pulse = 0;
  TMR_OCInitStruct.TMR_OCPolarity = TMR_OCPolarity_High;
  TMR_OCInitStruct.TMR_OCNPolarity = TMR_OCNPolarity_Low;
  TMR_OCInitStruct.TMR_OCIdleState = TMR_OCIdleState_Reset;
  TMR_OCInitStruct.TMR_OCNIdleState = TMR_OCNIdleState_Set;
  TMR_OCInitStruct.TMR_OutputState = TMR_OutputState_Disable;
  TMR_OCInitStruct.TMR_OutputNState = TMR_OutputNState_Disable;
  TMR_OC1Init(LEFT_TIM, &TMR_OCInitStruct);
  TMR_OC2Init(LEFT_TIM, &TMR_OCInitStruct);
  TMR_OC3Init(LEFT_TIM, &TMR_OCInitStruct);
  TMR_OC1PreloadConfig(LEFT_TIM, TMR_OCPreload_Enable);
  TMR_OC2PreloadConfig(LEFT_TIM, TMR_OCPreload_Enable);
  TMR_OC3PreloadConfig(LEFT_TIM, TMR_OCPreload_Enable);
  TMR_OC1FastConfig(LEFT_TIM, TMR_OCFast_Disable);
  TMR_OC2FastConfig(LEFT_TIM, TMR_OCFast_Disable);
  TMR_OC3FastConfig(LEFT_TIM, TMR_OCFast_Disable);

  TMR_BRKDTInitStruct.TMR_OSIMRState = TMR_OSIMRState_Enable;
  TMR_BRKDTInitStruct.TMR_OSIMIState = TMR_OSIMIState_Enable;
  TMR_BRKDTInitStruct.TMR_LOCKgrade = TMR_LOCKgrade_OFF;
  TMR_BRKDTInitStruct.TMR_DeadTime = DEAD_TIME;
  TMR_BRKDTInitStruct.TMR_Break = TMR_Break_Disable;
  TMR_BRKDTInitStruct.TMR_BreakPolarity = TMR_BreakPolarity_Low;
  TMR_BRKDTInitStruct.TMR_AutomaticOutput = TMR_AutomaticOutput_Disable;
  TMR_BRKDTConfig(LEFT_TIM, &TMR_BRKDTInitStruct);

  TMR_CtrlPWMOutputs(LEFT_TIM, DISABLE);
  TMR_CtrlPWMOutputs(RIGHT_TIM, DISABLE);

  TMR_CCxCmd(LEFT_TIM, TMR_Channel_1, TMR_CCx_Enable);
  TMR_CCxCmd(LEFT_TIM, TMR_Channel_2, TMR_CCx_Enable);
  TMR_CCxCmd(LEFT_TIM, TMR_Channel_3, TMR_CCx_Enable);
  TMR_CCxNCmd(LEFT_TIM, TMR_Channel_1, TMR_CCxN_Enable);
  TMR_CCxNCmd(LEFT_TIM, TMR_Channel_2, TMR_CCxN_Enable);
  TMR_CCxNCmd(LEFT_TIM, TMR_Channel_3, TMR_CCxN_Enable);
  TMR_CtrlPWMOutputs(LEFT_TIM, ENABLE);
  TMR_Cmd(LEFT_TIM, ENABLE);

  TMR_CCxCmd(RIGHT_TIM, TMR_Channel_1, TMR_CCx_Enable);
  TMR_CCxCmd(RIGHT_TIM, TMR_Channel_2, TMR_CCx_Enable);
  TMR_CCxCmd(RIGHT_TIM, TMR_Channel_3, TMR_CCx_Enable);
  TMR_CCxNCmd(RIGHT_TIM, TMR_Channel_1, TMR_CCxN_Enable);
  TMR_CCxNCmd(RIGHT_TIM, TMR_Channel_2, TMR_CCxN_Enable);
  TMR_CCxNCmd(RIGHT_TIM, TMR_Channel_3, TMR_CCxN_Enable);
  TMR_CtrlPWMOutputs(RIGHT_TIM, ENABLE);
  TMR_Cmd(RIGHT_TIM, ENABLE);

  LEFT_TIM->RC = 1;
}

void MX_ADC1_Init(void)
{
  ADC_InitType ADC_InitStructure = {0};
  DMA_InitType DMA_InitStructure = {0};

  RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_ADC1, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_DMA1, ENABLE);

  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
  ADC_InitStructure.ADC_ScanMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrig = ADC_ExternalTrig_Ext_INT11_TMR8_TRGO_ADC12;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NumOfChannel = 5;
  ADC_Init(ADC1, &ADC_InitStructure);
  ADC_ExternalTrigConvCtrl(ADC1, DISABLE);
  ADC_DiscModeCtrl(ADC1, DISABLE);

  /**Enable or disable the remapping of ADC1_ETRGREG:
    * ADC1 External Event regular conversion is connected to TIM8 TRG0
    */
  AFIO->MAP |= AFIO_MAP_ADC1_EXTRGREG_REMAP;

  ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_7_5);  // pc4 left b
  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 2, ADC_SampleTime_7_5);   // pa0 right a
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 3, ADC_SampleTime_13_5); // pc1 left cur
  ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 4, ADC_SampleTime_13_5); // pc2 vbat
  //temperature requires at least 17.1uS sampling time
  ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 5, ADC_SampleTime_239_5); // internal temp

  ADC_TempSensorVrefintCtrl(ENABLE);
  ADC_DMACtrl(ADC1, ENABLE);

  ADC_Ctrl(ADC1, ENABLE);

  //calibrate ADC
  ADC_RstCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while (ADC_GetResetCalibrationStatus(ADC1))
    ;
  /* Start ADC1 calibration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while (ADC_GetCalibrationStatus(ADC1))
    ;

  DMA_Reset(DMA1_Channel1);
  DMA_DefaultInitParaConfig(&DMA_InitStructure);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (ADC1->RDOR);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&adc_buffer;
  DMA_InitStructure.DMA_Direction = DMA_DIR_PERIPHERALSRC;
  DMA_InitStructure.DMA_BufferSize = 5;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
  DMA_InitStructure.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
  DMA_InitStructure.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_WORD;
  DMA_InitStructure.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_WORD;
  DMA_InitStructure.DMA_Mode = DMA_MODE_CIRCULAR;
  DMA_InitStructure.DMA_Priority = DMA_PRIORITY_HIGH;
  DMA_InitStructure.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  DMA_INTConfig(DMA1_Channel1, DMA_INT_TC, ENABLE);
  DMA_ChannelEnable(DMA1_Channel1, ENABLE);

  NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/* ADC2 init function */
void MX_ADC2_Init(void)
{
  ADC_InitType ADC_InitStructure = {0};

  RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_ADC2, ENABLE);

  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
  ADC_InitStructure.ADC_ScanMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrig = ADC_ExternalTrig_TMR1_CC1_ADC12; // have to set it to something
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NumOfChannel = 5;
  ADC_Init(ADC2, &ADC_InitStructure);
  ADC_ExternalTrigConvCtrl(ADC2, DISABLE);
  ADC_DiscModeCtrl(ADC2, DISABLE);

  ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 1, ADC_SampleTime_7_5);  // pc5 left c
  ADC_RegularChannelConfig(ADC2, ADC_Channel_13, 2, ADC_SampleTime_7_5);  // pc3 right b
  ADC_RegularChannelConfig(ADC2, ADC_Channel_10, 3, ADC_SampleTime_13_5); // pc0 right cur
  ADC_RegularChannelConfig(ADC2, ADC_Channel_2, 4, ADC_SampleTime_13_5);  // pa2 uart-l-tx
  ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 5, ADC_SampleTime_239_5); // pa3 uart-l-rx

  ADC_DMACtrl(ADC2, ENABLE);
  ADC_Ctrl(ADC2, ENABLE);
}
