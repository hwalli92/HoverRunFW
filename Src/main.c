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

#include "at32f4xx.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "uart.h"

// ###############################################################################
#include "BLDC_controller.h" /* Model's header file */
#include "rtwtypes.h"

RT_MODEL rtM_Left_;  /* Real-time model */
RT_MODEL rtM_Right_; /* Real-time model */
RT_MODEL *const rtM_Left = &rtM_Left_;
RT_MODEL *const rtM_Right = &rtM_Right_;

P rtP; /* Block parameters (auto storage) */

DW rtDW_Left;  /* Observable states */
ExtU rtU_Left; /* External inputs */
ExtY rtY_Left; /* External outputs */

DW rtDW_Right;  /* Observable states */
ExtU rtU_Right; /* External inputs */
ExtY rtY_Right; /* External outputs */
// ###############################################################################

void SystemClock_Config(void);

extern volatile adc_buf_t adc_buffer;

int cmd1; // normalized input values. -1000 to 1000
int cmd2;
int cmd3;

typedef struct
{
  int16_t steer;
  int16_t speed;
  double pidvalue;
  //uint32_t crc;
} Serialcommand;

volatile Serialcommand command;

uint8_t button1, button2;

int steer; // global variable for steering. -1000 to 1000
int speed; // global variable for speed. -1000 to 1000

float local_speed_coefficent;
float local_steer_coefficent;

extern volatile int pwml; // global variable for pwm left. -1000 to 1000
extern volatile int pwmr; // global variable for pwm right. -1000 to 1000
//extern volatile int weakl; // global variable for field weakening left. -1000 to 1000
//extern volatile int weakr; // global variable for field weakening right. -1000 to 1000

extern uint8_t buzzerFreq;    // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...

extern uint8_t enable; // global variable for motor enable

extern volatile uint32_t timeout; // global variable for timeout
extern float batteryVoltage;      // global variable for battery voltage

extern volatile uint32_t systick_counter;

uint32_t inactivity_timeout_counter;

extern volatile uint8_t hall_idx_left;
extern volatile uint8_t hall_idx_right;

int milli_vel_error_sum = 0;

uint32_t millis(void)
{
  return systick_counter;
}

void delay(uint32_t millis)
{
  uint32_t start = systick_counter;
  while ((systick_counter - start) < millis)
  {
    // do nothing
  }
}

void poweroff()
{
  if (ABS(speed) < 20)
  {
    buzzerPattern = 0;
    enable = 0;
    for (int i = 0; i < 8; i++)
    {
#ifdef QUIET_MODE
      GPIO_WriteBit(LLED_PORT, LLED_PIN, i & 1);
      GPIO_WriteBit(RLED_PORT, RLED_PIN, i & 1);
#else
      buzzerFreq = i;
#endif
      delay(100);
    }
    GPIO_WriteBit(OFF_PORT, OFF_PIN, 0);
    while (1)
    {
    }
  }
}

int main(void)
{
  hall_idx_left = CONFIG_HALL_IDX_LEFT;
  hall_idx_right = CONFIG_HALL_IDX_RIGHT;

  RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_AFIO, ENABLE);
  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  NVIC_SetPriority(MemoryManagement_IRQn, 0);
  /* BusFault_IRQn interrupt configuration */
  NVIC_SetPriority(BusFault_IRQn, 0);
  /* UsageFault_IRQn interrupt configuration */
  NVIC_SetPriority(UsageFault_IRQn, 0);
  /* SVCall_IRQn interrupt configuration */
  NVIC_SetPriority(SVCall_IRQn, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  NVIC_SetPriority(DebugMonitor_IRQn, 0);
  /* PendSV_IRQn interrupt configuration */
  NVIC_SetPriority(PendSV_IRQn, 0);
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 0);

  SystemClock_Config();
  RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_DMA1, DISABLE);

  MX_GPIO_Init();
  MX_TIM_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();

  uart_initialize();

  GPIO_WriteBit(OFF_PORT, OFF_PIN, 1);

  ADC_ExternalTrigConvCtrl(ADC1, ENABLE);
  ADC_SoftwareStartConvCtrl(ADC2, ENABLE);

  // ###############################################################################

  /* Set BLDC controller parameters */
  rtP.z_ctrlTypSel = CTRL_TYP_SEL;
  rtP.b_phaAdvEna = PHASE_ADV_ENA;

  /* Pack LEFT motor data into RTM */
  rtM_Left->defaultParam = &rtP;
  rtM_Left->dwork = &rtDW_Left;
  rtM_Left->inputs = &rtU_Left;
  rtM_Left->outputs = &rtY_Left;

  /* Pack RIGHT motor data into RTM */
  rtM_Right->defaultParam = &rtP;
  rtM_Right->dwork = &rtDW_Right;
  rtM_Right->inputs = &rtU_Right;
  rtM_Right->outputs = &rtY_Right;

  /* Initialize BLDC controllers */
  BLDC_controller_initialize(rtM_Left);
  BLDC_controller_initialize(rtM_Right);

  for (int i = 8; i >= 0; i--)
  {
#ifdef QUIET_MODE
    GPIO_WriteBit(LLED_PORT, LLED_PIN, i & 1);
    GPIO_WriteBit(RLED_PORT, RLED_PIN, i & 1);
#else
    buzzerFreq = i;
#endif
    delay(100);
  }
  buzzerFreq = 0;

  GPIO_WriteBit(LLED_PORT, LLED_PIN, 1);
  GPIO_WriteBit(RLED_PORT, RLED_PIN, 1);

  int speedL = 0, speedR = 0;
  int lastSpeedL = 0, lastSpeedR = 0;
  local_speed_coefficent = SPEED_COEFFICIENT;
  local_steer_coefficent = STEER_COEFFICIENT;
  float board_temp_adc_filtered = (float)adc_buffer.temp;
  float board_temp_deg_c;

  enable = 1; // enable motors

  while (1)
  {
    delay(DELAY_IN_MAIN_LOOP); //delay in ms

    uart_handle_command();

    cmd1 = CLAMP((int16_t)command.steer, -1000, 1000);
    cmd2 = CLAMP((int16_t)command.speed, -1000, 1000);

    // ####### LOW-PASS FILTER #######
    steer = steer * (1.0 - FILTER) + cmd1 * FILTER;
    speed = speed * (1.0 - FILTER) + cmd2 * FILTER;

    // ####### MIXER #######
    speedR = CLAMP(speed * local_speed_coefficent + steer * local_steer_coefficent, -1000, 1000);
    speedL = CLAMP(speed * local_speed_coefficent - steer * local_steer_coefficent, -1000, 1000);

    // ####### SET OUTPUTS #######
    if ((speedL < lastSpeedL + 50 && speedL > lastSpeedL - 50) && (speedR < lastSpeedR + 50 && speedR > lastSpeedR - 50))
    {
      pwmr = speedR;
      pwml = -speedL;
    }

    lastSpeedL = speedL;
    lastSpeedR = speedR;

    if (inactivity_timeout_counter % 25 == 0)
    {
      // ####### CALC BOARD TEMPERATURE #######
      board_temp_adc_filtered = board_temp_adc_filtered * 0.99 + (float)adc_buffer.temp * 0.01;
      board_temp_deg_c = ((float)TEMP_CAL_HIGH_DEG_C - (float)TEMP_CAL_LOW_DEG_C) / ((float)TEMP_CAL_HIGH_ADC - (float)TEMP_CAL_LOW_ADC) * (board_temp_adc_filtered - (float)TEMP_CAL_LOW_ADC) + (float)TEMP_CAL_LOW_DEG_C;

      // ####### DEBUG SERIAL OUT #######
      setScopeChannel(0, (int)speedR);                   // 0: output speed: 0-1000
      setScopeChannel(1, (int)speedL);                   // 1: output speed: 0-1000
      setScopeChannel(2, (int)steer);                    // 2: steer value: 0-1000
      setScopeChannel(3, (int)batteryVoltage);           // 3: battery voltage
      setScopeChannel(4, (int)adc_buffer.batt1);         // 4: for battery voltage calibration
      setScopeChannel(5, (int)(command.pidvalue * 100)); // 5: for verifying battery voltage calibration
      setScopeChannel(6, (int)rtU_Left.b_hallB);         // 6: for board temperature calibration
      setScopeChannel(7, (int)rtU_Left.b_hallC);         // 7: for verifying board temperature calibration
      setScopeChannel(8, (int)rtU_Right.b_hallA);
      setScopeChannel(9, (int)rtU_Right.b_hallB);
      setScopeChannel(10, (int)rtU_Right.b_hallC);
    }

    // ####### POWEROFF BY POWER-BUTTON #######
    if (GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_PIN))
    {
      enable = 0;
      while (GPIO_ReadInputDataBit(BUTTON_PORT, BUTTON_PIN))
      {
      }
      poweroff();
    }

    // ####### BEEP AND EMERGENCY POWEROFF #######
    if ((TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && ABS(speed) < 20) || (batteryVoltage < ((float)BAT_LOW_DEAD * (float)BAT_NUMBER_OF_CELLS) && ABS(speed) < 20))
    { // poweroff before mainboard burns OR low bat 3
      poweroff();
    }
    else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING)
    { // beep if mainboard gets hot
      buzzerFreq = 4;
      buzzerPattern = 1;
    }
    else if (batteryVoltage < ((float)BAT_LOW_LVL1 * (float)BAT_NUMBER_OF_CELLS) && batteryVoltage > ((float)BAT_LOW_LVL2 * (float)BAT_NUMBER_OF_CELLS) && BAT_LOW_LVL1_ENABLE)
    { // low bat 1: slow beep
      buzzerFreq = 5;
      buzzerPattern = 42;
    }
    else if (batteryVoltage < ((float)BAT_LOW_LVL2 * (float)BAT_NUMBER_OF_CELLS) && batteryVoltage > ((float)BAT_LOW_DEAD * (float)BAT_NUMBER_OF_CELLS) && BAT_LOW_LVL2_ENABLE)
    { // low bat 2: fast beep
      buzzerFreq = 5;
      buzzerPattern = 6;
    }
    else if (BEEPS_BACKWARD && speed < -50)
    { // backward beep
      buzzerFreq = 5;
      buzzerPattern = 1;
    }
    else
    { // do not beep
      buzzerFreq = 0;
      buzzerPattern = 0;
    }

    // ####### INACTIVITY TIMEOUT #######
    if (ABS(speedL) > 50 || ABS(speedR) > 50)
    {
      inactivity_timeout_counter = 0;
    }
    else
    {
      inactivity_timeout_counter++;
    }
    if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60 * 1000) / (DELAY_IN_MAIN_LOOP + 1))
    { // rest of main loop needs maybe 1ms
      poweroff();
    }
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{
  SysTick_Config(SystemCoreClock / 1000); // tick every 1 millisecond
  NVIC_SetPriority(SysTick_IRQn, 0);
}

void set_steer(int w)
{
  command.steer = w;
}

void set_speed(int v)
{
  command.speed = v;
}

void update_timeout()
{
  inactivity_timeout_counter = 0;
}

void set_pidvalue(double pidvalue)
{
  command.pidvalue = pidvalue;
}