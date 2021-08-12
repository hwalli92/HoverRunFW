#pragma once
#include "at32f4xx.h"

// ############################### DO-NOT-TOUCH SETTINGS ###############################

#define PWM_FREQ 16000 // PWM frequency in Hz
#define DEAD_TIME 32   // PWM deadtime

#define DELAY_IN_MAIN_LOOP 5 // in ms. default 5. it is independent of all the timing critical stuff. do not touch if you do not know what you are doing.

#define TIMEOUT 5 // number of wrong / missing input commands before emergency off

// ############################### GENERAL ###############################

// How to calibrate: connect GND and RX of a 3.3v uart-usb adapter to the right sensor board cable (be careful not to use the red wire of the cable. 15v will destroye verything.). if you are using nunchuck, disable it temporarily. enable DEBUG_SERIAL_USART3 and DEBUG_SERIAL_ASCII use asearial terminal.

// Battery voltage calibration: connect power source. see <How to calibrate>. write value nr 5 to BAT_CALIB_ADC. make and flash firmware. then you can verify voltage on value 6 (devide it by 100.0 to get calibrated voltage).
#define BAT_CALIB_REAL_VOLTAGE 36.59 // input voltage measured by multimeter
#define BAT_CALIB_ADC 1430           // adc-value measured by mainboard (value nr 5 on UART debug output)

#define BAT_NUMBER_OF_CELLS 9 // normal Hoverboard battery: 10s
#define BAT_LOW_LVL1_ENABLE 0 // to beep or not to beep, 1 or 0
#define BAT_LOW_LVL1 3.6      // gently beeps at this voltage level. [V/cell]
#define BAT_LOW_LVL2_ENABLE 1 // to beep or not to beep, 1 or 0
#define BAT_LOW_LVL2 3.5      // your battery is almost empty. Charge now! [V/cell]
#define BAT_LOW_DEAD 3.37     // undervoltage poweroff. (while not driving) [V/cell]

#define DC_CUR_LIMIT 15 // DC current limit in amps per motor. so 15 means it will draw 30A out of your battery. it does not disable motors, it is a soft current limit.

// Board overheat detection: the sensor is inside the STM/GD chip. it is very inaccurate without calibration (up to 45°C). so only enable this funcion after calibration! let your board cool down. see <How to calibrate>. get the real temp of the chip by thermo cam or another temp-sensor taped on top of the chip and write it to TEMP_CAL_LOW_DEG_C. write debug value 8 to TEMP_CAL_LOW_ADC. drive around to warm up the board. it should be at least 20°C warmer. repeat it for the HIGH-values. enable warning and/or poweroff and make and flash firmware.
#define TEMP_CAL_LOW_ADC 1655    // temperature 1: ADC value
#define TEMP_CAL_LOW_DEG_C 35.8  // temperature 1: measured temperature [°C]
#define TEMP_CAL_HIGH_ADC 1588   // temperature 2: ADC value
#define TEMP_CAL_HIGH_DEG_C 48.9 // temperature 2: measured temperature [°C]
#define TEMP_WARNING_ENABLE 0    // to beep or not to beep, 1 or 0, DO NOT ACTIVITE WITHOUT CALIBRATION!
#define TEMP_WARNING 60          // annoying fast beeps [°C]
#define TEMP_POWEROFF_ENABLE 0   // to poweroff or not to poweroff, 1 or 0, DO NOT ACTIVITE WITHOUT CALIBRATION!
#define TEMP_POWEROFF 65         // overheat poweroff. (while not driving) [°C]

// Quiet mode will, most of the time, flash the LED instead of using the
// buzzer. However the buzzer will always be used when in a dangerous
// situation like low battery or overheating.
//#define QUIET_MODE 0 // whether to flash LED instead of beeping

#define INACTIVITY_TIMEOUT 8 // minutes of not driving until poweroff. it is not very precise.

// ############################### INPUT ###############################

#define CONTROL_UART
#define CONTROL_UART_BAUD 115200 // control via uart from eg an Arduino or raspberry

// ############################### MOTOR CONTROL (overwrite) #########################
#define CTRL_TYP_SEL 3  // [-] Control method selection: 0 = Commutation , 1 = Pure Trapezoidal , 2 = Sinusoidal, 3 = Sinusoidal 3rd armonic (default)
#define PHASE_ADV_ENA 1 // [-] Phase advance enable parameter: 0 = disabled, 1 = enabled (default)

// GENERAL NOTES:
// 1. All the available motor parameters can be found in the BLDC_controller_data.c
// 2. For more details regarding the parameters and the working principle of the controller please consult the Simulink model
// 3. A webview was created, so Matlab/Simulink installation is not needed, unless you want to regenerate the code

// NOTES Phase Advance / Field weakening:
// 1. In BLDC_controller_data.c you can find the Phase advance Map as a function of Duty Cycle: MAP = a_phaAdv_M1, XAXIS = r_phaAdvDC_XA
// 2. The default calibration was experimentally calibrated on the real motor based on the minimum noise and minimum torque ripple
// 3. If you re-calibrate the Phase advance map please take all the safety measures!
// 4. I do not recommend more than 40 deg MAX Phase advance. The motors can spin VERY VERY FAST!!! Please use it with care!!

// ############################### MOTOR SETTINGS #################################
// 1-6 representing the 6 possible ways to map the hall sensors to the motors
// a wrong setting will result in a not or badly turning motor. Use the MOTOR_TEST
// control method to find the right setting for your board.
#define CONFIG_HALL_IDX_LEFT 0
#define CONFIG_HALL_IDX_RIGHT 4

// ############################### DRIVING BEHAVIOR ###############################

// inputs:
// - cmd1 and cmd2: analog normalized input values. -1000 to 1000
// - button1 and button2: digital input values. 0 or 1
// - adc_buffer.l_tx2 and adc_buffer.l_rx2: unfiltered ADC values (you do not need them). 0 to 4095
// outputs:
// - speedR and speedL: normal driving -1000 to 1000
// - weakr and weakl: field weakening for extra boost at high speed (speedR > 700 and speedL > 700). 0 to ~400

#define FILTER 0.1            // lower value == softer filter. do not use values <0.01, you will get float precision issues.
#define SPEED_COEFFICIENT 0.6 // higher value == stronger. 0.0 to ~2.0?
#define STEER_COEFFICIENT 0.5 // higher value == stronger. if you do not want any steering, set it to 0.0; 0.0 to 1.0
#define PID_COEFFICIENT 0.4
//#define INVERT_R_DIRECTION
//#define INVERT_L_DIRECTION
#define BEEPS_BACKWARD 0 // 0 or 1