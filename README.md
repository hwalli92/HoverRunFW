# hoverboard-firmware-hack

This repo is a fork of Niklas Fauths Hoverboard Hack Firmware for some of the newer (V5.1) mainboards with the AT32F413RCT7 controller.

---

## Build Instructions

The code was compiled with GNU arm-none-eabi-gcc 7.3.1 under windows and should be compilable with other versions of the arm-gcc.

To build, clone this repository and run make:

    git clone https://github.com/someone42/hoverboard-firmware-hack.git
    make

Only boards with the AT32F413RCT7 controller are supported. Use https://github.com/cloidnerux/hoverboard-firmware-hack if you need support for boards that use the AT32F403RCT6 or STM32F103 controller.

---

## Hardware
![otter](https://raw.githubusercontent.com/cloidnerux/hoverboard-firmware-hack/master/pinout.png)

The above image is a V4 board, however, the V5.1 looks very similar. The main difference of the V5.1 board compared to V4 is the use of a cheaper controller and also
(somehow) even less capacitors. All the pinouts for the connectors on the V5.1 mainboard are identical to the V4 mainboard.

These boards see a lot of small changes over the years to make them cheaper or to implement certain functions. Your board
might not be quite the same as the board shown here.

The reverse-engineered schematics of an older version of the mainboard can be found here:
http://vocke.tv/lib/exe/fetch.php?media=20150722_hoverboard_sch.pdf

---

## Flashing
To build the firmware, just type "make". Make sure you have specified your gcc-arm-none-eabi binary location in the Makefile ("PREFIX = ...") (version 7 works, there is a version that does not!) (if the ons in linux repos do not work, use the official version: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads). Right to the STM32, there is a debugging header with 3V3, SWCLK, SWDIO and GND (pin 1). Connect GND, SWDIO and SWCLK to your SWD programmer, like the ST-Link found on many STM devboards.

Do not power the mainboard from the 3.3V of your programmer! This has already killed multiple mainboards.

Make sure you hold the powerbutton or connect a jumper to the power button pins while flashing the firmware, as the controller might release the power latch and switches itself off during flashing. Battery > 36V have to be connected while flashing.

I flashed my AT32 with a ST-Link using the SWD connector.

If you never flashed your mainboard before, the STM is probably locked. To unlock the flash, use the following OpenOCD command:
```
openocd -f interface/stlink.cfg -f target/stm32f3x.cfg -c init -c "reset halt" -c "stm32f2x unlock 0"
```

To program, use the following OpenOCD command:
```
openocd -f interface/stlink.cfg -f target/stm32f3x.cfg -c init -c "reset halt" -c "flash write_image erase build/hover.hex 0 ihex"
```

If using OpenOCD, you will probably encounter a problem with it not recognizing the AT32 as a STM32 ("Cannot identify target as a STM32 family"). You will need to modify and compile OpenOCD, like in https://github.com/bipropellant/bipropellant-hoverboard-firmware/issues/67#issuecomment-516657520, however:
- Use 0x240 instead of 0x242
- Use "max_flash_size_in_kb = 256;" instead of "max_flash_size_in_kb = 512;"
- You might was well modify the debug message to say ATF32F413 instead of ATF32F403

## Troubleshooting
First, check that power is connected and voltage is >36V while flashing.
If the board draws more than 100mA in idle, it's probably broken.

If the motors do something, but don't rotate smooth and quietly, try to use an alternative phase mapping. Usually, color-correct mapping (blue to blue, green to green, yellow to yellow) works fine. However, some hoverboards have a different layout then others, and this might be the reason your motor isn't spinning.

Nunchuck not working: Use the right one of the 2 types of nunchucks. Use i2c pullups.

Nunchuck or PPM working bad: The i2c bus and PPM signal are very sensitive to emv distortions of the motor controller. They get stronger the faster you are. Keep cables short, use shielded cable, use ferrits, stabilize voltage in nunchuck or reviever, add i2c pullups. To many errors leads to very high accelerations which triggers the protection board within the battery to shut everything down.

Most robust way for input is to use the ADC and potis. It works well even on 1m unshielded cable. Solder ~100k Ohm resistors between ADC-inputs and gnd directly on the mainboard. Use potis as pullups to 3.3V.

ADC Joystick control uses two ADCs on pins PA_2 and PA_3. ADC1 is used for steering, values below `ADC1_CENTER` mean left. ADC2 is used for speed control, values above `ADC2_CENTER` mean forward and below `ADC2_CENTER` mean backward.

---

## Uart communication

You can connect to your hoverboard using UART to pins L_TX (PA2), L_RX (PA3) and GND on the left sensor board via RaspberryPI or Arduino and use some simple commands:
* ping (you should receive "pong" as a response)
* status (you should receive basic diagnostic info like speed of motors, battery status, and so on. Response format is like "1:0 2:0 3:54 4:54 5:1491 6:3815 7:1569 8:52")
* test (makes both motors to turn really slow)
* move [steer] [speed] (sets motor speed and steering)
* stop (stops both motors)
* poweroff (shuts the hoverboard down)
You should terminate commands with ';' or new line characters.

## Examples

Have a look at the config.h in the Inc directory. That's where you configure to firmware to match your project.
Currently supported: Wii Nunchuck, analog potentiometer and PPM-Sum signal from a RC remote.
A good example of control via UART, eg. from an Arduino or raspberryPi, can be found here:
https://github.com/p-h-a-i-l/hoverboard-firmware-hack

If you need additional features like a boost button, have a look at the while(1) loop in the main.c

### Additional Hardware

* [breakout/interconnect boards](https://github.com/Jan--Henrik/hoverboard-breakout)  Breakout/Interconnection boards for hoverboard hacking.

### Projects based on it
* [bobbycar-optimized firmware](https://github.com/larsmm/hoverboard-firmware-hack-bbcar)  based on this one with driving modes, acceleration ramps and some other features
* [wheel chair](https://github.com/Lahorde/steer_speed_ctrl) controlled with a joystick or using a CC2650 sensortag to control it over  bluetooth with pitch/roll.
* [TranspOtterNG](https://github.com/Jan--Henrik/transpOtterNG) TranspOtter is an open source semi self driving transportation platform based on hoverboard hardware
* [BiPropellant](https://github.com/bipropellant) - fork which focusses on reliable machine control, but also retains HoverBoard functionality if desired.
