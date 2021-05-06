#include <stdio.h>
#include <string.h>
#include "at32f4xx.h"
#include "defines.h"
#include "setup.h"
#include "config.h"

volatile uint8_t uart_buf[100];
volatile int16_t ch_buf[7];

void setScopeChannel(uint8_t ch, int16_t val)
{
  ch_buf[ch] = val;
}

char *get_diagnostic_info()
{
  memset((void *)uart_buf, 0, sizeof(uart_buf));
  sprintf((char *)uart_buf, "1:%i 2:%i 3:%i 4:%i 5:%i 6:%i 7:%i",
          ch_buf[0], ch_buf[1], ch_buf[2], ch_buf[3], ch_buf[4], ch_buf[5],
          ch_buf[6]);
  return (char *)uart_buf;
}