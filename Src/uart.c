#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "uart.h"

#define COMMAND_DELIMITER ';'

volatile circ_buffer_t uart_rx_circBuff = {uart_rxBuff, 0, 0};
volatile circ_buffer_t uart_tx_circBuff = {uart_txBuff, 0, 0};

int8_t uart_command_ptr = 0;

void uart_initialize()
{
	memset((void *)uart_command, 0, sizeof(uart_command));

	GPIO_InitType GPIO_InitStruct = {0};
	USART_InitType USART_InitStruct = {0};
	NVIC_InitType NVIC_InitStruct = {0};

	RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_DMA1, ENABLE);

	GPIO_InitStruct.GPIO_Pins = GPIO_Pins_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pins = GPIO_Pins_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	USART_InitStruct.USART_BaudRate = CONTROL_UART_BAUD;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART3, &USART_InitStruct);

	USART_INTConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_INTConfig(USART3, USART_IT_TXE, ENABLE);
	USART_Cmd(USART3, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

void USART3_IRQHandler(void)
{
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		uint8_t head_temp = uart_rx_circBuff.head + 1;

		if (head_temp == UART_RX_BUF_SIZE)
			head_temp = 0;

		if (head_temp == uart_rx_circBuff.tail)
		{
			USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		}
		else
		{
			uart_rx_circBuff.buffer[head_temp] = USART_ReceiveData(USART3);
			uart_rx_circBuff.head = head_temp;
		}
	}

	if (USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
	{
		if (uart_tx_circBuff.head == uart_tx_circBuff.tail)
		{
			USART_INTConfig(USART3, USART_IT_TXE, DISABLE);
		}
		else
		{
			uart_tx_circBuff.tail++;

			if (uart_tx_circBuff.tail == UART_TX_BUF_SIZE)
				uart_tx_circBuff.tail = 0;

			USART_SendData(USART3,
						   uart_tx_circBuff.buffer[uart_tx_circBuff.tail]);
		}
	}
}

void uart_handle_command()
{
	if (uart_read_command() == 1)
	{
		if (!strcmp(uart_command, "ping"))
		{
			uart_put_string("pong" NL);
		}
		else if (!strcmp(uart_command, "test"))
		{
			update_timeout();
			uart_put_string("starting motor test" NL);
			set_steer(0);
			set_speed(100);
		}
		else if (startswith(uart_command, "move"))
		{
			update_timeout();
			int steer;
			int speed;
			int chksum;
			sscanf(uart_command, "move %d %d %d", &steer, &speed, &chksum);
			if (chksum == steer + (speed * 1000))
			{
				// char out[128];
				// sprintf(out, "setting motors speed:%d steer:%d" NL, speed,
				// 		steer);
				// uart_put_string(out);
				set_steer(steer);
				set_speed(speed);
			}
			else
			{
				uart_put_string("checksum error" NL);
			}
		}
		else if (startswith(uart_command, "mpu"))
		{
			update_timeout();
			char strgyrox[20];
			char straccx[20];
			char strcfanglex[20];
			//char out[128];
			sscanf(uart_command, "mpu %s %s %s", &strgyrox, &straccx, &strcfanglex);
			double gyrox = atof(strgyrox);
			double accx = atof(straccx);
			double cfanglex = atof(strcfanglex);
			// sprintf(out, "setting motors from mpu speed: %f steer: %f" NL, gyrox,
			// 		accx);
			// uart_put_string(out);
			set_angle(gyrox, accx, cfanglex);
		}
		else if (!strcmp(uart_command, "stop"))
		{
			update_timeout();
			//uart_put_string("stopping motors" NL);
			set_steer(0);
			set_speed(0);
		}
		else if (!strcmp(uart_command, "poweroff"))
		{
			update_timeout();
			//uart_put_string("shutting down" NL);
			poweroff();
		}
		else if (!strcmp(uart_command, "status"))
		{
			update_timeout();
			uart_put_string(get_diagnostic_info());
			uart_put_string(NL);
		}
		else
		{
			uart_put_string("Unknown command");
			uart_put_string(uart_command);
			uart_put_string(NL);
		}
	}
}

void uart_put_string(char *s)
{
	while (*s)
		uart_put_char(*s++);
}

int8_t uart_put_char(char data)
{
	uint8_t head_temp = uart_tx_circBuff.head + 1;

	if (head_temp == UART_TX_BUF_SIZE)
		head_temp = 0;

	if (head_temp == uart_tx_circBuff.tail)
		return -1;

	uart_tx_circBuff.buffer[head_temp] = data;
	uart_tx_circBuff.head = head_temp;

	USART_INTConfig(USART3, USART_IT_TXE, ENABLE);

	return 0;
}

int8_t uart_read_command()
{
	char c;
	if (uart_get_char(&c) != -1)
	{
		if (c == '\n' || c == '\r' || c == '\0' || c == COMMAND_DELIMITER || uart_command_ptr == (sizeof(uart_command) - 1))
		{
			uart_command[uart_command_ptr] = '\0';
			uart_command_ptr = 0;
			return 1;
		}

		uart_command[uart_command_ptr] = c;
		uart_command_ptr++;
	}

	return 0;
}

int8_t uart_get_char(char *data)
{

	if (uart_rx_circBuff.head == uart_rx_circBuff.tail)
		return -1;

	uart_rx_circBuff.tail++;

	if (uart_rx_circBuff.tail == UART_RX_BUF_SIZE)
		uart_rx_circBuff.tail = 0;

	*data = uart_rx_circBuff.buffer[uart_rx_circBuff.tail];

	return 0;
}

int8_t startswith(const char *a, const char *b)
{
	if (strncmp(a, b, strlen(b)) == 0)
	{
		return 1;
	}
	return 0;
}
