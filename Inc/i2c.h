#pragma once

#include <stdint.h>
#include "at32f4xx.h"
#include <math.h>

#define I2C2_TIMEOUT 20000

uint16_t mpu_data[14];

void I2C2_Init();
void I2C2_DMA_Read(uint8_t devaddress, uint8_t memregister);
uint8_t I2C2_Read(uint8_t devaddress, uint8_t memregister);
void I2C2_ReadMulti(uint8_t devaddress, uint8_t memregister, uint8_t *data, uint16_t count);
void I2C2_Write(uint8_t devaddress, uint8_t memregister, uint8_t data);
int16_t I2C2_Start(I2C_Type *I2Cx, uint8_t address, uint8_t direction, uint8_t ack);
uint8_t I2C2_Stop(I2C_Type *I2Cx);
void I2C2_WriteData(I2C_Type *I2Cx, uint8_t data);
uint8_t I2C2_ReadData(I2C_Type *I2Cx, uint8_t ack);