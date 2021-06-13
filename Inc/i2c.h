#pragma once

#include <stdint.h>
#include "at32f4xx.h"
#include <math.h>

#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

// Setup MPU6050
#define MPU6050_ADDR 0xD0

#define I2C2_TIMEOUT 20000

typedef struct
{
    float Accel_X;
    float Accel_Y;
    float Accel_Z;
    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;
} MPU6050Data;

void I2C2_Init();
uint8_t MPU6050_Init();
void MPU6050_Read(MPU6050Data *MPUData);
void MPU6050_GyroRead(MPU6050Data *MPUData);
void MPU6050_AccelRead(MPU6050Data *MPUData);
uint8_t I2C2_Read(uint8_t devaddress, uint8_t memregister);
void I2C2_ReadMulti(uint8_t devaddress, uint8_t memregister, uint8_t *data, uint16_t count);
void I2C2_Write(uint8_t devaddress, uint8_t memregister, uint8_t data);
int16_t I2C2_Start(I2C_Type *I2Cx, uint8_t address, uint8_t direction, uint8_t ack);
uint8_t I2C2_Stop(I2C_Type *I2Cx);
void I2C2_WriteData(I2C_Type *I2Cx, uint8_t data);
uint8_t I2C2_ReadData(I2C_Type *I2Cx, uint8_t ack);