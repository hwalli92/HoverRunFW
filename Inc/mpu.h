#include <stdint.h>
#include "at32f4xx.h"
#include <math.h>
#include <i2c.h>

#define RAD_TO_DEG 57.295779513082320876798154814105

#define MPU6050_ADDR 0x69

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define RA_CONFIG 0x1A
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define INT_PIN_CFG 0x37
#define INT_PIN_EN 0x38

typedef struct
{
    float Accel_X;
    float Accel_Y;
    float Accel_Z;
    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;
} MPU6050Data;

uint8_t MPU6050_Init();
void MPU6050_Read(MPU6050Data *MPUData);
void MPU6050_GyroRead(MPU6050Data *MPUData);
void MPU6050_AccelRead(MPU6050Data *MPUData);