#include <stdio.h>
#include <string.h>
#include "at32f4xx.h"
#include "mpu.h"

uint8_t MPU6050_Init()
{
    uint8_t check, temp;

    check = I2C2_Read(MPU6050_ADDR, WHO_AM_I_REG);

    I2C2_Write(MPU6050_ADDR, PWR_MGMT_1_REG, 0x01);

    I2C2_Write(MPU6050_ADDR, SMPLRT_DIV_REG, 0x07);

    I2C2_Write(MPU6050_ADDR, RA_CONFIG, 0x07);

    I2C2_Write(MPU6050_ADDR, ACCEL_CONFIG_REG, 0x00);

    I2C2_Write(MPU6050_ADDR, GYRO_CONFIG_REG, 0x00);

    I2C2_Write(MPU6050_ADDR, INT_PIN_CFG, 0x10);

    I2C2_Write(MPU6050_ADDR, INT_PIN_EN, 0x01);

    return check;
}

void MPU6050_Read(MPU6050Data *MPUData)
{
    //uint8_t data[14];

    /* Read gyroscope data */
    //I2C2_ReadMulti(MPU6050_ADDR, ACCEL_XOUT_H_REG, data, 14);

    // MPUData->Accel_X = (int16_t)(data[0] << 8 | data[1]) / 16384.0f;
    // MPUData->Accel_Y = (int16_t)(data[2] << 8 | data[3]) / 16384.0f;
    // MPUData->Accel_Z = (int16_t)(data[4] << 8 | data[5]) / 16384.0f;

    // MPUData->Gyro_X = (int16_t)(data[8] << 8 | data[9]) / 131.0f;
    // MPUData->Gyro_Y = (int16_t)(data[10] << 8 | data[11]) / 131.0f;
    // MPUData->Gyro_Z = (int16_t)(data[12] << 8 | data[13]) / 131.0f;

    MPUData->Accel_X = mpu_data[0] / 16384.0f;
    MPUData->Accel_Y = mpu_data[1] / 16384.0f;
    MPUData->Accel_Z = mpu_data[2] / 16384.0f;

    MPUData->Gyro_X = mpu_data[4] / 131.0f;
    MPUData->Gyro_Y = mpu_data[5] / 131.0f;
    MPUData->Gyro_Z = mpu_data[6] / 131.0f;
}

void MPU6050_GyroRead(MPU6050Data *MPUData)
{
    uint8_t data[6];

    /* Read gyroscope data */
    I2C2_ReadMulti(MPU6050_ADDR, GYRO_XOUT_H_REG, data, 6);

    /* Format */
    MPUData->Gyro_X = (int16_t)(data[0] << 8 | data[1]) / 131.0f;
    MPUData->Gyro_Y = (int16_t)(data[2] << 8 | data[3]) / 131.0f;
    MPUData->Gyro_Z = (int16_t)(data[4] << 8 | data[5]) / 131.0f;
}

void MPU6050_AccelRead(MPU6050Data *MPUData)
{
    uint8_t data[6];

    /* Read gyroscope data */
    I2C2_ReadMulti(MPU6050_ADDR, ACCEL_XOUT_H_REG, data, 6);

    /* Format */
    MPUData->Accel_X = (int16_t)(data[0] << 8 | data[1]) / 16384.0f;
    MPUData->Accel_Y = (int16_t)(data[2] << 8 | data[3]) / 16384.0f;
    MPUData->Accel_Z = (int16_t)(data[4] << 8 | data[5]) / 16384.0f;
}