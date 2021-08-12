#include <stdio.h>
#include <string.h>
#include "at32f4xx.h"
#include "i2c.h"

DMA_InitType hdma_i2c2_rx = {0};
DMA_InitType hdma_i2c2_tx = {0};
I2C_InitType hi2c2 = {0};

uint8_t gyro_data[6] = {0};
uint8_t i2cBuffer[2];
uint32_t I2C2_Timeout;

void I2C2_Init()
{
    GPIO_InitType GPIO_I2C_Structure;
    I2C_InitType MPU_I2C_Structure;

    RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOB, ENABLE);

    GPIO_I2C_Structure.GPIO_Pins = GPIO_Pins_10 | GPIO_Pins_11;
    GPIO_I2C_Structure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_I2C_Structure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
    GPIO_Init(GPIOB, &GPIO_I2C_Structure);

    MPU_I2C_Structure.I2C_BitRate = 400000;
    MPU_I2C_Structure.I2C_Mode = I2C_Mode_I2CDevice;
    MPU_I2C_Structure.I2C_FmDutyCycle = I2C_FmDutyCycle_2_1;
    MPU_I2C_Structure.I2C_OwnAddr1 = 0;
    MPU_I2C_Structure.I2C_Ack = 0;
    MPU_I2C_Structure.I2C_AddrMode = I2C_AddrMode_7bit;
    I2C_Init(I2C2, &MPU_I2C_Structure);

    // RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_I2C2, ENABLE);
    // RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_DMA1, ENABLE);

    // /* DMA1_Channel4_IRQn interrupt configuration */
    // NVIC_InitType NVIC_InitStruct = {0};

    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    // NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    // NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    // NVIC_InitStruct.NVIC_IRQChannelSubPriority = 4;
    // NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_Init(&NVIC_InitStruct);

    // /* DMA1_Channel5_IRQn interrupt configuration */
    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    // NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel5_IRQn;
    // NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    // NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
    // NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_Init(&NVIC_InitStruct);

    // hi2c2.I2C_BitRate = 100000;
    // hi2c2.I2C_FmDutyCycle = I2C_FmDutyCycle_2_1;
    // hi2c2.I2C_OwnAddr1 = 0;
    // hi2c2.I2C_AddrMode = I2C_AddrMode_7bit;
    // I2C_Init(I2C2, &hi2c2);

    // GPIO_InitType GPIO_InitStruct = {0};

    // RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_DMA1, ENABLE);
    // RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOB, ENABLE);
    // /* USER CODE BEGIN I2C2_MspInit 0 */

    // /* USER CODE END I2C2_MspInit 0 */

    // /**I2C2 GPIO Configuration
    // PB10     ------> I2C2_SCL
    // PB11     ------> I2C2_SDA
    // */
    // GPIO_InitStruct.GPIO_Pins = GPIO_Pins_10 | GPIO_Pins_11;
    // GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    // GPIO_InitStruct.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
    // GPIO_Init(GPIOB, &GPIO_InitStruct);

    // /* Peripheral clock enable */
    // RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_I2C2, ENABLE);

    // /* Peripheral DMA init*/

    // hdma_i2c2_rx.DMA_Direction = DMA_DIR_PERIPHERALSRC;
    // hdma_i2c2_rx.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    // hdma_i2c2_rx.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
    // hdma_i2c2_rx.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_BYTE;
    // hdma_i2c2_rx.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_BYTE;
    // hdma_i2c2_rx.DMA_Mode = DMA_MODE_NORMAL;
    // hdma_i2c2_rx.DMA_Priority = DMA_PRIORITY_MEDIUM;
    // DMA_Init(DMA1_Channel5, &hdma_i2c2_rx);

    // //__HAL_LINKDMA(&hi2c2, hdmarx, hdma_i2c2_rx);

    // hdma_i2c2_tx.DMA_Direction = DMA_DIR_PERIPHERALDST;
    // hdma_i2c2_tx.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    // hdma_i2c2_tx.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
    // hdma_i2c2_tx.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_BYTE;
    // hdma_i2c2_tx.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_BYTE;
    // hdma_i2c2_tx.DMA_Mode = DMA_MODE_NORMAL;
    // hdma_i2c2_tx.DMA_Priority = DMA_PRIORITY_MEDIUM;
    // DMA_Init(DMA1_Channel4, &hdma_i2c2_tx);

    // //__HAL_LINKDMA(&hi2c2, hdmatx, hdma_i2c2_tx);

    // /* Peripheral interrupt init */

    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    // NVIC_InitStruct.NVIC_IRQChannel = I2C2_EV_IRQn;
    // NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    // NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    // NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_Init(&NVIC_InitStruct);

    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    // NVIC_InitStruct.NVIC_IRQChannel = I2C2_ER_IRQn;
    // NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    // NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    // NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    // NVIC_Init(&NVIC_InitStruct);
    // /* USER CODE BEGIN I2C2_MspInit 1 */

    // /* USER CODE END I2C2_MspInit 1 */
}

uint8_t I2C2_Read(uint8_t devaddress, uint8_t memregister)
{
    uint8_t received_data;

    I2C2_Start(I2C2, devaddress, I2C_Direction_Transmit, 0);
    I2C2_WriteData(I2C2, memregister);
    I2C2_Stop(I2C2);
    I2C2_Start(I2C2, devaddress, I2C_Direction_Receive, 0);
    received_data = I2C2_ReadData(I2C2, 0);

    return received_data;
}

void I2C2_Write(uint8_t devaddress, uint8_t memregister, uint8_t data)
{
    I2C2_Start(I2C2, devaddress, I2C_Direction_Transmit, 0);
    I2C2_WriteData(I2C2, memregister);
    I2C2_WriteData(I2C2, data);
    I2C2_Stop(I2C2);
}

void I2C2_ReadMulti(uint8_t devaddress, uint8_t memregister, uint8_t *data, uint16_t count)
{
    uint8_t i;
    I2C2_Start(I2C2, devaddress, I2C_Direction_Transmit, 1);
    I2C2_WriteData(I2C2, memregister);
    I2C2_Stop(I2C2);
    I2C2_Start(I2C2, devaddress, I2C_Direction_Receive, 1);
    for (i = 0; i < count; i++)
    {
        if (i == (count - 1))
        {
            //Last byte
            data[i] = I2C2_ReadData(I2C2, 0);
        }
        else
        {
            data[i] = I2C2_ReadData(I2C2, 1);
        }
    }
}

int16_t I2C2_Start(I2C_Type *I2Cx, uint8_t address, uint8_t direction, uint8_t ack)
{
    I2C_GenerateSTART(I2Cx, ENABLE);

    I2C2_Timeout = I2C2_TIMEOUT;
    while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_STARTF) && I2C2_Timeout)
    {
        if (--I2C2_Timeout == 0x00)
        {
            return 1;
        }
    }

    if (ack)
    {
        I2C_AcknowledgeConfig(I2Cx, ENABLE);
    }

    I2C_Send7bitAddress(I2Cx, address, direction);

    if (direction == I2C_Direction_Transmit)
    {
        I2C2_Timeout = I2C2_TIMEOUT;
        while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_ADDRF) && I2C2_Timeout)
        {
            if (--I2C2_Timeout == 0x00)
            {
                return 1;
            }
        }
    }
    else if (direction == I2C_Direction_Receive)
    {
        I2C2_Timeout = I2C2_TIMEOUT;
        while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_ADDRESS_WITH_RECEIVER) && I2C2_Timeout)
        {
            if (--I2C2_Timeout == 0x00)
            {
                return 1;
            }
        }
    }

    return 0;
}

uint8_t I2C2_Stop(I2C_Type *I2Cx)
{
    I2C2_Timeout = I2C2_TIMEOUT;
    while (((!I2C_GetFlagStatus(I2Cx, I2C_FLAG_TDE)) || (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_BTFF))) && I2C2_Timeout)
    {
        if (--I2C2_Timeout == 0x00)
        {
            return 1;
        }
    }
    I2C_GenerateSTOP(I2Cx, ENABLE);

    return 0;
}

void I2C2_WriteData(I2C_Type *I2Cx, uint8_t data)
{
    I2C2_Timeout = I2C2_TIMEOUT;
    while (!I2C_GetFlagStatus(I2Cx, I2C_FLAG_TDE) && I2C2_Timeout)
    {
        I2C2_Timeout--;
    }
    I2C_SendData(I2Cx, data);
}

uint8_t I2C2_ReadData(I2C_Type *I2Cx, uint8_t ack)
{
    uint8_t data;

    if (ack)
    {
        I2C_AcknowledgeConfig(I2Cx, ENABLE);
    }
    else
    {
        I2C_AcknowledgeConfig(I2Cx, DISABLE);

        I2C_GenerateSTOP(I2Cx, ENABLE);
    }

    I2C2_Timeout = I2C2_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_DATA_RECEIVED) && I2C2_Timeout)
    {
        I2C2_Timeout--;
    }

    data = I2C_ReceiveData(I2Cx);
    return data;
}