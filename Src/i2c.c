#include <stdio.h>
#include <string.h>
#include "at32f4xx.h"
#include "i2c.h"

uint8_t i2c_rx_Buffer[14];
uint32_t I2C2_Timeout;

void I2C2_Init()
{
    GPIO_InitType GPIO_I2C_Structure;
    I2C_InitType MPU_I2C_Structure;
    NVIC_InitType NVIC_I2C_Structure;
    DMA_InitType DMA_I2C_Structure;
    EXTI_InitType EXTI_I2C_Structure;

    RCC_APB1PeriphClockCmd(RCC_APB1PERIPH_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2PERIPH_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPERIPH_DMA1, ENABLE);

    GPIO_I2C_Structure.GPIO_Pins = GPIO_Pins_10 | GPIO_Pins_11;
    GPIO_I2C_Structure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_I2C_Structure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
    GPIO_Init(GPIOB, &GPIO_I2C_Structure);

    GPIO_I2C_Structure.GPIO_Pins = GPIO_Pins_2;
    GPIO_I2C_Structure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_I2C_Structure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
    GPIO_Init(GPIOB, &GPIO_I2C_Structure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinsSource2);

    MPU_I2C_Structure.I2C_BitRate = 400000;
    MPU_I2C_Structure.I2C_Mode = I2C_Mode_I2CDevice;
    MPU_I2C_Structure.I2C_FmDutyCycle = I2C_FmDutyCycle_2_1;
    MPU_I2C_Structure.I2C_OwnAddr1 = 0;
    MPU_I2C_Structure.I2C_Ack = 0;
    MPU_I2C_Structure.I2C_AddrMode = I2C_AddrMode_7bit;
    I2C_Init(I2C2, &MPU_I2C_Structure);

    DMA_Reset(DMA1_Channel5);

    DMA_I2C_Structure.DMA_PeripheralBaseAddr = (uint32_t)0x40005810;
    DMA_I2C_Structure.DMA_MemoryBaseAddr = (uint32_t)i2c_rx_Buffer;
    DMA_I2C_Structure.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
    DMA_I2C_Structure.DMA_Mode = DMA_MODE_NORMAL;
    DMA_I2C_Structure.DMA_Priority = DMA_PRIORITY_MEDIUM;
    DMA_I2C_Structure.DMA_Direction = DMA_DIR_PERIPHERALSRC;
    DMA_I2C_Structure.DMA_BufferSize = 14;
    DMA_I2C_Structure.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
    DMA_I2C_Structure.DMA_MemoryInc = DMA_MEMORYINC_DISABLE;
    DMA_I2C_Structure.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_BYTE;
    DMA_I2C_Structure.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_BYTE;
    DMA_Init(DMA1_Channel5, &DMA_I2C_Structure);
    DMA_INTConfig(DMA1_Channel5, DMA_INT_TC, ENABLE);

    NVIC_I2C_Structure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
    NVIC_I2C_Structure.NVIC_IRQChannelPreemptionPriority = 0x05;
    NVIC_I2C_Structure.NVIC_IRQChannelSubPriority = 0x05;
    NVIC_I2C_Structure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_I2C_Structure);

    GPIO_I2C_Structure.GPIO_Pins = GPIO_Pins_2;
    GPIO_I2C_Structure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_I2C_Structure.GPIO_MaxSpeed = GPIO_MaxSpeed_50MHz;
    GPIO_Init(GPIOB, &GPIO_I2C_Structure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinsSource2);

    EXTI_I2C_Structure.EXTI_Line = EXTI_Line2;
    EXTI_I2C_Structure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_I2C_Structure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_I2C_Structure.EXTI_LineEnable = ENABLE;
    EXTI_Init(&EXTI_I2C_Structure);

    NVIC_I2C_Structure.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_I2C_Structure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_I2C_Structure.NVIC_IRQChannelSubPriority = 1;
    NVIC_I2C_Structure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_I2C_Structure);
}

void DMA1_Channel5_IRQHandler(void)
{
    if (DMA_GetFlagStatus(DMA1_FLAG_TC5))
    {
        DMA_ClearFlag(DMA1_FLAG_TC5);

        I2C_DMACmd(I2C2, DISABLE);
        I2C_GenerateSTOP(I2C2, ENABLE);

        DMA_ChannelEnable(DMA1_Channel5, DISABLE);

        for (int i = 0; i < 7; i++)
        {
            mpu_data[i] = (int16_t)(i2c_rx_Buffer[2 * i] << 8 | i2c_rx_Buffer[2 * i + 1]);
        }
    }
}

void I2C2_DMA_Read(uint8_t devaddress, uint8_t memregister)
{
    DMA_ChannelEnable(DMA1_Channel5, DISABLE);

    DMA_SetCurrDataCounter(DMA1_Channel5, 14);

    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSYF))
        ;

    I2C_DMALastTransferCmd(I2C2, ENABLE);

    I2C_GenerateSTART(I2C2, ENABLE);

    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_START_GENERATED))
        ;

    I2C_Send7bitAddress(I2C2, devaddress, I2C_Direction_Transmit);

    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER))
        ;

    I2C_Cmd(I2C2, ENABLE);

    I2C_SendData(I2C2, memregister);

    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_DATA_TRANSMITTED))
        ;

    I2C_GenerateSTART(I2C2, ENABLE);

    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_START_GENERATED))
        ;

    I2C_Send7bitAddress(I2C2, devaddress, I2C_Direction_Receive);

    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_ADDRESS_WITH_RECEIVER))
        ;

    DMA_ChannelEnable(DMA1_Channel5, ENABLE);
    I2C_DMACmd(I2C2, ENABLE);
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