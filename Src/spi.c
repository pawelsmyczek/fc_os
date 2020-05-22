//
// Created by pablito on 20.05.2020.
//
#include "spi.h"


int spi_init(void){


    SPI_I2S_DeInit(SPI1);
    SPI_InitStruct.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_Mode              = SPI_Mode_Master;
    SPI_InitStruct.SPI_DataSize          = SPI_DataSize_8b;
    SPI_InitStruct.SPI_CPOL              = SPI_CPOL_High;
    SPI_InitStruct.SPI_CPHA              = SPI_CPHA_2Edge;
    SPI_InitStruct.SPI_NSS               = SPI_NSS_Soft;
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;  // 42/64 = 0.65625 MHz SPI Clock
    SPI_InitStruct.SPI_FirstBit          = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_CRCPolynomial     = 7;

    SPI_Init(SPI1, &SPI_InitStruct);
    SPI_CalculateCRC(SPI1, DISABLE);
    SPI_Cmd(SPI1, ENABLE);

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

    *dummyread = SPI_I2S_ReceiveData(SPI1);

    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Channel = DMA_Channel_3;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI1->DR));
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;

    // Configure the Appropriate Interrupt Routine
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_Init(&NVIC_InitStruct);

    cb = NULL;

    return *dummyread;
}


void spi_tx(uint8_t address, uint8_t data){
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);

    while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
    SPI_I2S_SendData(SPI1, address);
    while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
    SPI_I2S_ReceiveData(SPI1);
    while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
    SPI_I2S_SendData(SPI1, data);
    while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
    SPI_I2S_ReceiveData(SPI1);

    GPIO_SetBits(GPIOA, GPIO_Pin_4);
}


uint8_t spi_transfer(uint8_t data) {
    uint16_t spiTimeout = 0x1000;
    uint8_t received = 0;

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
        if ((spiTimeout--) == 0)
            return(0);

    SPI_I2S_SendData(SPI1, data);
    spiTimeout = 0x1000;

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
        if ((spiTimeout--) == 0)
            return(0);

    received = (uint8_t)SPI_I2S_ReceiveData(SPI1);
    return received;
}


void spi_perform_transfer(){
    DMA_DeInit(DMA2_Stream3); // SPI1_TX_DMA_STREAM
    DMA_DeInit(DMA2_Stream2); // SPI1_RX_DMA_STREAM

    DMA_InitStructure.DMA_BufferSize = dma_buffer_size;

    // Configure Tx DMA
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(in_buffer);
    DMA_Init(DMA2_Stream3, &DMA_InitStructure);

    // Configure Rx DMA
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(out_buffer);
    DMA_Init(DMA2_Stream2, &DMA_InitStructure);
    //  Configure the Interrupt
    DMA_ITConfig(DMA2_Stream3, DMA_IT_TC, ENABLE);


    ENABLE_SPI;

    // Turn on the DMA streams
    DMA_Cmd(DMA2_Stream3, ENABLE);
    DMA_Cmd(DMA2_Stream2, ENABLE);

    // Enable the SPI Rx/Tx DMA request
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

    // SPI_Cmd(SPI1, ENABLE);
}


void transfer_complete_callback(void){
    DISABLE_SPI;
    DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3);
    DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);

    DMA_Cmd(DMA2_Stream3, DISABLE);
    DMA_Cmd(DMA2_Stream2, DISABLE);
                                                     
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, DISABLE);
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);

    // SPI_Cmd(SPI1, DISABLE);

    if (cb != NULL)
        cb();
}


void dma_transfer(uint8_t* in_data, uint8_t* out_data, uint8_t number_of_bytes, void (*callback)(void)){
    dma_buffer_size = number_of_bytes;
    in_buffer = (in_data == NULL) ? dummyread : in_data;
    out_buffer = (out_data == NULL) ? dummyread : out_data;

    cb = callback;
    spi_perform_transfer();
}


void set_spi_divisor(uint16_t data)
{
#define BR_CLEAR_MASK 0xFFC7

    uint16_t tempRegister;
    SPI_Cmd(SPI1, DISABLE);
    tempRegister = SPI1->CR1;

    switch (data)
    {
        case 2:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_2;
            break;
        case 4:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_4;
            break;
        case 8:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_8;
            break;
        case 16:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_16;
            break;
        case 32:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_32;
            break;
        case 64:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_64;
            break;
        case 128:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_128;
            break;
        case 256:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_256;
            break;
    }

    SPI1->CR1 = tempRegister;
    SPI_Cmd(SPI1, ENABLE);
}

void DMA2_Stream3_IRQHandler()
{
    if (DMA_GetITStatus(DMA2_Stream3, DMA_IT_TCIF3))
    {
        DMA_ClearITPendingBit(DMA2_Stream3, DMA_IT_TCIF3);
        transfer_complete_callback();
    }
}

