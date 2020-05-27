//
// Created by pablito on 20.05.2020.
//
#include "spi.h"

uint8_t*            spi1_in_buffer;
uint8_t*            spi1_out_buffer;
uint8_t*            spi2_in_buffer;
uint8_t*            spi2_out_buffer;
uint32_t dma_buffer_size;

void chip_select_init(CS_Pin* pin, GPIO_TypeDef* gpio, uint16_t pinNumber){
    pin->GPIO       = gpio;
    pin->PinNumber  = pinNumber;
}

int spi_init(SPI_Dev* dev, SPI_TypeDef* SPI, DMA_InitTypeDef* dmaInitStructure, CS_Pin* chipSelect,
        uint8_t irqChannel, uint32_t dmaChannel, DMA_Stream_TypeDef *txDmaStream,
        DMA_Stream_TypeDef *rxDmaStream, uint32_t dmaTxFlag, uint32_t dmaRxFlag, uint8_t priority){

    SPI_InitTypeDef             SPI_InitStruct;

    dev->SPI                  = SPI;
    dev->ChipSelect           = chipSelect;
    dev->IRQChannel           = irqChannel;
    dev->DMA_InitStructure    = dmaInitStructure;
    dev->DMA_Channel          = dmaChannel;
    dev->TX_DMA_Stream        = txDmaStream;
    dev->RX_DMA_Stream        = rxDmaStream;
    dev->DMA_FLAG_TX          = dmaTxFlag;
    dev->DMA_FLAG_RX          = dmaRxFlag;
    if(SPI == SPI1) {
        dev->in_buffer = spi1_in_buffer;
        dev->out_buffer = spi1_out_buffer;
    }
    if(SPI == SPI2) {
        dev->in_buffer = spi2_in_buffer;
        dev->out_buffer = spi2_out_buffer;
    }

    SPI_I2S_DeInit(dev->SPI);
    SPI_InitStruct.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_Mode              = SPI_Mode_Master;
    SPI_InitStruct.SPI_DataSize          = SPI_DataSize_8b;
    SPI_InitStruct.SPI_CPOL              = SPI_CPOL_High;
    SPI_InitStruct.SPI_CPHA              = SPI_CPHA_2Edge;
    SPI_InitStruct.SPI_NSS               = SPI_NSS_Soft;
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;  // 42/64 = 0.65625 MHz SPI Clock
    SPI_InitStruct.SPI_FirstBit          = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_CRCPolynomial     = 7;

    SPI_Init(dev->SPI, &SPI_InitStruct);
    SPI_CalculateCRC(dev->SPI, DISABLE);
    SPI_Cmd(dev->SPI, ENABLE);

    while (SPI_I2S_GetFlagStatus(dev->SPI, SPI_I2S_FLAG_TXE) == RESET);

    *dummyread = SPI_I2S_ReceiveData(dev->SPI);

    dev->DMA_InitStructure->DMA_FIFOMode = DMA_FIFOMode_Disable;
    dev->DMA_InitStructure->DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dev->DMA_InitStructure->DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dev->DMA_InitStructure->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dev->DMA_InitStructure->DMA_MemoryInc = DMA_MemoryInc_Enable;
    dev->DMA_InitStructure->DMA_Mode = DMA_Mode_Normal;
    dev->DMA_InitStructure->DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    dev->DMA_InitStructure->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dev->DMA_InitStructure->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dev->DMA_InitStructure->DMA_Channel = dev->DMA_Channel;
    dev->DMA_InitStructure->DMA_PeripheralBaseAddr = (uint32_t)(&(SPI->DR));
    dev->DMA_InitStructure->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dev->DMA_InitStructure->DMA_Priority = DMA_Priority_High;

    // Configure the Appropriate Interrupt Routine
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = dev->IRQChannel;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = priority;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = priority;
    NVIC_Init(&NVIC_InitStruct);

    dev->callback = NULL;

    return *dummyread;
}



uint8_t spi_transfer(SPI_Dev *dev, uint8_t data) {
    uint16_t spiTimeout = 0x1000;
    uint8_t received = 0;

    while (SPI_I2S_GetFlagStatus(dev->SPI, SPI_I2S_FLAG_TXE) == RESET)
        if ((spiTimeout--) == 0)
            return(0);

    SPI_I2S_SendData(dev->SPI, data);
    spiTimeout = 0x1000;

    while (SPI_I2S_GetFlagStatus(dev->SPI, SPI_I2S_FLAG_RXNE) == RESET)
        if ((spiTimeout--) == 0)
            return(0);

    received = (uint8_t)SPI_I2S_ReceiveData(dev->SPI);
    return received;
}


void spi_enable(CS_Pin *pin){
    GPIO_ResetBits(pin->GPIO, pin->PinNumber);
}

void spi_disable(CS_Pin *pin){
    GPIO_SetBits(pin->GPIO, pin->PinNumber);
}


void spi_perform_transfer(SPI_Dev *dev){

    DMA_DeInit(dev->TX_DMA_Stream); // SPI1_TX_DMA_STREAM
    DMA_DeInit(dev->RX_DMA_Stream); // SPI1_RX_DMA_STREAM

    dev->DMA_InitStructure->DMA_BufferSize = dma_buffer_size;

    // Configure Tx DMA
    dev->DMA_InitStructure->DMA_DIR = DMA_DIR_MemoryToPeripheral;
    dev->DMA_InitStructure->DMA_Memory0BaseAddr = (uint32_t)(dev->out_buffer);
    DMA_Init(dev->TX_DMA_Stream, dev->DMA_InitStructure);

    // Configure Rx DMA
    dev->DMA_InitStructure->DMA_DIR = DMA_DIR_PeripheralToMemory;
    dev->DMA_InitStructure->DMA_Memory0BaseAddr = (uint32_t)(dev->in_buffer);
    DMA_Init(dev->RX_DMA_Stream, dev->DMA_InitStructure);
    //  Configure the Interrupt

    DMA_ITConfig(dev->TX_DMA_Stream, DMA_IT_TC, ENABLE);


    spi_enable(dev->ChipSelect);

    // Turn on the DMA streams
    DMA_Cmd(dev->TX_DMA_Stream, ENABLE);
    DMA_Cmd(dev->RX_DMA_Stream, ENABLE);

    // Enable the SPI Rx/Tx DMA request
    SPI_I2S_DMACmd(dev->SPI, SPI_I2S_DMAReq_Rx, ENABLE);
    SPI_I2S_DMACmd(dev->SPI, SPI_I2S_DMAReq_Tx, ENABLE);
}


void mpu_transfer_complete_callback(void){
    spi_disable(MPU6000.ChipSelect);
    DMA_ClearFlag(MPU6000.TX_DMA_Stream, MPU6000.DMA_FLAG_TX);
    DMA_ClearFlag(MPU6000.RX_DMA_Stream, MPU6000.DMA_FLAG_RX);

    DMA_Cmd(MPU6000.TX_DMA_Stream, DISABLE);
    DMA_Cmd(MPU6000.RX_DMA_Stream, DISABLE);
                                                     
    SPI_I2S_DMACmd(MPU6000.SPI, SPI_I2S_DMAReq_Rx, DISABLE);
    SPI_I2S_DMACmd(MPU6000.SPI, SPI_I2S_DMAReq_Tx, DISABLE);

    MPU6000.busy = false;

    if (MPU6000.callback != NULL)
        MPU6000.callback();
}

void m25p16_transfer_complete_callback(void){
    spi_disable(M25P16.ChipSelect);
    DMA_ClearFlag(M25P16.TX_DMA_Stream, M25P16.DMA_FLAG_TX);
    DMA_ClearFlag(M25P16.RX_DMA_Stream, M25P16.DMA_FLAG_RX);

    DMA_Cmd(M25P16.TX_DMA_Stream, DISABLE);
    DMA_Cmd(M25P16.RX_DMA_Stream, DISABLE);

    SPI_I2S_DMACmd(M25P16.SPI, SPI_I2S_DMAReq_Rx, DISABLE);
    SPI_I2S_DMACmd(M25P16.SPI, SPI_I2S_DMAReq_Tx, DISABLE);

    M25P16.busy = false;

    if (M25P16.callback != NULL)
        M25P16.callback();
}

void dma_mem_write(SPI_Dev* dev, const uint8_t* out_data, uint32_t number_of_bytes){
    dev->busy = true;
    dma_buffer_size = number_of_bytes;
    dev->in_buffer  = dummyread;
    dev->out_buffer = (out_data == NULL) ? dummyread : out_data;
    dev->callback = NULL;
    spi_perform_transfer(dev);
}

void dma_transfer(SPI_Dev *dev, uint8_t* out_data, uint8_t* in_data, uint32_t number_of_bytes, void (*callback)(void)){
    dev->busy = true;
    dma_buffer_size = number_of_bytes;
    dev->in_buffer = (in_data == NULL) ? dummyread : in_data;
    dev->out_buffer = (out_data == NULL) ? dummyread : out_data;
    dev->callback = callback;
    spi_perform_transfer(dev);
}

bool is_busy(SPI_Dev* dev) { return dev->busy; }

void set_spi_divisor(SPI_Dev* dev, uint16_t data) {
#define BR_CLEAR_MASK 0xFFC7

    uint16_t tempRegister;
    SPI_Cmd(dev->SPI, DISABLE);
    tempRegister = dev->SPI->CR1;

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

    dev->SPI->CR1 = tempRegister;
    SPI_Cmd(dev->SPI, ENABLE);
}

void DMA2_Stream3_IRQHandler()
{
    if (DMA_GetITStatus(DMA2_Stream3, DMA_IT_TCIF3))
    {
        DMA_ClearITPendingBit(DMA2_Stream3, DMA_IT_TCIF3);
        mpu_transfer_complete_callback();
    }
}


void DMA1_Stream4_IRQHandler()
{
    if (DMA_GetITStatus(DMA1_Stream4, DMA_IT_TCIF4))
    {
        DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);
        m25p16_transfer_complete_callback();
    }
}
