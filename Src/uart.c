//
// Created by pablito on 06.11.2020.
//

#include "uart.h"


void uart_init(
                    UART_dev*           dev,
                    USART_TypeDef*      UART,
                    uint16_t            RX_Pin,
                    uint16_t            Tx_Pin,
                    uint8_t             RX_PinSource,
                    uint8_t             TX_PinSource,
                    IRQn_Type           USART_IRQn,
                    IRQn_Type           Rx_DMA_IRQn,
                    IRQn_Type           Tx_DMA_IRQn,
                    DMA_Stream_TypeDef* Rx_DMA_Stream,
                    DMA_Stream_TypeDef* Tx_DMA_Stream,
                    uint32_t            DMA_Channel,
                    uint32_t            DMA_Rx_IT_Bit,
                    uint32_t            DMA_Tx_IT_Bit,
                    uint32_t            baudrate_,
                    UART_Mode           mode)
{

    dev->UART          = UART;
    dev->RX_Pin        = RX_Pin;
    dev->Tx_Pin        = Tx_Pin;
    dev->RX_PinSource  = RX_PinSource;
    dev->TX_PinSource  = TX_PinSource;
    dev->USART_IRQn    = USART_IRQn;
    dev->Rx_DMA_IRQn   = Rx_DMA_IRQn;
    dev->Tx_DMA_IRQn   = Tx_DMA_IRQn;
    dev->Rx_DMA_Stream = Rx_DMA_Stream;
    dev->Tx_DMA_Stream = Tx_DMA_Stream;
    dev->DMA_Channel   = DMA_Channel;
    dev->DMA_Rx_IT_Bit = DMA_Rx_IT_Bit;
    dev->DMA_Tx_IT_Bit = DMA_Tx_IT_Bit;
    dev->baudrate     = baudrate_;
    // init uart the device
    USART_Cmd(dev->UART, DISABLE); // Disable the usart device for configuration

    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = dev->baudrate;

    switch (mode)
    {
        case MODE_8N1:
        default:
            USART_InitStruct.USART_WordLength = USART_WordLength_8b;
            USART_InitStruct.USART_Parity = USART_Parity_No;
            USART_InitStruct.USART_StopBits = USART_StopBits_1;
            break;
        case MODE_8E2:
            USART_InitStruct.USART_WordLength = USART_WordLength_8b;
            USART_InitStruct.USART_Parity = USART_Parity_Even;
            USART_InitStruct.USART_StopBits = USART_StopBits_2;
            break;
    }

    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // Set to both receive and send
    USART_Init(dev->UART, &USART_InitStruct);
    // USART_OverSampling8Cmd(dev->dev, ENABLE);//Please don't break anything

    // Throw interrupts on byte receive
    USART_ITConfig(dev->UART, USART_IT_RXNE, ENABLE); // enable interupts on receive
    USART_Cmd(dev->UART, ENABLE);                     // reenable the usart

    DMA_InitTypeDef DMA_InitStructure;

    // Common DMA Configuration
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(dev->UART->DR));
    DMA_InitStructure.DMA_Channel =dev->DMA_Channel;

    // Configure the Tx DMA
    DMA_DeInit(dev->Tx_DMA_Stream);
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = UART_TX_BUFFER_SIZE;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(dev->tx_buffer_);
    DMA_Init(dev->Tx_DMA_Stream, &DMA_InitStructure);

    // Configure the Rx DMA
    DMA_DeInit(dev->Rx_DMA_Stream);
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = UART_RX_BUFFER_SIZE;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(dev->rx_buffer_);
    DMA_Init(dev->Rx_DMA_Stream, &DMA_InitStructure);

    // Turn on the Rx DMA Stream
    DMA_Cmd(dev->Rx_DMA_Stream, ENABLE);

    //  Hook up the DMA to the uart
    USART_DMACmd(dev->UART, USART_DMAReq_Tx, ENABLE);
    USART_DMACmd(dev->UART, USART_DMAReq_Rx, ENABLE);

    // Turn on the transfer complete interrupt source from the DMA
    DMA_ITConfig(dev->Tx_DMA_Stream, DMA_IT_TC, ENABLE);
    DMA_ITConfig(dev->Rx_DMA_Stream, DMA_IT_TC, ENABLE);

    // Initialize the Circular Buffers
    // set the buffer pointers to where the DMA is starting (starts at 256 and counts down)
    dev->rx_buffer_tail_ = DMA_GetCurrDataCounter(dev->Rx_DMA_Stream);
    dev->rx_buffer_head_ = dev->rx_buffer_tail_;
    dev->tx_buffer_head_ = 0;
    dev->tx_buffer_tail_ = 0;

    memset(dev->rx_buffer_, 0, UART_RX_BUFFER_SIZE);
    memset(dev->tx_buffer_, 0, UART_TX_BUFFER_SIZE);


    // Configure the Interrupt
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = dev->USART_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = dev->Tx_DMA_IRQn;
    NVIC_Init(&NVIC_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = dev->Rx_DMA_IRQn;
    NVIC_Init(&NVIC_InitStruct);


}


void uart_write(UART_dev* dev, const uint8_t* ch, uint8_t len)
{
    // Put Data on the tx_buffer
    for (int i = 0; i < len; i++)
    {
        dev->tx_buffer_[dev->tx_buffer_head_] = ch[i];
        dev->tx_buffer_head_ = (dev->tx_buffer_head_ + 1) % UART_TX_BUFFER_SIZE;
    }
    if (DMA_GetCmdStatus(dev->Tx_DMA_Stream) == DISABLE)
    {
        uart_start_dma(dev);
    }
    //  this->flush();//testing
}

void uart_start_dma(UART_dev* dev)
{
    // Set the start of the transmission to the oldest data
    dev->Tx_DMA_Stream->M0AR = (uint32_t)(&dev->tx_buffer_[dev->tx_buffer_tail_]);
    if (dev->tx_buffer_head_ > dev->tx_buffer_tail_)
    {
        // Set the length of the transmission to the data on the buffer
        // if contiguous, this is easy
        DMA_SetCurrDataCounter(dev->Tx_DMA_Stream, dev->tx_buffer_head_ - dev->tx_buffer_tail_);
        dev->tx_buffer_tail_ = dev->tx_buffer_head_;
    }
    else
    {
        // We will have to send the data in two groups, first the tail,
        // then the head we will do later
        DMA_SetCurrDataCounter(dev->Tx_DMA_Stream, UART_TX_BUFFER_SIZE - dev->tx_buffer_tail_);
        dev->tx_buffer_tail_ = 0;
    }
    // Start the Transmission
    DMA_Cmd(dev->Tx_DMA_Stream, ENABLE);
}

uint8_t uart_read_byte(UART_dev* dev)
{
    uint8_t byte = 0;
    // pull the next byte off the array
    // (the head counts down, because CNTR counts down)
    if (dev->rx_buffer_head_ != dev->rx_buffer_tail_)
    {
        // read a new byte and decrement the tail
        byte = dev->rx_buffer_[UART_RX_BUFFER_SIZE - dev->rx_buffer_tail_];
        if (--dev->rx_buffer_tail_ == 0)
        {
            // wrap to the top if at the bottom
            dev->rx_buffer_tail_ = UART_RX_BUFFER_SIZE;
        }
    }
    return byte;
}

void uart_put_byte(UART_dev* dev, uint8_t byte)
{
    uart_write(dev, &byte, 1);
}

uint32_t uart_rx_bytes_waiting(UART_dev* dev)
{
    // Remember, the DMA CNDTR counts down
    dev->rx_buffer_head_ = DMA_GetCurrDataCounter(dev->Rx_DMA_Stream);
    if (dev->rx_buffer_head_ < dev->rx_buffer_tail_)
    {
        // Easy, becasue it's contiguous
        return dev->rx_buffer_tail_ - dev->rx_buffer_head_;
    }
    else if (dev->rx_buffer_head_ > dev->rx_buffer_tail_)
    {
        // Add the parts on either end of the buffer
        // I'm pretty sure this is wrong
        return dev->rx_buffer_tail_ + UART_RX_BUFFER_SIZE - dev->rx_buffer_head_;
    }
    else
    {
        return 0;
    }
}

uint32_t uart_tx_bytes_free(UART_dev* dev)
{
    // Remember, the DMA CNDTR counts down
    dev->tx_buffer_head_ = DMA_GetCurrDataCounter(dev->Rx_DMA_Stream);
    if (dev->tx_buffer_head_ >= dev->tx_buffer_tail_)
    {
        return UART_TX_BUFFER_SIZE - (dev->tx_buffer_head_ - dev->tx_buffer_tail_);
    }
    else
        return dev->tx_buffer_tail_ - dev->rx_buffer_head_;
}

bool uart_set_mode(UART_dev* dev, uint32_t baud, UART_Mode mode)
{
    USART_Cmd(dev->UART, DISABLE); // Disable the usart device for configuration

    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = dev->baudrate;

    switch (mode)
    {
        case MODE_8N1:
        default:
            USART_InitStruct.USART_WordLength = USART_WordLength_8b;
            USART_InitStruct.USART_Parity = USART_Parity_No;
            USART_InitStruct.USART_StopBits = USART_StopBits_1;
            break;
        case MODE_8E2:
            USART_InitStruct.USART_WordLength = USART_WordLength_8b;
            USART_InitStruct.USART_Parity = USART_Parity_Even;
            USART_InitStruct.USART_StopBits = USART_StopBits_2;
            break;
    }

    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // Set to both receive and send
    USART_Init(dev->UART, &USART_InitStruct);
    // USART_OverSampling8Cmd(dev->dev, ENABLE);//Please don't break anything

    // Throw interrupts on byte receive
    USART_ITConfig(dev->UART, USART_IT_RXNE, ENABLE); // enable interupts on receive
    USART_Cmd(dev->UART, ENABLE);                     // reenable the usart

    return true;
}

bool uart_tx_buffer_empty(UART_dev* dev)
{
    return dev->tx_buffer_head_ == dev->tx_buffer_tail_;
}
//
//bool UART::flush()
//{
//    uint32_t timeout = 100000;
//    while (!tx_buffer_empty() && --timeout)
//        ;
//    if (timeout)
//        return true;
//    else
//        return false;
//}
//
void UART_DMA_Rx_IRQ_callback(UART_dev* dev)
{
    // DMA took care of putting the data on the buffer
    // Just call the callback until we have no more data
    // Update the head position from the DMA
    dev->rx_buffer_head_ = DMA_GetCurrDataCounter(dev->Rx_DMA_Stream);
    if (dev->rx_cb != NULL)
    {
        while (dev->rx_buffer_head_ != dev->rx_buffer_tail_)
        {
            // read a new byte and decrement the tail
            uint8_t byte = dev->rx_buffer_[UART_RX_BUFFER_SIZE - dev->rx_buffer_tail_];
            dev->rx_cb(byte);
            if (--dev->rx_buffer_tail_ == 0)
            {
                // wrap to the top if at the bottom
                dev->rx_buffer_tail_ = UART_RX_BUFFER_SIZE;
            }
        }
    }
}

void UART_DMA_Tx_IRQ_callback(UART_dev* dev)
{
    // If there is more data to be sent
    if (dev->tx_buffer_head_ != dev->tx_buffer_tail_)
    {
        uart_start_dma(dev);
    }
}

void uart_register_rx_callback(UART_dev* dev, void (*cb)(uint8_t data))
{
    dev->rx_cb = cb;
}

void uart_unregister_rx_callback(UART_dev* dev)
{
    dev->rx_cb = NULL;
}

void USART1_IRQHandler(void) { UART_DMA_Rx_IRQ_callback(NULL); }
void USART3_IRQHandler(void) { UART_DMA_Rx_IRQ_callback(NULL); }

void DMA2_Stream5_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA2_Stream5, DMA_IT_TCIF5))
    {
        DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);
        UART_DMA_Rx_IRQ_callback(NULL);
    }
}

void DMA2_Stream7_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7))
    {
        DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
        DMA_Cmd(DMA2_Stream7, DISABLE);
        UART_DMA_Tx_IRQ_callback(NULL);
    }
}

void DMA1_Stream1_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1))
    {
        DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
        UART_DMA_Rx_IRQ_callback(NULL);
    }
}

void DMA1_Stream3_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3))
    {
        DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
        DMA_Cmd(DMA1_Stream3, DISABLE);
        UART_DMA_Tx_IRQ_callback(NULL);
    }
}
