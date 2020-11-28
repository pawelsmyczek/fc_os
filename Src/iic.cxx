//
// Created by pablito on 24.05.2020.
//

#include "iic.h"

uint16_t hardware_errors = 0;
uint64_t last_event;

#define while_check(cond, result)                \
  {                                              \
    int32_t timeout_var = 400;                   \
    while ((cond) && timeout_var) timeout_var--; \
    if (!timeout_var)                            \
    {                                            \
      handle_hardware_failure(&BMP180);          \
      result = RESULT_ERROR;                     \
    }                                            \
  }

#define while_check_new(cond, result)                \
  {                                              \
    int32_t timeout_var = 400;                   \
    while ((cond) && timeout_var) timeout_var--; \
    if (!timeout_var)                            \
    {                                            \
      handle_hardware_failure();          \
      result = RESULT_ERROR;                     \
    }                                            \
  }

I2C* i2c1 = nullptr;



I2C::I2C(I2C_Dev_* dev_) noexcept
: dev(dev_)
{
    delay_us(100);

    I2C_SoftwareResetCmd(dev->I2C, ENABLE);
    I2C_SoftwareResetCmd(dev->I2C, DISABLE);

    I2C_DeInit(dev->I2C);

    I2C_InitTypeDef I2C_InitStructure;
    I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_ClockSpeed = dev->I2C_ClockSpeed;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0; // The first device address as in master mode it is
    I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(dev->I2C, &I2C_InitStructure);

    // Interrupts
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannel = dev->I2C_EV_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    // I2C Event Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = dev->I2C_ER_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    // DMA Event Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = dev->DMA_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    DMA_Cmd(dev->DMA_Stream, DISABLE);
    DMA_DeInit(dev->DMA_Stream);
    dma.DMA_FIFOMode = DMA_FIFOMode_Enable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Channel = dev->DMA_Channel;

    dma.DMA_PeripheralBaseAddr = (uint32_t)(&(dev->I2C->DR));
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;

    last_event = micros();
    I2C_Cmd(dev->I2C, ENABLE);
    unstick();

    initialised = true;

}
I2C::~I2C() noexcept
{

}

void I2C::unstick(){
    GPIO_InitTypeDef GPIO_InitStruct;
    I2C_Cmd(dev->I2C, DISABLE);
    I2C_ClearFlag(dev->I2C, I2C_FLAG_BUSY);
    // Turn off the interrupts
    I2C_ITConfig(dev->I2C, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
    // reset errors
    I2C_ClearFlag(dev->I2C, I2C_SR1_OVR | I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR);

    /*set output modes for pins*/

    GPIO_InitStructure.GPIO_Pin = dev->SCL_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = dev->SDA_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB, dev->SCL_Pin);
    GPIO_SetBits(GPIOB, dev->SDA_Pin);
    delay_us(100);

    // clock out some bits from SCL
    for (int i = 0; i < 16; ++i)
    {
        delay_us(1);
        if (GPIO_ReadOutputDataBit(GPIOB, dev->SCL_Pin))
            GPIO_ResetBits(GPIOB, dev->SCL_Pin);
        else
            GPIO_SetBits(GPIOB, dev->SCL_Pin);
    }
    delay_us(1);

    // send a start condition
    GPIO_ResetBits(GPIOB, dev->SDA_Pin);
    delay_us(1);
    GPIO_ResetBits(GPIOB, dev->SCL_Pin);
    delay_us(1);

    // then a stop
    GPIO_SetBits(GPIOB, dev->SDA_Pin);
    delay_us(1);
    GPIO_SetBits(GPIOB, dev->SCL_Pin);
    delay_us(1);

    // turn things back on
    GPIO_InitStructure.GPIO_Pin = dev->SCL_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = dev->SDA_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    I2C_Cmd(dev->I2C, ENABLE);

    current_status = IDLE;
    // write_data(dev, 0, 0, 0);

    last_event = micros();
}
bool I2C::check_busy(){
    GPIO_InitTypeDef GPIO_InitStruct;
    if (current_status == IDLE)
        return false;
    else {
        // If we haven't seen anything in a long while, then restart the device
        if (micros() > last_event + 2000) {
            hardware_errors++;
            // Send a stop condition
            I2C_GenerateSTOP(dev->I2C, ENABLE);
            return_code = RESULT_SUCCESS;
            while_check_new(dev->I2C->SR2 & BUSY, return_code)

            // Force reset of the bus
            // This is really slow, but it seems to be the only
            // way to regain a connection if bad things happen
            if (return_code == RESULT_ERROR) {

                I2C_Cmd(dev->I2C, DISABLE);
                GPIO_InitStructure.GPIO_Pin = dev->SCL_Pin;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
                GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
                GPIO_Init(GPIOB, &GPIO_InitStructure);

                GPIO_InitStructure.GPIO_Pin = dev->SDA_Pin;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
                GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
                GPIO_Init(GPIOB, &GPIO_InitStructure);

                // send a start condition
                GPIO_ResetBits(GPIOB, dev->SCL_Pin);
                delay_us(1);
                GPIO_ResetBits(GPIOB, dev->SDA_Pin);
                delay_us(1);

                // then a stop
                GPIO_SetBits(GPIOB, dev->SCL_Pin);
                delay_us(1);
                GPIO_SetBits(GPIOB, dev->SDA_Pin);
                delay_us(1);

                // turn things back on
                GPIO_InitStructure.GPIO_Pin = dev->SCL_Pin;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
                GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
                GPIO_Init(GPIOB, &GPIO_InitStructure);

                GPIO_InitStructure.GPIO_Pin = dev->SDA_Pin;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
                GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
                GPIO_Init(GPIOB, &GPIO_InitStructure);
                I2C_Cmd(dev->I2C, ENABLE);

                current_status = IDLE;
            }
            return false;
        }
        else {
            return true;
        }
    }
}

void I2C::handle_hardware_failure()
{
    hardware_errors++;
    unstick();
    return_code = RESULT_ERROR;
}

bool I2C::is_initialised() { return initialised; }

int8_t I2C::read_data_async(uint8_t _addr,
                       uint8_t _reg,
                       uint8_t number_of_bytes,
                       uint8_t *data,
                       void (*callback)(uint8_t),
                       bool is_blocking)
{
    if (check_busy())
        return RESULT_BUSY;
    //log_line;

    // configure the job
    current_status = READING;
    address = _addr;
    callback = callback;
    reg = _reg;
    subaddress_sent = (reg == 0xFF);
    length = number_of_bytes;
    done = false;
    data = data;
    return_code = RESULT_SUCCESS;

    DMA_DeInit(dev->DMA_Stream);
    dma.DMA_BufferSize = (uint16_t)(length);
    dma.DMA_Memory0BaseAddr = (uint32_t)(data);
    DMA_Init(dev->DMA_Stream, &dma);

    I2C_Cmd(dev->I2C, ENABLE);

    while_check_new(I2C_GetFlagStatus(dev->I2C, I2C_FLAG_BUSY), return_code);

    // If we don't need to send the subaddress, then go ahead and spool up the DMA NACK
    if (subaddress_sent) {
        I2C_AcknowledgeConfig(dev->I2C, ENABLE);
        I2C_DMALastTransferCmd(dev->I2C, ENABLE);
    }
    I2C_GenerateSTART(dev->I2C, ENABLE);

    I2C_ITConfig(dev->I2C, I2C_IT_EVT | I2C_IT_ERR, ENABLE);

    last_event = micros();
    if (is_blocking)
    {
        while (check_busy());
    }
    last_event = micros();

    return return_code;
}


int8_t I2C::read_data(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t length){
    if (check_busy())
        return RESULT_BUSY;
    // log_line;
    return_code = RESULT_SUCCESS;

    // Turn off interrupts while blocking
    I2C_ITConfig(dev->I2C, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
    while_check_new(I2C_GetFlagStatus(dev->I2C, I2C_FLAG_BUSY), return_code);

    I2C_Cmd(dev->I2C, ENABLE);
    if (reg != 0xFF)
    {
        // log_line;
        I2C_GenerateSTART(dev->I2C, ENABLE);
        while_check_new(!I2C_CheckEvent(dev->I2C, I2C_EVENT_MASTER_MODE_SELECT), return_code)
        I2C_Send7bitAddress(dev->I2C, addr, I2C_Direction_Transmitter);
        uint32_t timeout = 500;
        while (!I2C_CheckEvent(dev->I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && !(I2C_GetLastEvent(dev->I2C) & AF) && timeout--);

        // No acknowledgement or timeout
        if (I2C_GetLastEvent(dev->I2C) & AF || timeout == 0)
        {
//        log_line;
            I2C_GenerateSTOP(dev->I2C, ENABLE);
            I2C_Cmd(dev->I2C, DISABLE);
            return RESULT_ERROR;
        }
        I2C_Cmd(dev->I2C, ENABLE);
        I2C_SendData(dev->I2C, reg);
        while_check_new(!I2C_CheckEvent(dev->I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED), return_code);
    }

    // Read bytes
    I2C_GenerateSTART(dev->I2C, ENABLE);
    while_check_new(!I2C_CheckEvent(dev->I2C, I2C_EVENT_MASTER_MODE_SELECT), return_code);
    //I2C_Cmd(dev->I2C, ENABLE);
    I2C_AcknowledgeConfig(dev->I2C, ENABLE);
    I2C_Send7bitAddress(dev->I2C, addr, I2C_Direction_Receiver);
    while_check_new(!I2C_CheckEvent(dev->I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED), return_code)
    while(length--){
        if(length > 0){
            while (!I2C_CheckEvent(dev->I2C, I2C_EVENT_MASTER_BYTE_RECEIVED));
            *data = I2C_ReceiveData(dev->I2C);
            data++;
        }
        if(length == 0){
            I2C_AcknowledgeConfig(dev->I2C, DISABLE);
            I2C_GenerateSTOP(dev->I2C, ENABLE);
            while (!I2C_CheckEvent(dev->I2C, I2C_EVENT_MASTER_BYTE_RECEIVED));
            *data = I2C_ReceiveData(dev->I2C);
        }
    }
    I2C_Cmd(dev->I2C, DISABLE);
    // log_line;

    return return_code;

}


int8_t I2C::write_data_async(uint8_t addr, uint8_t reg, uint8_t* data, void (*callback)(uint8_t), bool is_blocking)
{
    if (check_busy())
        return RESULT_BUSY;

    //log_line;
    current_status = WRITING;
    address = addr;
    callback = callback;
    reg = reg;
    subaddress_sent = (reg == 0xFF);
    length = 1;
    done = false;
    data = data;
    return_code = RESULT_SUCCESS;

    I2C_Cmd(dev->I2C, ENABLE);

    while_check_new(I2C_GetFlagStatus(dev->I2C, I2C_FLAG_BUSY), return_code);

    I2C_GenerateSTART(dev->I2C, ENABLE);

    I2C_ITConfig(dev->I2C, I2C_IT_EVT | I2C_IT_ERR, ENABLE);

    last_event = micros();
    if (is_blocking)
    {
        //log_line;
        while (check_busy())
            ;
    }
    //log_line;
    return return_code;
}


int8_t I2C::write_data(uint8_t addr, uint8_t reg, uint8_t* data){
    if (check_busy())
        return RESULT_BUSY;

    //log_line;
    return_code = RESULT_SUCCESS;
    while_check_new(I2C_GetFlagStatus(dev->I2C, I2C_FLAG_BUSY), return_code);

    // Turn off interrupts for is_blocking write
    I2C_ITConfig(dev->I2C, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
    I2C_Cmd(dev->I2C, ENABLE);

    // start the transfer
    I2C_GenerateSTART(dev->I2C, ENABLE);
    while_check_new(!I2C_CheckEvent(dev->I2C, I2C_EVENT_MASTER_MODE_SELECT), return_code);
    I2C_Send7bitAddress(dev->I2C, addr, I2C_Direction_Transmitter);
    uint32_t timeout = 500;
    while (!I2C_CheckEvent(dev->I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && !(I2C_GetLastEvent(dev->I2C) & AF) && timeout--);

    // No acknowledgement or timeout
    if (I2C_GetLastEvent(dev->I2C) & AF || timeout == 0)
    {
//        log_line;
        I2C_GenerateSTOP(dev->I2C, ENABLE);
        I2C_Cmd(dev->I2C, DISABLE);
        return RESULT_ERROR;
    }

    // Send the register
    if (reg != 0xFF)
    {
//        log_line;
        I2C_SendData(dev->I2C, reg);
//        while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS);
        while_check_new(!I2C_CheckEvent(dev->I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED), return_code);
    }

    // Write the byte with a NACK
    I2C_AcknowledgeConfig(dev->I2C, DISABLE);
    I2C_SendData(dev->I2C, *data);
    while_check_new(!I2C_CheckEvent(dev->I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED), return_code);
    I2C_GenerateSTOP(dev->I2C, ENABLE);
    I2C_Cmd(dev->I2C, DISABLE);
//    log_line;
    last_event = micros();
    return return_code;
}

void I2C::handle_error(){
    // log_line;
    I2C_Cmd(dev->I2C, DISABLE);
    return_code = RESULT_ERROR;
    while_check_new(I2C_GetFlagStatus(dev->I2C, I2C_FLAG_BUSY), return_code);

    // Turn off the interrupts
    I2C_ITConfig(dev->I2C, I2C_IT_EVT | I2C_IT_ERR, DISABLE);

    // reset errors
    I2C_ClearFlag(dev->I2C, I2C_SR1_OVR | I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR);
    current_status = IDLE;
    //log_line;
    bmp180_transfer_complete_cb();
}

// This is the I2C_IT_EV handler
void I2C::handle_event(){
    uint32_t last_event = I2C_GetLastEvent(dev->I2C);
    // interrupt_history_.add_event(dev->I2C->SR2 << 16 | dev->I2C->SR1);
    // We just sent a byte
    if ((last_event & I2C_EVENT_MASTER_BYTE_TRANSMITTED) == I2C_EVENT_MASTER_BYTE_TRANSMITTED)
    {
        last_event = micros();
        // If we are reading, then we just sent a subaddress and need to send
        // a repeated start, and enable the DMA NACK
        if (current_status == READING)
        {
            //log_line;
            I2C_AcknowledgeConfig(dev->I2C, ENABLE);
            I2C_DMALastTransferCmd(dev->I2C, ENABLE);
            I2C_GenerateSTART(dev->I2C, ENABLE);
        }
            // We are in write mode and are done, need to clean up
        else
        {
            //log_line;
            I2C_GenerateSTOP(dev->I2C, ENABLE);
            I2C_ITConfig(dev->I2C, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
            bmp180_transfer_complete_cb();
        }
    }

        // We just sent the address in write mode
    else if ((last_event & I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)
    {
        last_event = micros();
        // We need to send the subaddress
        if (!subaddress_sent)
        {
            // log_line;
            I2C_SendData(dev->I2C, reg);
            subaddress_sent = true;
            if (current_status == WRITING)
            {
                // log_line;
                I2C_SendData(dev->I2C, *data);
                done = true;
            }
        }
            // We need to send our data (no subaddress)
        else
        {
            // log_line;
            I2C_SendData(dev->I2C, *data);
            done = true;
        }
    }

        // We are in receiving mode, preparing to receive the big DMA dump
    else if ((last_event & I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)
    {
        last_event = micros();
        // log_line;
        //    I2C_ITConfig(dev->I2C, I2C_IT_EVT, DISABLE);
        DMA_SetCurrDataCounter(dev->DMA_Stream, length);
        I2C_DMACmd(dev->I2C, ENABLE);
        DMA_ITConfig(dev->DMA_Stream, DMA_IT_TC, ENABLE);
        DMA_Cmd(dev->DMA_Stream, ENABLE);
    }

        // Start just sent
    else if ((last_event & I2C_EVENT_MASTER_MODE_SELECT) == I2C_EVENT_MASTER_MODE_SELECT)
    {
        last_event = micros();
        // we either don't need to send, or already sent the subaddress
        if (subaddress_sent && current_status == READING)
        {
            // log_line;
            // Set up a receive
            I2C_Send7bitAddress(dev->I2C, address, I2C_Direction_Receiver);
        }
            // We need to either send the subaddress or our datas
        else
        {
            // log_line;
            // Set up a write
            I2C_Send7bitAddress(dev->I2C, address, I2C_Direction_Transmitter);
        }
    }

        // Sometimes we just get this over and over and over again
    else if (last_event == SB)
    {
        // SB is cleared by clearing and resetting PE
        I2C_Cmd(dev->I2C, DISABLE);
        I2C_Cmd(dev->I2C, ENABLE);
        // error_count_++;
    }
}


void i2c_init(I2C_Dev* dev, I2C_TypeDef*        i2c,
                            uint16_t            sclPin,
                            uint16_t            sdaPin,
                            uint32_t            i2cClockSpeed,
                            IRQn_Type           i2cEV_IRQn,
                            IRQn_Type           i2cER_IRQn,
                            DMA_Stream_TypeDef* dmaStream,
                            uint32_t            dmaChannel,
                            IRQn_Type           dmaIRQn,
                            uint32_t            dmaTCIF,
                            bool                is_initialised){
    delay_us(100);
    DMA_InitTypeDef           dma;
    dev->I2C                = i2c;
    dev->SCL_Pin            = sclPin;
    dev->SDA_Pin            = sdaPin;
    dev->DMA_InitStructure  = &dma;
    dev->I2C_ClockSpeed     = i2cClockSpeed;
    dev->I2C_EV_IRQn        = i2cEV_IRQn;
    dev->I2C_ER_IRQn        = i2cER_IRQn;
    dev->DMA_Stream         = dmaStream;
    dev->DMA_Channel        = dmaChannel;
    dev->DMA_IRQn           = dmaIRQn;
    dev->DMA_TCIF           = dmaTCIF;
    dev->initialised        = is_initialised;

    I2C_SoftwareResetCmd(dev->I2C, ENABLE);
    I2C_SoftwareResetCmd(dev->I2C, DISABLE);

    I2C_DeInit(dev->I2C);

    I2C_InitTypeDef I2C_InitStructure;
    I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_ClockSpeed = dev->I2C_ClockSpeed;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0; // The first device address as in master mode it is
    I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(dev->I2C, &I2C_InitStructure);

    // Interrupts
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannel = dev->I2C_EV_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    // I2C Event Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = dev->I2C_ER_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    // DMA Event Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = dev->DMA_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    DMA_Cmd(dev->DMA_Stream, DISABLE);
    DMA_DeInit(dev->DMA_Stream);
    dev->DMA_InitStructure->DMA_FIFOMode = DMA_FIFOMode_Enable;
    dev->DMA_InitStructure->DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    dev->DMA_InitStructure->DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dev->DMA_InitStructure->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dev->DMA_InitStructure->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dev->DMA_InitStructure->DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dev->DMA_InitStructure->DMA_MemoryInc = DMA_MemoryInc_Enable;
    dev->DMA_InitStructure->DMA_Mode = DMA_Mode_Normal;
    dev->DMA_InitStructure->DMA_Channel = dev->DMA_Channel;

    dev->DMA_InitStructure->DMA_PeripheralBaseAddr = (uint32_t)(&(i2c->DR));
    dev->DMA_InitStructure->DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    dev->DMA_InitStructure->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dev->DMA_InitStructure->DMA_Priority = DMA_Priority_High;
    dev->DMA_InitStructure->DMA_DIR = DMA_DIR_PeripheralToMemory;

    last_event = micros();
    I2C_Cmd(dev->I2C, ENABLE);
    // unstick(dev);

    dev->initialised = is_initialised;

}
void unstick(I2C_Dev* dev){
    GPIO_InitTypeDef GPIO_InitStruct;
    I2C_Cmd(dev->I2C, DISABLE);
    I2C_ClearFlag(dev->I2C, I2C_FLAG_BUSY);
    // Turn off the interrupts
    I2C_ITConfig(dev->I2C, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
    // reset errors
    I2C_ClearFlag(dev->I2C, I2C_SR1_OVR | I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR);

    /*set output modes for pins*/

    GPIO_InitStructure.GPIO_Pin = dev->SCL_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = dev->SDA_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB, dev->SCL_Pin);
    GPIO_SetBits(GPIOB, dev->SDA_Pin);
    delay_us(100);

    // clock out some bits from SCL
    for (int i = 0; i < 16; ++i)
    {
        delay_us(1);
        if (GPIO_ReadOutputDataBit(GPIOB, dev->SCL_Pin))
            GPIO_ResetBits(GPIOB, dev->SCL_Pin);
        else
            GPIO_SetBits(GPIOB, dev->SCL_Pin);
    }
    delay_us(1);

    // send a start condition
    GPIO_ResetBits(GPIOB, dev->SDA_Pin);
    delay_us(1);
    GPIO_ResetBits(GPIOB, dev->SCL_Pin);
    delay_us(1);

    // then a stop
    GPIO_SetBits(GPIOB, dev->SDA_Pin);
    delay_us(1);
    GPIO_SetBits(GPIOB, dev->SCL_Pin);
    delay_us(1);

    // turn things back on
    GPIO_InitStructure.GPIO_Pin = dev->SCL_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = dev->SDA_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    I2C_Cmd(dev->I2C, ENABLE);

    dev->current_status = IDLE;
    // write_data(dev, 0, 0, 0);

    last_event = micros();
}
bool check_busy(I2C_Dev* dev){
    GPIO_InitTypeDef GPIO_InitStruct;
    if (dev->current_status == IDLE)
        return false;
    else {
        // If we haven't seen anything in a long while, then restart the device
        if (micros() > last_event + 2000) {
            hardware_errors++;
            // Send a stop condition
            I2C_GenerateSTOP(dev->I2C, ENABLE);
            dev->return_code = RESULT_SUCCESS;
            while_check(dev->I2C->SR2 & BUSY, dev->return_code)

            // Force reset of the bus
            // This is really slow, but it seems to be the only
            // way to regain a connection if bad things happen
            if (dev->return_code == RESULT_ERROR) {

                I2C_Cmd(dev->I2C, DISABLE);
                GPIO_InitStructure.GPIO_Pin = dev->SCL_Pin;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
                GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
                GPIO_Init(GPIOB, &GPIO_InitStructure);

                GPIO_InitStructure.GPIO_Pin = dev->SDA_Pin;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
                GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
                GPIO_Init(GPIOB, &GPIO_InitStructure);

                // send a start condition
                GPIO_ResetBits(GPIOB, dev->SCL_Pin);
                delay_us(1);
                GPIO_ResetBits(GPIOB, dev->SDA_Pin);
                delay_us(1);

                // then a stop
                GPIO_SetBits(GPIOB, dev->SCL_Pin);
                delay_us(1);
                GPIO_SetBits(GPIOB, dev->SDA_Pin);
                delay_us(1);

                // turn things back on
                GPIO_InitStructure.GPIO_Pin = dev->SCL_Pin;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
                GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
                GPIO_Init(GPIOB, &GPIO_InitStructure);

                GPIO_InitStructure.GPIO_Pin = dev->SDA_Pin;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
                GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
                GPIO_Init(GPIOB, &GPIO_InitStructure);
                I2C_Cmd(dev->I2C, ENABLE);

                dev->current_status = IDLE;
            }
            return false;
        }
        else {
            return true;
        }
    }
}

void handle_hardware_failure(I2C_Dev* dev){
    hardware_errors++;
    unstick(dev);
    dev->return_code = RESULT_ERROR;
}

bool is_initialised(I2C_Dev* dev) { return dev->initialised; }

int8_t read_data_async(I2C_Dev* dev, uint8_t _addr,
                       uint8_t _reg,
                       uint8_t number_of_bytes,
                       uint8_t *data,
                       void (*callback)(uint8_t),
                       bool is_blocking) {
    if (check_busy(dev))
        return RESULT_BUSY;
    //log_line;

    // configure the job
    dev->current_status = READING;
    dev->address = _addr;
    dev->callback = callback;
    dev->reg = _reg;
    dev->subaddress_sent = (dev->reg == 0xFF);
    dev->length = number_of_bytes;
    dev->done = false;
    dev->data = data;
    dev->return_code = RESULT_SUCCESS;

    DMA_DeInit(dev->DMA_Stream);
    dev->DMA_InitStructure->DMA_BufferSize = (uint32_t)(dev->length);
    dev->DMA_InitStructure->DMA_Memory0BaseAddr = (uint32_t)(dev->data);
    DMA_Init(dev->DMA_Stream, dev->DMA_InitStructure);

    I2C_Cmd(dev->I2C, ENABLE);

    while_check(I2C_GetFlagStatus(dev->I2C, I2C_FLAG_BUSY), dev->return_code);

    // If we don't need to send the subaddress, then go ahead and spool up the DMA NACK
    if (dev->subaddress_sent) {
        I2C_AcknowledgeConfig(dev->I2C, ENABLE);
        I2C_DMALastTransferCmd(dev->I2C, ENABLE);
    }
    I2C_GenerateSTART(dev->I2C, ENABLE);

    I2C_ITConfig(dev->I2C, I2C_IT_EVT | I2C_IT_ERR, ENABLE);

    last_event = micros();
    if (is_blocking)
    {
        while (check_busy(dev))
            ;
    }
    last_event = micros();

    return dev->return_code;
}

void bmp180_transfer_complete_cb(){
    BMP180.current_status = IDLE;
    if (BMP180.callback)
        BMP180.callback(BMP180.return_code);
}

int8_t read_data(I2C_Dev* dev, uint8_t addr, uint8_t reg, uint8_t* data, uint8_t length){
    if (check_busy(dev))
        return RESULT_BUSY;
    // log_line;
    dev->return_code = RESULT_SUCCESS;

    // Turn off interrupts while blocking
    I2C_ITConfig(dev->I2C, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
    while_check(I2C_GetFlagStatus(dev->I2C, I2C_FLAG_BUSY), dev->return_code);

    I2C_Cmd(dev->I2C, ENABLE);
    if (reg != 0xFF)
    {
        // log_line;
        I2C_GenerateSTART(dev->I2C, ENABLE);
        while_check(!I2C_CheckEvent(dev->I2C, I2C_EVENT_MASTER_MODE_SELECT), dev->return_code)
        I2C_Send7bitAddress(dev->I2C, addr, I2C_Direction_Transmitter);
        uint32_t timeout = 500;
        while (!I2C_CheckEvent(dev->I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && !(I2C_GetLastEvent(dev->I2C) & AF) && timeout--);

        // No acknowledgement or timeout
        if (I2C_GetLastEvent(dev->I2C) & AF || timeout == 0)
        {
//        log_line;
            I2C_GenerateSTOP(dev->I2C, ENABLE);
            I2C_Cmd(dev->I2C, DISABLE);
            return RESULT_ERROR;
        }
        I2C_Cmd(dev->I2C, ENABLE);
        I2C_SendData(dev->I2C, reg);
        while_check(!I2C_CheckEvent(dev->I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED), dev->return_code);
    }

    // Read bytes
    I2C_GenerateSTART(dev->I2C, ENABLE);
    while_check(!I2C_CheckEvent(dev->I2C, I2C_EVENT_MASTER_MODE_SELECT), dev->return_code);
    //I2C_Cmd(dev->I2C, ENABLE);
    I2C_AcknowledgeConfig(dev->I2C, ENABLE);
    I2C_Send7bitAddress(dev->I2C, addr, I2C_Direction_Receiver);
    while_check(!I2C_CheckEvent(dev->I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED), dev->return_code)
    while(length--){
        if(length > 0){
            while (!I2C_CheckEvent(dev->I2C, I2C_EVENT_MASTER_BYTE_RECEIVED));
            *data = I2C_ReceiveData(dev->I2C);
            data++;
        }
        if(length == 0){
            I2C_AcknowledgeConfig(dev->I2C, DISABLE);
            I2C_GenerateSTOP(dev->I2C, ENABLE);
            while (!I2C_CheckEvent(dev->I2C, I2C_EVENT_MASTER_BYTE_RECEIVED));
            *data = I2C_ReceiveData(dev->I2C);
        }
    }
    I2C_Cmd(dev->I2C, DISABLE);
    // log_line;

    return dev->return_code;

}


int8_t write_data_async(I2C_Dev* dev, uint8_t addr, uint8_t reg, uint8_t* data, void (*callback)(uint8_t), bool is_blocking)
{
    if (check_busy(dev))
        return RESULT_BUSY;

    //log_line;
    dev->current_status = WRITING;
    dev->address = addr;
    dev->callback = callback;
    dev->reg = reg;
    dev->subaddress_sent = (dev->reg == 0xFF);
    dev->length = 1;
    dev->done = false;
    dev->data = data;
    dev->return_code = RESULT_SUCCESS;

    I2C_Cmd(dev->I2C, ENABLE);

    while_check(I2C_GetFlagStatus(dev->I2C, I2C_FLAG_BUSY), dev->return_code);

    I2C_GenerateSTART(dev->I2C, ENABLE);

    I2C_ITConfig(dev->I2C, I2C_IT_EVT | I2C_IT_ERR, ENABLE);

    last_event = micros();
    if (is_blocking)
    {
        //log_line;
        while (check_busy(dev))
            ;
    }
    //log_line;
    return dev->return_code;
}


int8_t write_data(I2C_Dev* dev, uint8_t addr, uint8_t reg, uint8_t* data){
    if (check_busy(dev))
        return RESULT_BUSY;

    //log_line;
    dev->return_code = RESULT_SUCCESS;
    while_check(I2C_GetFlagStatus(dev->I2C, I2C_FLAG_BUSY), dev->return_code);

    // Turn off interrupts for is_blocking write
    I2C_ITConfig(dev->I2C, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
    I2C_Cmd(dev->I2C, ENABLE);

    // start the transfer
    I2C_GenerateSTART(dev->I2C, ENABLE);
    while_check(!I2C_CheckEvent(dev->I2C, I2C_EVENT_MASTER_MODE_SELECT), dev->return_code);
    I2C_Send7bitAddress(dev->I2C, addr, I2C_Direction_Transmitter);
    uint32_t timeout = 500;
    while (!I2C_CheckEvent(dev->I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && !(I2C_GetLastEvent(dev->I2C) & AF) && timeout--);

    // No acknowledgement or timeout
    if (I2C_GetLastEvent(dev->I2C) & AF || timeout == 0)
    {
//        log_line;
        I2C_GenerateSTOP(dev->I2C, ENABLE);
        I2C_Cmd(dev->I2C, DISABLE);
        return RESULT_ERROR;
    }

    // Send the register
    if (reg != 0xFF)
    {
//        log_line;
        I2C_SendData(dev->I2C, reg);
//        while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS);
        while_check(!I2C_CheckEvent(dev->I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED), dev->return_code);
    }

    // Write the byte with a NACK
    I2C_AcknowledgeConfig(dev->I2C, DISABLE);
    I2C_SendData(dev->I2C, *data);
    while_check(!I2C_CheckEvent(dev->I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED), dev->return_code);
    I2C_GenerateSTOP(dev->I2C, ENABLE);
    I2C_Cmd(dev->I2C, DISABLE);
//    log_line;
    last_event = micros();
    return dev->return_code;
}

void handle_error(I2C_Dev* dev){
    // log_line;
    I2C_Cmd(dev->I2C, DISABLE);
    dev->return_code = RESULT_ERROR;
    while_check(I2C_GetFlagStatus(dev->I2C, I2C_FLAG_BUSY), dev->return_code);

    // Turn off the interrupts
    I2C_ITConfig(dev->I2C, I2C_IT_EVT | I2C_IT_ERR, DISABLE);

    // reset errors
    I2C_ClearFlag(dev->I2C, I2C_SR1_OVR | I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR);
    dev->current_status = IDLE;
    //log_line;
    bmp180_transfer_complete_cb();
}

// This is the I2C_IT_EV handler
void handle_event(I2C_Dev* dev){
    uint32_t last_event = I2C_GetLastEvent(dev->I2C);
    // interrupt_history_.add_event(dev->I2C->SR2 << 16 | dev->I2C->SR1);
    // We just sent a byte
    if ((last_event & I2C_EVENT_MASTER_BYTE_TRANSMITTED) == I2C_EVENT_MASTER_BYTE_TRANSMITTED)
    {
        last_event = micros();
        // If we are reading, then we just sent a subaddress and need to send
        // a repeated start, and enable the DMA NACK
        if (dev->current_status == READING)
        {
            //log_line;
            I2C_AcknowledgeConfig(dev->I2C, ENABLE);
            I2C_DMALastTransferCmd(dev->I2C, ENABLE);
            I2C_GenerateSTART(dev->I2C, ENABLE);
        }
            // We are in write mode and are done, need to clean up
        else
        {
            //log_line;
            I2C_GenerateSTOP(dev->I2C, ENABLE);
            I2C_ITConfig(dev->I2C, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
            bmp180_transfer_complete_cb();
        }
    }

        // We just sent the address in write mode
    else if ((last_event & I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)
    {
        last_event = micros();
        // We need to send the subaddress
        if (!dev->subaddress_sent)
        {
            // log_line;
            I2C_SendData(dev->I2C, dev->reg);
            dev->subaddress_sent = true;
            if (dev->current_status == WRITING)
            {
                // log_line;
                I2C_SendData(dev->I2C, *dev->data);
                dev->done = true;
            }
        }
            // We need to send our data (no subaddress)
        else
        {
            // log_line;
            I2C_SendData(dev->I2C, *dev->data);
            dev->done = true;
        }
    }

        // We are in receiving mode, preparing to receive the big DMA dump
    else if ((last_event & I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)
    {
        last_event = micros();
        // log_line;
        //    I2C_ITConfig(dev->I2C, I2C_IT_EVT, DISABLE);
        DMA_SetCurrDataCounter(dev->DMA_Stream, dev->length);
        I2C_DMACmd(dev->I2C, ENABLE);
        DMA_ITConfig(dev->DMA_Stream, DMA_IT_TC, ENABLE);
        DMA_Cmd(dev->DMA_Stream, ENABLE);
    }

        // Start just sent
    else if ((last_event & I2C_EVENT_MASTER_MODE_SELECT) == I2C_EVENT_MASTER_MODE_SELECT)
    {
        last_event = micros();
        // we either don't need to send, or already sent the subaddress
        if (dev->subaddress_sent && dev->current_status == READING)
        {
            // log_line;
            // Set up a receive
            I2C_Send7bitAddress(dev->I2C, dev->address, I2C_Direction_Receiver);
        }
            // We need to either send the subaddress or our datas
        else
        {
            // log_line;
            // Set up a write
            I2C_Send7bitAddress(dev->I2C, dev->address, I2C_Direction_Transmitter);
        }
    }

        // Sometimes we just get this over and over and over again
    else if (last_event == SB)
    {
        // SB is cleared by clearing and resetting PE
        I2C_Cmd(dev->I2C, DISABLE);
        I2C_Cmd(dev->I2C, ENABLE);
        // error_count_++;
    }
}

extern "C"
{

    void DMA1_Stream0_IRQHandler(void)
    {
        if (DMA_GetFlagStatus(DMA1_Stream0, DMA_FLAG_TCIF0))
        {
            /* Clear transmission complete flag */
            DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);

            I2C_DMACmd(I2C1, DISABLE);
            /* Send I2C1 STOP Condition */
            I2C_GenerateSTOP(I2C1, ENABLE);
            /* Disable DMA channel*/
            DMA_Cmd(DMA1_Stream0, DISABLE);
            /* Turn off I2C interrupts, because we are done with the transfer */
            I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR, DISABLE);

            bmp180_transfer_complete_cb(); // TODO make this configurable
        }
    }
void I2C1_ER_IRQHandler(void) { handle_error(&BMP180); }

void I2C1_EV_IRQHandler(void) { handle_event(&BMP180); }
}

