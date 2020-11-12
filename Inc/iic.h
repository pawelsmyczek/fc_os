//
// Created by pablito on 24.05.2020.
//

#ifndef FC_SOFT_IIC_H
#define FC_SOFT_IIC_H
#include "board.h"

#define RESULT_ERROR        0
#define RESULT_SUCCESS      1
#define RESULT_BUSY         -1

#define        SB      0x0001
#define        ADDR    0x0002
#define        BTF     0x0004
#define        ADD10   0x0008
#define        STOPF   0x0010
#define        RES1    0x0020
#define        RXNE    0x0040
#define        TXE     0x0080
#define        BERR    0x0100
#define        ARLO    0x0200
#define        AF      0x0400
#define        OVR     0x0800
#define        PEC_ERR 0x1000
#define        RES2    0x2000
#define        TIMEOUT 0x4000
#define        SMB_ALERT = 0x8000
#define        MSL     0x1 << 16
#define        BUSY    0x2 << 16
#define        TRA     0x4 << 16
#define        RES3    0x8 << 16
#define        GEN_CALL 0x10 << 16
#define        SMBDE_FAULT 0x20 << 16
#define        DUALF   0x40 << 16
typedef enum{
    IDLE,
    READING,
    WRITING
}current_status_t;


class I2C
{
    I2C(const I2C&) = delete;
    const I2C& operator=(const I2C&) = delete;
public:
    I2C(I2C_Dev_* devv) noexcept;
    ~I2C() noexcept;
    void i2c_init(I2C_Dev* dev, I2C_TypeDef*        i2c,
                  uint16_t            SCL_Pin,
                  uint16_t            SDA_Pin,
                  DMA_InitTypeDef*    dmaInitTypeDef,
                  uint32_t            i2cClockSpeed,
                  IRQn_Type           i2cEV_IRQn,
                  IRQn_Type           i2cER_IRQn,
                  DMA_Stream_TypeDef* dmaStream,
                  uint32_t            dmaChannel,
                  IRQn_Type           dmaIRQn,
                  uint32_t            dmaTCIF,
                  bool                is_initialised);
    void unstick();
    bool check_busy();
    bool is_initialised();
    void handle_hardware_failure();
    int8_t read_data_async(uint8_t _addr,
                           uint8_t _reg,
                           uint8_t number_of_bytes,
                           uint8_t *data,
                           void (*callback)(uint8_t),
                           bool is_blocking);
    int8_t read_data(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t length);
    int8_t write_data_async(uint8_t addr,
                            uint8_t reg,
                            uint8_t* data,
                            void (*callback)(uint8_t),
                            bool is_blocking);
    int8_t write_data(uint8_t addr,
                      uint8_t reg,
                      uint8_t* data);

    void handle_error();
    void handle_event();

private:
    I2C_Dev_*           dev;
    uint8_t             return_code;
    void                (*callback)(uint8_t);
    uint8_t             address;
    uint8_t             length;
    uint8_t             reg;
    uint8_t*            data;
    bool                subaddress_sent;
    bool                done;
    bool                initialised;
    uint8_t             current_status;
    uint64_t last_event;
};

extern uint64_t last_event;

void i2c_init(I2C_Dev* dev, I2C_TypeDef*        i2c,
              uint16_t            SCL_Pin,
              uint16_t            SDA_Pin,
              DMA_InitTypeDef*    dmaInitTypeDef,
              uint32_t            i2cClockSpeed,
              IRQn_Type           i2cEV_IRQn,
              IRQn_Type           i2cER_IRQn,
              DMA_Stream_TypeDef* dmaStream,
              uint32_t            dmaChannel,
              IRQn_Type           dmaIRQn,
              uint32_t            dmaTCIF,
              bool                is_initialised);
void unstick(I2C_Dev* dev);
bool check_busy(I2C_Dev* dev);
bool is_initialised(I2C_Dev* dev);
void handle_hardware_failure(I2C_Dev* dev);
int8_t read_data_async(I2C_Dev* dev, uint8_t _addr,
                       uint8_t _reg,
                       uint8_t number_of_bytes,
                       uint8_t *data,
                       void (*callback)(uint8_t),
                       bool is_blocking);
int8_t read_data(I2C_Dev* dev, uint8_t addr, uint8_t reg, uint8_t* data, uint8_t length);
void bmp180_transfer_complete_cb();
int8_t write_data_async(I2C_Dev* dev, uint8_t addr,
                                    uint8_t reg,
                                    uint8_t* data,
                                    void (*callback)(uint8_t),
                                    bool is_blocking);
int8_t write_data(I2C_Dev* dev, uint8_t addr,
                                uint8_t reg,
                                uint8_t* data);

void handle_error(I2C_Dev* dev);
void handle_event(I2C_Dev* dev);
void DMA1_Stream0_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
void I2C1_EV_IRQHandler(void);


#endif //FC_SOFT_IIC_H
