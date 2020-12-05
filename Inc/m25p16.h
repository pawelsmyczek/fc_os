//
// Created by pablito on 24.05.2020.
//

#ifndef FC_SOFT_M25P16_H
#define FC_SOFT_M25P16_H
#include "spi.h"
#include "setup.h"

#define WRITE_ENABLE                 0x06
#define WRITE_DISABLE                0x04
#define READ_IDENTIFICATION          0x9F
#define READ_IDENTIFICATION2         0x9E
#define READ_STATUS                  0x05
#define WRITE_STATUS                 0x01
#define READ_DATA                    0x03
#define READ_DATA_HIGH_SPEED         0x0B
#define PAGE_PROGRAM                 0x02
#define SECTOR_ERASE                 0xD8
#define BULK_ERASE                   0xC7
#define DEEP_POWER_DOWN              0xB9
#define RELEASE_DEEP_POWER_DOWN      0xAB
#define STATUS_WEL_BIT               0x02
#define STATUS_WIP_BIT               0x01
#define STATUS_BLOCK_PROTECT_BITS    0x1C
#define STATUS_SRWD_BIT              0x80

#define SECTOR_ERASE_TIMEOUT_MILLIS  5000

//static uint32_t current_page;
//static uint32_t current_position;
//static uint32_t config_size;


class M25P16
{
    M25P16(const M25P16&) = delete;
    const M25P16& operator=(const M25P16&) = delete;
public:

    M25P16(SPI* spi) noexcept;
    ~M25P16() noexcept;

    void init_m25p16(void);
    uint8_t get_status(void);
    void read_config(uint8_t *data, uint32_t len, uint8_t addr_start);
    bool write_config(const uint8_t *data, const uint32_t len);
    void write_page(uint8_t* data);
    void read_mem(uint8_t* data, uint8_t len);
private:
    SPI* spi;
    uint32_t current_page;
    uint32_t current_position;
    uint32_t config_size;
    uint32_t num_pages_for_config;
};



//void init_m25p16(void);
//void read_config(uint8_t *data, uint32_t len, uint8_t addr_start);
//bool write_config(const uint8_t *data, const uint32_t len);
//void write_page(uint8_t* data);
//void read_mem(uint8_t* data, uint8_t len);

#endif //FC_SOFT_M25P16_H
