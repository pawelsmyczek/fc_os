//
// Created by pablito on 24.05.2020.
//

#include "m25p16.h"

void init_m25p16(void){
    set_spi_divisor(&M25P16, 16);
}

static uint8_t get_status(void){
    spi_enable(&M25P16_CS);

    spi_transfer(&M25P16, READ_STATUS);
    uint8_t status = spi_transfer(&M25P16, 0xFF);

    spi_disable(&M25P16_CS);
    return status;
}

void read_config(uint8_t *data, uint32_t len, uint8_t addr_start){
    // READ DATA COMMAND
    // blocking sequence, not working with DMA
    uint8_t addr[4] = {READ_DATA, (uint8_t)(addr_start >> 8), (uint8_t)(addr_start & 0xFF), 0};
    spi_enable(&M25P16_CS);

    spi_transfer(&M25P16, addr[0]);
    spi_transfer(&M25P16, addr[1]);
    spi_transfer(&M25P16, addr[2]);
    spi_transfer(&M25P16, addr[3]);

    while(len--){
        *data = spi_transfer(&M25P16, NULL);
        data++;
        delay_ms(6);
    }

    spi_disable(&M25P16_CS);
}

bool write_config(const uint8_t *data, const uint32_t len){
    // Calculate the correct number of pages to store the config
    num_pages_for_config = len / 256;
    if (len % 256 != 0)
        num_pages_for_config++; // We need an extra partial page

    // Enable the write
    spi_enable(&M25P16_CS);
    spi_transfer(&M25P16, WRITE_ENABLE);
    spi_disable(&M25P16_CS);
    // Make sure we can erase (WEL bit is set)
    uint8_t status = get_status();
    if (!(status & STATUS_WEL_BIT))
        return false;

    // Erase Sector (There is really no way around this, we have to erase the entire sector
    uint8_t sector_addr[4] = {SECTOR_ERASE, 0, 0, 0};
    spi_enable(&M25P16_CS);
    spi_transfer(&M25P16, sector_addr[0]);
    spi_transfer(&M25P16, sector_addr[1]);
    spi_transfer(&M25P16, sector_addr[2]);
    spi_transfer(&M25P16, sector_addr[3]);
    spi_disable(&M25P16_CS);

    // Wait for Sector Erase to complete
    bool WIP = true;
    do {
        delay_ms(5000);
        status = get_status();
        if ((status & STATUS_WIP_BIT) == 0x00)
            WIP = false;
    } while (WIP);

    // Program the data
    for (uint32_t i = 0; i < num_pages_for_config; i++) {
        // Re-Enable the write (the WEL bit is reset after each program completion)
        spi_enable(&M25P16_CS);
        spi_transfer(&M25P16, WRITE_ENABLE);
        spi_disable(&M25P16_CS);

        // Make sure that the WEL bit has been set, so we can write
        status = get_status();

        // Figure out how much of this page we are going to use
        uint16_t page_len = 256;
        if (i == num_pages_for_config - 1)
            page_len = len % 256; // If this is last page, then we write the partial

        // Send the PAGE_PROGRAM command with the right address
        spi_enable(&M25P16_CS);
        uint8_t addr[4] = {PAGE_PROGRAM, (uint8_t)(i >> 8), (uint8_t)(i & 0xFF), 0};
        spi_transfer(&M25P16, addr[0]);
        spi_transfer(&M25P16, addr[1]);
        spi_transfer(&M25P16, addr[2]);
        spi_transfer(&M25P16, addr[3]);

        // Transfer the data
        while(page_len--){
            spi_transfer(&M25P16, *data);
            data++;
        }

        spi_disable(&M25P16_CS);

        // poll the status register to establish, whether the writing operation ended
        WIP = true;
        do
        {
            delay_ms(6);
            status = get_status();
            if ((status & STATUS_WIP_BIT) == 0x00)
                WIP = false;
        } while (WIP);
    }

    // Disable the write
    spi_enable(&M25P16_CS);
    spi_transfer(&M25P16, WRITE_DISABLE);
    spi_disable(&M25P16_CS);
    return true;
}

