/*
 SDClass for use by RepRapFirmware
 
<<<<<<< HEAD
 Author: ghent360
 
*/
#ifndef SDCARD_SDIO_H
#define SDCARD_SDIO_H

#include "diskio.h" //fatfs
#include "SDCard.h"
#include "bsp_driver_sd.h"
#include "sd_diskio.h"

class SDCardSDIO : public SDCard {
public:
    SDCardSDIO();
    ~SDCardSDIO();

    CARD_TYPE card_type(void) override {
        return CardInfo.CardType;
    }
    
    void unmount() override;

    uint32_t interface_speed() override {
        return 1000000;
    };
    uint32_t disk_sectors() override {
        return CardInfo.LogBlockNbr;
    };
    uint32_t disk_blocksize() override {
        return CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
    };
    
    //DiskIO
    uint8_t disk_initialize() override;

    uint8_t disk_status() override {
        return SD_status();
    }
    DRESULT disk_read (uint8_t *buff, uint32_t sector, uint32_t count) override {
        return SD_read(buff, sector, count);
    }

    DRESULT disk_write (const uint8_t *buff, uint32_t sector, uint32_t count) override {
        return SD_write(buff, sector, count);
    }

    DRESULT disk_ioctl (uint8_t cmd, void *buff) override {
        return SD_ioctl(cmd, buff);
    }

private:
    BSP_SD_CardInfo CardInfo;
};

#endif // SDCARD_SPI_H
=======
 Author: GA
 
*/
#ifndef SDCARDSDIO_H
#define SDCARDSDIO_H


#include "SDCard.h"
#include "HardwareSDIO.h"

class SDCardSDIO : public SDCard {
public:
    SDCardSDIO() noexcept;
    ~SDCardSDIO() noexcept {};

    void unmount() noexcept;
    void set_max_frequency(uint32_t maxFrequency) noexcept;  

    //DiskIO
    uint8_t disk_initialize() noexcept;
    uint8_t disk_status() noexcept;
    DRESULT disk_read (uint8_t *buff, uint32_t sector, uint32_t count) noexcept;
    DRESULT disk_write (const uint8_t *buff, uint32_t sector, uint32_t count) noexcept;
    DRESULT disk_ioctl (uint8_t cmd, void *buff) noexcept;

protected:
    HardwareSDIO *sdio;
    uint8_t status;
};

#endif
>>>>>>> v3.02-dev-unified
