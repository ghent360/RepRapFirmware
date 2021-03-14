/*
 SDClass for use by RepRapFirmware
 
 Author: GA
 
*/
#ifndef SDCARDSDIO_H
#define SDCARDSDIO_H


#include "SDCard.h"
#ifdef USE_SDIO_LIB
#include "HardwareSDIO.h"
#else
#include "bsp_driver_sd.h"
#include "sd_diskio.h"
#endif

class SDCardSDIO : public SDCard {
public:
    SDCardSDIO() noexcept;
    ~SDCardSDIO() noexcept {};

    void unmount() noexcept override;
    void set_max_frequency(uint32_t maxFrequency) noexcept override;

    //DiskIO
    uint8_t disk_initialize() noexcept override;
    uint8_t disk_status() noexcept override
#ifdef USE_SDIO_LIB
    ;
#else
    {
        return SD_status();
    }
#endif

    DRESULT disk_read (uint8_t *buff, uint32_t sector, uint32_t count) noexcept override
#ifdef USE_SDIO_LIB
    ;
#else
    {
        return SD_read(buff, sector, count);
    }
#endif

    DRESULT disk_write (const uint8_t *buff, uint32_t sector, uint32_t count) noexcept override
#ifdef USE_SDIO_LIB
    ;
#else
    {
        return SD_write(buff, sector, count);
    }
#endif

    DRESULT disk_ioctl (uint8_t cmd, void *buff) noexcept override
#ifdef USE_SDIO_LIB
    ;
#else
    {
        return SD_ioctl(cmd, buff);
    }
#endif


protected:
#ifdef USE_SDIO_LIB
    HardwareSDIO *sdio;
    uint8_t status;
#else
    BSP_SD_CardInfo CardInfo;
#endif
};

#endif
