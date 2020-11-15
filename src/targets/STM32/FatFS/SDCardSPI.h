/*
 SDClass for use by RepRapFirmware
 
 Author: sdavi
 
*/
#ifndef SDCARD_SPI_H
#define SDCARD_SPI_H


//#include "gpio.h"
#include "Core.h"
#include "SharedSpiClient.h"
#include "diskio.h" //fatfs
#include "SDCard.h"

class SDCardSPI : public SDCard {
public:
    SDCardSPI(SSPChannel SSPSlot, Pin cs);
    ~SDCardSPI() {};
    void ReInit(Pin cs, uint32_t frequency);
    void SetSSPChannel(SSPChannel channel);

    CARD_TYPE card_type(void) override ;
    
    void unmount() override;
    uint32_t interface_speed() override {
        return frequency;
    };
    uint32_t disk_sectors() override {
        return sdcardSectors;
    };
    uint32_t disk_blocksize() override {
        return sdcardBlockSize;
    };
    uint32_t disk_highSpeedMode() {return isHighSpeed; };
    
    //DiskIO
    uint8_t disk_initialize() override;
    uint8_t disk_status() override;
    DRESULT disk_read (uint8_t *buff, uint32_t sector, uint32_t count) override;
    DRESULT disk_write (const uint8_t *buff, uint32_t sector, uint32_t count) override;
    DRESULT disk_ioctl (uint8_t cmd, void *buff) override;

private:
    inline uint8_t xchg_spi (uint8_t dat);
    inline void rcvr_spi_multi(uint8_t *buff, uint32_t btr);
    inline void xmit_spi_multi (const uint8_t *buff, uint32_t btx);
    int wait_ready (uint32_t wt);
    void deselect (void);
    int select (void);
    int rcvr_datablock (uint8_t *buff, uint32_t btr);
    int xmit_datablock (const uint8_t *buff, uint8_t token);
    uint8_t send_cmd (uint8_t cmd, uint32_t arg);

    bool enableHighSpeedMode();
    
    //variables
    SharedSpiClient *spi;
    CARD_TYPE cardtype;
    uint32_t maxFrequency; //max frequency the user has specified
    uint32_t frequency;
    uint8_t status;
    
    
    uint32_t sdcardSectors;
    uint32_t sdcardBlockSize;
    
    bool isHighSpeed;
};

#endif // SDCARD_SPI_H
