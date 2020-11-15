/*
 SDClass for use by RepRapFirmware
 
 Author: sdavi
 
*/
#ifndef SDCARD_H
#define SDCARD_H

//#include "gpio.h"
#include "Core.h"
#include "diskio.h" //fatfs

/* MMC card type flags (MMC_GET_TYPE) */
#define CT_MMC        0x01        /* MMC ver 3 */
#define CT_SD1        0x02        /* SD ver 1 */
#define CT_SD2        0x04        /* SD ver 2 */
#define CT_SDC        (CT_SD1|CT_SD2)    /* SD */
#define CT_BLOCK      0x08        /* Block addressing */

typedef uint8_t CARD_TYPE;

class SDCard {
public:
    virtual ~SDCard() = default;

    virtual CARD_TYPE card_type(void) = 0;
    virtual void unmount() = 0;
    virtual uint32_t interface_speed() = 0;
    virtual uint32_t disk_sectors() = 0;
    virtual uint32_t disk_blocksize() = 0;
    
    virtual uint8_t disk_initialize() = 0;
    virtual uint8_t disk_status() = 0;
    virtual DRESULT disk_read (uint8_t *buff, uint32_t sector, uint32_t count) = 0;
    virtual DRESULT disk_write (const uint8_t *buff, uint32_t sector, uint32_t count) = 0;
    virtual DRESULT disk_ioctl (uint8_t cmd, void *buff) = 0;
};

#endif // SDCARD_H
