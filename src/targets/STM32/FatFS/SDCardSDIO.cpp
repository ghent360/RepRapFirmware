#include "SDCardSDIO.h"

SDCardSDIO::SDCardSDIO() {
}

uint8_t SDCardSDIO::disk_initialize() {
    uint8_t status = SD_initialize();
    if (!status) {
        BSP_SD_GetCardInfo(&CardInfo);
    }
    return status;
}

SDCardSDIO::~SDCardSDIO() {

}

void SDCardSDIO::unmount() {
}