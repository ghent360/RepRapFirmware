/**
  ******************************************************************************
  * @file    sd_diskio.h
  * @brief   Header for sd_diskio.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#ifndef __SD_DISKIO_H
#define __SD_DISKIO_H

#include "bsp_driver_sd.h"
#include "diskio.h" //fatfs

#ifdef __cplusplus
extern "C" {
#endif
DSTATUS SD_initialize();
DSTATUS SD_status();
DRESULT SD_read(BYTE*, DWORD, UINT);
DRESULT SD_write(const BYTE*, DWORD, UINT);
DRESULT SD_ioctl(BYTE cmd, void *buff);

#ifdef __cplusplus
} // extern "C"
#endif

#define SD_DEFAULT_BLOCK_SIZE 512

#endif /* __SD_DISKIO_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
