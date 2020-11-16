/**
  ******************************************************************************
  * @file    sd_diskio.c
  * @brief   SD Disk I/O driver
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

#include "sd_diskio.h"

#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#define QUEUE_SIZE         (uint32_t) 10
#define READ_CPLT_MSG      (uint32_t) 1
#define WRITE_CPLT_MSG     (uint32_t) 2
#define RW_ABORT_MSG       (uint32_t) 3

#define osCMSIS            0x10002      ///< API version (main [31:16] .sub [15:0])

// Type wrappers for FreeRTOS
#if( configSUPPORT_STATIC_ALLOCATION == 1 )
#define osMessageQDef(name, queue_sz, type)   \
    const osMessageQDef_t os_messageQ_def_##name = \
    { (queue_sz), sizeof (type), NULL, NULL  }

#define osMessageQStaticDef(name, queue_sz, type, buffer, control)   \
    const osMessageQDef_t os_messageQ_def_##name = \
    { (queue_sz), sizeof (type) , (buffer), (control)}
#else //configSUPPORT_STATIC_ALLOCATION == 1
#define osMessageQDef(name, queue_sz, type)   \
    const osMessageQDef_t os_messageQ_def_##name = \
    { (queue_sz), sizeof (type) }
#endif

#define osMessageQ(name)  &os_messageQ_def_##name
#define osWaitForever     0xFFFFFFFF     ///< wait forever timeout value

typedef struct os_mailQ_cb *osMailQId;
typedef QueueHandle_t osMessageQId;
typedef TaskHandle_t osThreadId;

#if( configSUPPORT_STATIC_ALLOCATION == 1 )
typedef StaticQueue_t              osStaticMessageQDef_t;
#endif

typedef enum {
  osOK                    =     0,       ///< function completed; no error or event occurred.
  osEventSignal           =  0x08,       ///< function completed; signal event occurred.
  osEventMessage          =  0x10,       ///< function completed; message event occurred.
  osEventMail             =  0x20,       ///< function completed; mail event occurred.
  osEventTimeout          =  0x40,       ///< function completed; timeout occurred.
  osErrorParameter        =  0x80,       ///< parameter error: a mandatory parameter was missing or specified an incorrect object.
  osErrorResource         =  0x81,       ///< resource not available: a specified resource was not available.
  osErrorTimeoutResource  =  0xC1,       ///< resource not available within given time: a specified resource was not available within the timeout period.
  osErrorISR              =  0x82,       ///< not allowed in ISR context: the function cannot be called from interrupt service routines.
  osErrorISRRecursive     =  0x83,       ///< function called multiple times from ISR with same object.
  osErrorPriority         =  0x84,       ///< system cannot determine priority or thread has illegal priority.
  osErrorNoMemory         =  0x85,       ///< system is out of memory: it was impossible to allocate or reserve memory for the operation.
  osErrorValue            =  0x86,       ///< value of a parameter is out of range.
  osErrorOS               =  0xFF,       ///< unspecified RTOS error: run-time error but no other error message fits.
  os_status_reserved      =  0x7FFFFFFF  ///< prevent from enum down-size compiler optimization.
} osStatus;

typedef struct {
  osStatus                 status;     ///< status code: event or error information
  union  {
    uint32_t                    v;     ///< message as 32-bit value
    void                       *p;     ///< message or mail as void pointer
    int32_t               signals;     ///< signal flags
  } value;                             ///< event value
  union  {
    osMailQId             mail_id;     ///< mail id obtained by \ref osMailCreate
    osMessageQId       message_id;     ///< message id obtained by \ref osMessageCreate
  } def;                               ///< event definition
} osEvent;

typedef struct os_messageQ_def {
  uint32_t                queue_sz;       ///< number of elements in the queue
  uint32_t                item_sz;        ///< size of an item
#if( configSUPPORT_STATIC_ALLOCATION == 1 )
  uint8_t                 *buffer;        ///< buffer for static allocation; NULL for dynamic allocation
  osStaticMessageQDef_t   *controlblock;  ///< control block to hold queue's data for static allocation; NULL for dynamic allocation
#endif
  //void                    *pool;          ///< memory array for messages
} osMessageQDef_t;

static int inline inHandlerMode (void) {
  return __get_IPSR() != 0;
}

static uint32_t osKernelSysTick(void) {
  if (inHandlerMode()) {
    return xTaskGetTickCountFromISR();
  } else {
    return xTaskGetTickCount();
  }
}

static int32_t osKernelRunning(void) {
#if ( ( INCLUDE_xTaskGetSchedulerState == 1 ) || ( configUSE_TIMERS == 1 ) )
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    return 0;
  else
    return 1;
#else
	return (-1);
#endif	
}

static osMessageQId osMessageCreate (const osMessageQDef_t *queue_def) {

#if( configSUPPORT_STATIC_ALLOCATION == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
  if ((queue_def->buffer != NULL) && (queue_def->controlblock != NULL)) {
    return xQueueCreateStatic(
      queue_def->queue_sz,
      queue_def->item_sz,
      queue_def->buffer,
      queue_def->controlblock);
  } else {
    return xQueueCreate(queue_def->queue_sz, queue_def->item_sz);
  }
#elif ( configSUPPORT_STATIC_ALLOCATION == 1 )
  return xQueueCreateStatic(
    queue_def->queue_sz,
    queue_def->item_sz,
    queue_def->buffer,
    queue_def->controlblock);
#else  
  return xQueueCreate(queue_def->queue_sz, queue_def->item_sz);
#endif
}

static osEvent osMessageGet(osMessageQId queue_id, uint32_t millisec) {
  portBASE_TYPE taskWoken;
  TickType_t ticks;
  osEvent event;
  
  event.def.message_id = queue_id;
  event.value.v = 0;
  
  if (queue_id == NULL) {
    event.status = osErrorParameter;
    return event;
  }
  
  taskWoken = pdFALSE;
  
  ticks = 0;
  if (millisec == osWaitForever) {
    ticks = portMAX_DELAY;
  } else if (millisec != 0) {
    ticks = millisec / portTICK_PERIOD_MS;
    if (ticks == 0) {
      ticks = 1;
    }
  }
  
  if (inHandlerMode()) {
    if (xQueueReceiveFromISR(queue_id, &event.value.v, &taskWoken) == pdTRUE) {
      /* We have mail */
      event.status = osEventMessage;
    } else {
      event.status = osOK;
    }
    portEND_SWITCHING_ISR(taskWoken);
  } else {
    if (xQueueReceive(queue_id, &event.value.v, ticks) == pdTRUE) {
      /* We have mail */
      event.status = osEventMessage;
    } else {
      event.status = (ticks == 0) ? osOK : osEventTimeout;
    }
  }
  
  return event;
}

static osStatus osMessagePut(osMessageQId queue_id, uint32_t info, uint32_t millisec) {
  portBASE_TYPE taskWoken = pdFALSE;
  TickType_t ticks;
  
  ticks = millisec / portTICK_PERIOD_MS;
  if (ticks == 0) {
    ticks = 1;
  }
  
  if (inHandlerMode()) {
    if (xQueueSendFromISR(queue_id, &info, &taskWoken) != pdTRUE) {
      return osErrorOS;
    }
    portEND_SWITCHING_ISR(taskWoken);
  } else {
    if (xQueueSend(queue_id, &info, ticks) != pdTRUE) {
      return osErrorOS;
    }
  }
  
  return osOK;
}

/*
 * the following Timeout is useful to give the control back to the applications
 * in case of errors in either BSP_SD_ReadCpltCallback() or BSP_SD_WriteCpltCallback()
 * the value by default is as defined in the BSP platform driver otherwise 30 secs
 */
#define SD_TIMEOUT 30 * 1000

/*
 * Depending on the use case, the SD card initialization could be done at the
 * application level: if it is the case define the flag below to disable
 * the BSP_SD_Init() call in the SD_Initialize() and add a call to
 * BSP_SD_Init() elsewhere in the application.
 */
/* #define DISABLE_SD_INIT */

/*
 * when using cachable memory region, it may be needed to maintain the cache
 * validity. Enable the define below to activate a cache maintenance at each
 * read and write operation.
 * Notice: This is applicable only for cortex M7 based platform.
 */
/* #define ENABLE_SD_DMA_CACHE_MAINTENANCE  1 */

/*
* Some DMA requires 4-Byte aligned address buffer to correctly read/wite data,
* in FatFs some accesses aren't thus we need a 4-byte aligned scratch buffer to correctly
* transfer data
*/
/* #define ENABLE_SCRATCH_BUFFER */

/* Private variables ---------------------------------------------------------*/
#if defined(ENABLE_SCRATCH_BUFFER)
#if defined (ENABLE_SD_DMA_CACHE_MAINTENANCE)
ALIGN_32BYTES(static uint8_t scratch[BLOCKSIZE]); // 32-Byte aligned for cache maintenance
#else
__ALIGN_BEGIN static uint8_t scratch[BLOCKSIZE] __ALIGN_END;
#endif
#endif

/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

static osMessageQId SDQueueID = NULL;

#if ( configSUPPORT_STATIC_ALLOCATION == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 0 )
uint8_t SD_Queue_buffer[QUEUE_SIZE * sizeof(uint16_t)];
osStaticMessageQDef_t SD_Queue_cb;
#endif

static int SD_CheckStatusWithTimeout(uint32_t timeout) {
  uint32_t timer;
  /* block until SDIO peripherial is ready again or a timeout occur */
  timer = osKernelSysTick();
  while( osKernelSysTick() - timer < timeout) {
    if (BSP_SD_GetCardState() == SD_TRANSFER_OK) {
      return 0;
    }
  }

  return -1;
}

static DSTATUS SD_CheckStatus() {
  Stat = STA_NOINIT;

  if(BSP_SD_GetCardState() == SD_TRANSFER_OK) {
    Stat &= ~STA_NOINIT;
  }

  return Stat;
}

DSTATUS SD_initialize() {
  Stat = STA_NOINIT;

  /*
   * check that the kernel has been started before continuing
   * as the osMessage API will fail otherwise
   */
  if(osKernelRunning()) {
#if !defined(DISABLE_SD_INIT)
    if(BSP_SD_Init() == MSD_OK) {
      Stat = SD_CheckStatus();
    }
#else
    Stat = SD_CheckStatus();
#endif

    /*
    * if the SD is correctly initialized, create the operation queue
    * if not already created
    */
    if (Stat != STA_NOINIT) {
      if (SDQueueID == NULL) {
   #if ( configSUPPORT_STATIC_ALLOCATION == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 0 )
        osMessageQStaticDef(SD_Queue, QUEUE_SIZE, uint16_t, SD_Queue_buffer, &SD_Queue_cb);
   #else
        osMessageQDef(SD_Queue, QUEUE_SIZE, uint16_t);
   #endif
        SDQueueID = osMessageCreate (osMessageQ(SD_Queue));
      }

      if (SDQueueID == NULL) {
        Stat |= STA_NOINIT;
      }
    }
  }

  return Stat;
}

/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_status() {
  return SD_CheckStatus();
}

/**
  * @brief  Reads Sector(s)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */

DRESULT SD_read(BYTE *buff, DWORD sector, UINT count) {
  DRESULT res = RES_ERROR;
  uint32_t timer;
  osEvent event;
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
  uint32_t alignedAddr;
#endif
  /*
  * ensure the SDCard is ready for a new operation
  */

  if (SD_CheckStatusWithTimeout(SD_TIMEOUT) < 0) {
    return res;
  }

#if defined(ENABLE_SCRATCH_BUFFER)
  if (!((uint32_t)buff & 0x3)) {
#endif
    /* Fast path cause destination buffer is correctly aligned */
    uint8_t ret = BSP_SD_ReadBlocks_DMA((uint32_t*)buff, (uint32_t)(sector), count);

    if (ret == MSD_OK) {
      /* wait for a message from the queue or a timeout */
      event = osMessageGet(SDQueueID, SD_TIMEOUT);

      if (event.status == osEventMessage) {
        if (event.value.v == READ_CPLT_MSG) {
          timer = osKernelSysTick();
          /* block until SDIO IP is ready or a timeout occur */
          while(osKernelSysTick() - timer <SD_TIMEOUT) {
            if (BSP_SD_GetCardState() == SD_TRANSFER_OK) {
              res = RES_OK;
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
              /*
              the SCB_InvalidateDCache_by_Addr() requires a 32-Byte aligned address,
              adjust the address and the D-Cache size to invalidate accordingly.
              */
              alignedAddr = (uint32_t)buff & ~0x1F;
              SCB_InvalidateDCache_by_Addr((uint32_t*)alignedAddr, count*BLOCKSIZE + ((uint32_t)buff - alignedAddr));
#endif
              break;
            }
          }
        }
      }
    }

#if defined(ENABLE_SCRATCH_BUFFER)
  } else {
    /* Slow path, fetch each sector a part and memcpy to destination buffer */
    int i;

    for (i = 0; i < count; i++) {
      ret = BSP_SD_ReadBlocks_DMA((uint32_t*)scratch, (uint32_t)sector++, 1);
      if (ret == MSD_OK ) {
        /* wait until the read is successful or a timeout occurs */
        /* wait for a message from the queue or a timeout */
        event = osMessageGet(SDQueueID, SD_TIMEOUT);

        if (event.status == osEventMessage) {
          if (event.value.v == READ_CPLT_MSG) {
            timer = osKernelSysTick();
            /* block until SDIO IP is ready or a timeout occur */
            while(osKernelSysTick() - timer <SD_TIMEOUT) {
              ret = BSP_SD_GetCardState();

              if (ret == MSD_OK) {
                  break;
              }
            }

            if (ret != MSD_OK) {
              break;
            }
          }
        }
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
        /*
        *
        * invalidate the scratch buffer before the next read to get the actual data instead of the cached one
        */
        SCB_InvalidateDCache_by_Addr((uint32_t*)scratch, BLOCKSIZE);
#endif
        memcpy(buff, scratch, BLOCKSIZE);
        buff += BLOCKSIZE;
      } else {
        break;
      }
    }

    if ((i == count) && (ret == MSD_OK )) {
      res = RES_OK;
    }
  }
#endif
  return res;
}

/**
  * @brief  Writes Sector(s)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT SD_write(const BYTE *buff, DWORD sector, UINT count) {
  DRESULT res = RES_ERROR;
  uint32_t timer;
  osEvent event;
#if defined(ENABLE_SCRATCH_BUFFER)
  int32_t ret;
#endif

  /*
  * ensure the SDCard is ready for a new operation
  */

  if (SD_CheckStatusWithTimeout(SD_TIMEOUT) < 0) {
    return res;
  }

#if defined(ENABLE_SCRATCH_BUFFER)
  if (!((uint32_t)buff & 0x3)) {
#endif
#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
    uint32_t alignedAddr;
    /*
      the SCB_CleanDCache_by_Addr() requires a 32-Byte aligned address
      adjust the address and the D-Cache size to clean accordingly.
    */
    alignedAddr = (uint32_t)buff & ~0x1F;
    SCB_CleanDCache_by_Addr((uint32_t*)alignedAddr, count*BLOCKSIZE + ((uint32_t)buff - alignedAddr));
#endif

    if(BSP_SD_WriteBlocks_DMA((uint32_t*)buff,
                            (uint32_t) (sector),
                            count) == MSD_OK) {
      /* Get the message from the queue */
      event = osMessageGet(SDQueueID, SD_TIMEOUT);

      if (event.status == osEventMessage) {
        if (event.value.v == WRITE_CPLT_MSG) {
          timer = osKernelSysTick();
          /* block until SDIO IP is ready or a timeout occur */
          while(osKernelSysTick() - timer  < SD_TIMEOUT) {
            if (BSP_SD_GetCardState() == SD_TRANSFER_OK) {
              res = RES_OK;
              break;
            }
          }
        }
      }
    }
#if defined(ENABLE_SCRATCH_BUFFER)
  } else {
    /* Slow path, fetch each sector a part and memcpy to destination buffer */
    int i;

#if (ENABLE_SD_DMA_CACHE_MAINTENANCE == 1)
    /*
     * invalidate the scratch buffer before the next write to get the actual data instead of the cached one
     */
    SCB_InvalidateDCache_by_Addr((uint32_t*)scratch, BLOCKSIZE);
#endif
    for (i = 0; i < count; i++) {
      memcpy((void *)scratch, buff, BLOCKSIZE);
      buff += BLOCKSIZE;

      ret = BSP_SD_WriteBlocks_DMA((uint32_t*)scratch, (uint32_t)sector++, 1);
      if (ret == MSD_OK ) {
        /* wait until the read is successful or a timeout occurs */
        /* wait for a message from the queue or a timeout */
        event = osMessageGet(SDQueueID, SD_TIMEOUT);

        if (event.status == osEventMessage) {
          if (event.value.v == READ_CPLT_MSG) {
            timer = osKernelSysTick();
            /* block until SDIO IP is ready or a timeout occur */
            while(osKernelSysTick() - timer <SD_TIMEOUT) {
              ret = BSP_SD_GetCardState();

              if (ret == MSD_OK) {
                break;
              }
            }

            if (ret != MSD_OK) {
              break;
            }
          }
        }
      } else {
        break;
      }
    }

    if ((i == count) && (ret == MSD_OK )) {
      res = RES_OK;
    }
  }
#endif

  return res;
}
 
/**
  * @brief  I/O control operation
  * @param  lun : not used
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
DRESULT SD_ioctl(BYTE cmd, void *buff) {
  DRESULT res = RES_ERROR;
  BSP_SD_CardInfo CardInfo;

  if (Stat & STA_NOINIT) return RES_NOTRDY;

  switch (cmd) {
  /* Make sure that no pending write process */
  case CTRL_SYNC :
    res = RES_OK;
    break;

  /* Get number of sectors on the disk (DWORD) */
  case GET_SECTOR_COUNT :
    BSP_SD_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockNbr;
    res = RES_OK;
    break;

  /* Get R/W sector size (WORD) */
  case GET_SECTOR_SIZE :
    BSP_SD_GetCardInfo(&CardInfo);
    *(WORD*)buff = CardInfo.LogBlockSize;
    res = RES_OK;
    break;

  /* Get erase block size in unit of sector (DWORD) */
  case GET_BLOCK_SIZE :
    BSP_SD_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
    res = RES_OK;
    break;

  default:
    res = RES_PARERR;
  }

  return res;
}

/**
  * @brief Tx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void BSP_SD_WriteCpltCallback(void) {
   osMessagePut(SDQueueID, WRITE_CPLT_MSG, 0);
}

/**
  * @brief Rx Transfer completed callbacks
  * @param hsd: SD handle
  * @retval None
  */
void BSP_SD_ReadCpltCallback(void) {
   osMessagePut(SDQueueID, READ_CPLT_MSG, 0);
}

void BSP_SD_AbortCallback(void) {
   osMessagePut(SDQueueID, RW_ABORT_MSG, 0);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
