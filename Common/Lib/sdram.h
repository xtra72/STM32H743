#ifndef SDRAM_H__
#define SDRAM_H__

#include "target.h"

#if SUPPORT_DRAM

typedef struct
{
    uint32_t    startAddress;
    uint32_t    size;
    uint32_t    startHeapAddress;
    uint32_t    heapSize;
}   SDRAM_CONFIG;

RET_VALUE   SDRAM_init(SDRAM_HandleTypeDef *hsdram);

RET_VALUE   SDRAM_start(void);
RET_VALUE   SDRAM_fill(uint32_t address, uint32_t value, uint32_t size);

void *SDRAM_malloc( size_t xWantedSize );
void SDRAM_free( void *pv );
size_t SDRAM_getFreeHeapSize( void );
size_t SDRAM_getMinimumEverFreeHeapSize( void );

#endif
#endif
