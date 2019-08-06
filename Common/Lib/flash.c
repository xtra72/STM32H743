#include "target.h"
#include "config.h"
#include "flash.h"

#define __MODULE_NAME__     "FLASH"
#include "trace.h"

#define FLASH_BASE_ADDR      (uint32_t)(FLASH_BASE)
#define FLASH_END_ADDR       (uint32_t)(0x081FFFFF)

/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0_BANK1     ((uint32_t)0x08000000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK1     ((uint32_t)0x08020000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK1     ((uint32_t)0x08040000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK1     ((uint32_t)0x08060000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK1     ((uint32_t)0x08080000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK1     ((uint32_t)0x080A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK1     ((uint32_t)0x080C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK1     ((uint32_t)0x080E0000) /* Base @ of Sector 7, 128 Kbytes */

/* Base address of the Flash sectors Bank 2 */
#define ADDR_FLASH_SECTOR_0_BANK2     ((uint32_t)0x08100000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK2     ((uint32_t)0x08120000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK2     ((uint32_t)0x08140000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK2     ((uint32_t)0x08160000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK2     ((uint32_t)0x08180000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK2     ((uint32_t)0x081A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK2     ((uint32_t)0x081C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK2     ((uint32_t)0x081E0000) /* Base @ of Sector 7, 128 Kbytes */


static  uint32_t FLASH_getSector(uint32_t _address);

RET_VALUE   FLASH_erase(uint32_t _startAddress, uint32_t _endAddress)
{
    uint32_t firstSector = 0, numberOfSectors = 0;
    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t error = 0;

   /* -2- Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    /* -3- Erase the user Flash area
    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

    /* Get the 1st sector to erase */
    firstSector = FLASH_getSector(_startAddress);
    /* Get the number of sector to erase from 1st sector*/
    numberOfSectors = FLASH_getSector(_endAddress) - firstSector + 1;

    /* Fill EraseInit structure*/
    eraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
    eraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
    eraseInitStruct.Banks         = FLASH_BANK_1;
    eraseInitStruct.Sector        = firstSector;
    eraseInitStruct.NbSectors     = numberOfSectors;

    if (HAL_FLASHEx_Erase(&eraseInitStruct, &error) != HAL_OK)
    {
        DEBUG("Flash erase error : %d\n", HAL_FLASH_GetError());
        return  RET_ERROR;
    }

    return  RET_OK;
}

RET_VALUE   FLASH_write(uint32_t _address, uint8_t *_data, uint32_t _size)
{
    if (((_address & 31) != 0) || ((_size & 31) != 0))
    {
        DEBUG("Invalid data : %08x, %d\n", _address, _size);
        return  RET_ERROR;
    }

    uint32_t    startAddress = _address;
    uint32_t    endAddress = _address + _size - 1;
    uint32_t    dataAddress = (uint32_t)_data;

    while (startAddress < endAddress)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, startAddress, (uint64_t)dataAddress) == HAL_OK)
        {
            startAddress = startAddress + 32; /* increment for the next Flash word*/
            dataAddress = dataAddress + 32;
        }
        else
        {
            DEBUG("Flash write error : %08x, %d\n", startAddress, HAL_FLASH_GetError());
        }
    }

    return  RET_OK;

}



/**
  * @brief  Gets the sector of a given address
  * @param  Address Address of the FLASH Memory
  * @retval The sector of a given address
  */
uint32_t FLASH_getSector(uint32_t _address)
{
    uint32_t sector = 0;

    if(((_address < ADDR_FLASH_SECTOR_1_BANK1) && (_address >= ADDR_FLASH_SECTOR_0_BANK1)) || \
     ((_address < ADDR_FLASH_SECTOR_1_BANK2) && (_address >= ADDR_FLASH_SECTOR_0_BANK2)))
    {
        sector = FLASH_SECTOR_0;
    }
    else if(((_address < ADDR_FLASH_SECTOR_2_BANK1) && (_address >= ADDR_FLASH_SECTOR_1_BANK1)) || \
          ((_address < ADDR_FLASH_SECTOR_2_BANK2) && (_address >= ADDR_FLASH_SECTOR_1_BANK2)))
    {
        sector = FLASH_SECTOR_1;
    }
    else if(((_address < ADDR_FLASH_SECTOR_3_BANK1) && (_address >= ADDR_FLASH_SECTOR_2_BANK1)) || \
          ((_address < ADDR_FLASH_SECTOR_3_BANK2) && (_address >= ADDR_FLASH_SECTOR_2_BANK2)))
    {
        sector = FLASH_SECTOR_2;
    }
    else if(((_address < ADDR_FLASH_SECTOR_4_BANK1) && (_address >= ADDR_FLASH_SECTOR_3_BANK1)) || \
          ((_address < ADDR_FLASH_SECTOR_4_BANK2) && (_address >= ADDR_FLASH_SECTOR_3_BANK2)))
    {
        sector = FLASH_SECTOR_3;
    }
    else if(((_address < ADDR_FLASH_SECTOR_5_BANK1) && (_address >= ADDR_FLASH_SECTOR_4_BANK1)) || \
          ((_address < ADDR_FLASH_SECTOR_5_BANK2) && (_address >= ADDR_FLASH_SECTOR_4_BANK2)))
    {
        sector = FLASH_SECTOR_4;
    }
    else if(((_address < ADDR_FLASH_SECTOR_6_BANK1) && (_address >= ADDR_FLASH_SECTOR_5_BANK1)) || \
          ((_address < ADDR_FLASH_SECTOR_6_BANK2) && (_address >= ADDR_FLASH_SECTOR_5_BANK2)))
    {
        sector = FLASH_SECTOR_5;
    }
    else if(((_address < ADDR_FLASH_SECTOR_7_BANK1) && (_address >= ADDR_FLASH_SECTOR_6_BANK1)) || \
          ((_address < ADDR_FLASH_SECTOR_7_BANK2) && (_address >= ADDR_FLASH_SECTOR_6_BANK2)))
    {
        sector = FLASH_SECTOR_6;
    }
    else if(((_address < ADDR_FLASH_SECTOR_0_BANK2) && (_address >= ADDR_FLASH_SECTOR_7_BANK1)) || \
          ((_address < FLASH_END_ADDR) && (_address >= ADDR_FLASH_SECTOR_7_BANK2)))
    {
        sector = FLASH_SECTOR_7;
    }
    else
    {
        sector = FLASH_SECTOR_7;
    }

    return sector;
}
