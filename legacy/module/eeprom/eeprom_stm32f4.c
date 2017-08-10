#ifdef STM32F40_41xxx

#include "eeprom.h"
#include <stdint.h>

// implements a 4096byte eeprom
// 2048 16bit values
// --> only 16bit values are allowed

#define EEPROM_SIZE                4096 // bytes, 0x1000

// definitions has to be checked with linker configuration
#define EEPROM_FLASH_BASE_ADDRESS      0x08004000
#define EEPROM_FLASH_SECTOR_SIZE          0xC000  // 48k equals three 16k flash sectors
#define EEPROM_FLASH_PAGE_COUNT         3
#define EEPROM_FLASH_PAGE_SIZE         0x4000



// Device voltage range supposed to be [2.7V to 3.6V], the operation will be done by word
#define EEPROM_VOLTAGE_RANGE           (VoltageRange_3)



// Page status definitions
#define EEPROM_PAGE_STATUS_ERASED                ((uint16_t) 0xFFFF)     /* Page is empty */
#define EEPROM_PAGE_STATUS_RECEIVE_DATA          ((uint16_t) 0xEEEE)     /* Page is marked to receive data */
#define EEPROM_PAGE_STATUS_VALID_PAGE            ((uint16_t) 0xAAAA)     /* Page containing valid data */


#define EEPROM_ERROR_FLASH_BUSY         1
#define EEPROM_ERROR_FLASH_PGS          2
#define EEPROM_ERROR_FLASH_PGP          3
#define EEPROM_ERROR_FLASH_PGA          4
#define EEPROM_ERROR_FLASH_WRP          5
#define EEPROM_ERROR_FLASH_PROGRAM      6
#define FLASH_ERROR_OPERATION           7
#define EEPROM_ERROR_PAGE_FULL        100
#define EEPROM_ERROR_NO_VALID_PAGE    101
#define EEPROM_ERROR_NO_DATA          102
#define EEPROM_ERROR_INVALID_ADDRESS  103




// calc page base address
#define EEPROM_VALID_PAGE_INDEX(page_index)         (page_index>=0 && page_index<EEPROM_FLASH_PAGE_COUNT)
#define EEPROM_PAGE_BASE_ADDRESS(page_index)        (EEPROM_FLASH_BASE_ADDRESS + (EEPROM_FLASH_PAGE_SIZE * page_index))

const uint16_t EEPROM_FLASH_SECTOR_TYPE[EEPROM_FLASH_PAGE_COUNT] = {FLASH_Sector_1, FLASH_Sector_2, FLASH_Sector_3};

// page index of valid page (0 - (EEPROM_FLASH_PAGE_COUNT-1))
static int16_t eeprom_valid_page_index_;

////////////////////////////////////////////////////////////////////////////////////////////////////
// private functions
static uint16_t EEPROM_verifyPageFullWriteVariable(uint32_t address, uint16_t data);
static uint16_t EEPROM_pageTransfer(uint32_t eeprom_address, uint16_t data);

/**
  * @brief  Restore the pages to a known good state in case of page's status
  *   corruption after a power loss.
  * @param  None.
  * @retval - Flash error code: on write Flash error
  *         - FLASH_COMPLETE: on success
  */
uint16_t EEPROM_init() {
  uint16_t status = 0;
  int32_t i;
  int16_t validPage;
  uint16_t pageStatus;

  FLASH_Unlock();
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

  // find valid page
  validPage = -1;
  // get page status
  for (i=0;i<EEPROM_FLASH_PAGE_COUNT;++i) {
    pageStatus = *(uint16_t*) EEPROM_PAGE_BASE_ADDRESS(i);
    if (pageStatus==EEPROM_PAGE_STATUS_VALID_PAGE) {
      validPage = i;
      break;
    }
  }

  if (EEPROM_VALID_PAGE_INDEX(validPage))
    eeprom_valid_page_index_ = validPage;
  else
    // no valid page found
    // format eeprom
    status = EEPROM_format();

  return (status==FLASH_COMPLETE) ? 0 : status;
}

/**
  * @brief  Returns the last stored variable data, if found, which correspond to
  *   the passed virtual address
  * @param  VirtAddress: Variable virtual address
  * @param  Data: Global variable contains the read variable value
  * @retval Success or error status:
  *           - 0: if variable was found
  *           - 1: if the variable was not found
  *           - NO_VALID_PAGE: if no valid page was found.
  */
uint16_t EEPROM_readVariable(uint32_t address, uint16_t* data) {
  uint16_t status;
  uint16_t addressValue;
  uint32_t ptr, pageStartAddress;

  if (!EEPROM_VALID_PAGE_INDEX(eeprom_valid_page_index_))
    return EEPROM_ERROR_NO_VALID_PAGE;

  if (address>=EEPROM_SIZE || (address & 0x0001)) // address to big or no 16bit address
    return EEPROM_ERROR_INVALID_ADDRESS;

  pageStartAddress = EEPROM_PAGE_BASE_ADDRESS(eeprom_valid_page_index_);
  ptr = pageStartAddress + EEPROM_FLASH_PAGE_SIZE - 2; // get the valid page end address

  status = EEPROM_ERROR_NO_DATA;
  // check each active page address starting from end
  while (ptr > (pageStartAddress + 2)) {
    /// get the current location content to be compared with virtual address
    addressValue = (*(uint16_t *)ptr);

    // compare the read address with the virtual address
    if (addressValue == address) {
      // get content of address-2 which is variable value
      *data = (*(uint16_t *)(ptr - 2));
      status = 0;
      break;
    } else
      ptr -= 4; // next address location
  }

  return status;
}

/**
  * @brief  Writes/upadtes variable data in EEPROM.
  * @param  VirtAddress: Variable virtual address
  * @param  Data: 16 bit data to be written
  * @retval Success or error status:
  *           - FLASH_COMPLETE: on success
  *           - PAGE_FULL: if valid page is full
  *           - NO_VALID_PAGE: if no valid page was found
  *           - Flash error code: on write Flash error
  */
uint16_t EEPROM_writeVariable(uint32_t address, uint16_t data) {
  uint16_t status;

  if (address>=EEPROM_SIZE || (address & 0x0001)) // address to big or no 16bit address
    return EEPROM_ERROR_INVALID_ADDRESS;

  // write the variable virtual address and value in the EEPROM
  status = EEPROM_verifyPageFullWriteVariable(address, data);

  // in case the EEPROM active page is full
  if (status==EEPROM_ERROR_PAGE_FULL)
    status = EEPROM_pageTransfer(address, data); // perform page transfer to store new variable

  return (status==FLASH_COMPLETE) ? 0 : status;
}

//--------------------------------------------------------------------------------------------------
uint16_t EEPROM_format() {
  uint16_t status;
  int32_t i;

  // erase all pages
  for (i=0;i<EEPROM_FLASH_PAGE_COUNT;++i) {
    status = FLASH_EraseSector(EEPROM_FLASH_SECTOR_TYPE[i], EEPROM_VOLTAGE_RANGE);
    if (status != FLASH_COMPLETE)
      return status;
  }

  // set page 0 as valid page: Write VALID_PAGE at Page0 base address
  status = FLASH_ProgramHalfWord(EEPROM_PAGE_BASE_ADDRESS(0), EEPROM_PAGE_STATUS_VALID_PAGE);
  eeprom_valid_page_index_ = (status==FLASH_COMPLETE) ? 0 : -1;

  return (status==FLASH_COMPLETE) ? 0 : status;
}


/**
  * @brief  Verify if active page is full and Writes variable in EEPROM.
  * @param  VirtAddress: 16 bit virtual address of the variable
  * @param  Data: 16 bit data to be written as variable value
  * @retval Success or error status:
  *           - FLASH_COMPLETE: on success
  *           - PAGE_FULL: if valid page is full
  *           - NO_VALID_PAGE: if no valid page was found
  *           EEPROM_NO_VALID_PAGE no valid page found
  *           - Flash error code: on write Flash error
  */
static uint16_t EEPROM_verifyPageFullWriteVariable(uint32_t address, uint16_t data) {
  uint32_t ptr, pageEndAddress;
  uint16_t status = 0;

  if (!EEPROM_VALID_PAGE_INDEX(eeprom_valid_page_index_))
    return EEPROM_ERROR_NO_VALID_PAGE;

  pageEndAddress = EEPROM_PAGE_BASE_ADDRESS(eeprom_valid_page_index_) + EEPROM_FLASH_PAGE_SIZE - 2;
  ptr = EEPROM_PAGE_BASE_ADDRESS(eeprom_valid_page_index_);

  // check each active page address starting from begining
  while (ptr < pageEndAddress && status==0) {
    // verify if Address and Address+2 contents are 0xFFFFFFFF
    if ((*(uint32_t *)ptr) == 0xFFFFFFFF) {
      // set variable data
      status = FLASH_ProgramHalfWord(ptr, data);
      if (status != FLASH_COMPLETE)
        return status;

      // set variable virtual address
      status = FLASH_ProgramHalfWord(ptr + 2, address);
      return status;
    } else
      ptr += 4; // next address location
  }
  // no change to write variable -> page full
  return EEPROM_ERROR_PAGE_FULL;
}

/**
  * @brief  Transfers last updated variables data from the full Page to
  *   an empty one.
  * @param  VirtAddress: 16 bit virtual address of the variable
  * @param  Data: 16 bit data to be written as variable value
  * @retval Success or error status:
  *           - FLASH_COMPLETE: on success
  *           - PAGE_FULL: if valid page is full
  *           - NO_VALID_PAGE: if no valid page was found
  *           - Flash error code: on write Flash error
  */
static uint16_t EEPROM_pageTransfer(uint32_t eeprom_address, uint16_t data) {
  FLASH_Status status = FLASH_COMPLETE;
  uint32_t oldPageAddress, newPageAddress;
  int32_t i;
  uint32_t src_ptr, dst_ptr;
  // zeigt an ob die adresse schon tansferiert wurde
  // nötig für page transfer
  uint32_t eeprom_variable_address_[EEPROM_SIZE/(32*2)];
  uint32_t value, address;
  uint32_t ad_index, ad_mark;
  int16_t oldPage, newPage;

  if (!EEPROM_VALID_PAGE_INDEX(eeprom_valid_page_index_))
    return EEPROM_ERROR_NO_VALID_PAGE;

  oldPage = eeprom_valid_page_index_;
  newPage = oldPage + 1;
  if (newPage>=EEPROM_FLASH_PAGE_COUNT)
    newPage = 0;

  newPageAddress = EEPROM_PAGE_BASE_ADDRESS(newPage);
  oldPageAddress = EEPROM_PAGE_BASE_ADDRESS(oldPage);

  // set the new Page status to RECEIVE_DATA status
  status = FLASH_ProgramHalfWord(newPageAddress, EEPROM_PAGE_STATUS_RECEIVE_DATA);
  if (status != FLASH_COMPLETE)
    return status;

  for (i=0;i<(EEPROM_SIZE/(32*2));++i)
    eeprom_variable_address_[i] = 0;

  // start from the top
  src_ptr = oldPageAddress + EEPROM_FLASH_PAGE_SIZE - 4;
  dst_ptr = newPageAddress + 4;

  for (;src_ptr>oldPageAddress;src_ptr-=4) {
    value = *(uint32_t *) src_ptr; // address

    // addres -> bit position, brauche index und bitpos
    address = value & 0xFFFF;
    if (address>=EEPROM_SIZE || (address & 0x0001))
      continue;

    ad_index = address >> 6; // address/2/32
    ad_mark = address & 0x1F;
    ad_mark = 0x001 << ad_mark;

    if (eeprom_variable_address_[ad_index] & ad_mark) {
      // schon markiert -> adresse schon kopiert
      // aufhören alles ist kopiert
      break;
    } else {
      // adresse noch nicht kopiert -> markieren
      eeprom_variable_address_[ad_index] |= ad_mark;
      // kopieren
      FLASH_ProgramWord(dst_ptr, value);
      dst_ptr += 4;
    }
  }

  // write the variable passed as parameter in the new active page
  status = EEPROM_verifyPageFullWriteVariable(eeprom_address, data);
  if (status != FLASH_COMPLETE)
    return status;

  // erase the old page
  status = FLASH_EraseSector(EEPROM_FLASH_SECTOR_TYPE[oldPage], EEPROM_VOLTAGE_RANGE);
  if (status != FLASH_COMPLETE)
    return status;

  // set new page status to VALID_PAGE status
  status = FLASH_ProgramHalfWord(newPageAddress, EEPROM_PAGE_STATUS_VALID_PAGE);

  return status;
}

#endif // STM32F40_41xxx
