//EEPROM emulation library for STM32F1XX with HAL-Driver
//V2.0


//includes
#include "eeprom.h"


//private function prototypes;
static EEPROM_Result EEPROM_PageTransfer();
static EEPROM_Result EEPROM_SetPageStatus(EEPROM_Page Page, EEPROM_PageStatus PageStatus);
static EEPROM_Result EEPROM_PageToIndex(EEPROM_Page Page);


//global variables
static uint8_t EEPROM_SizeTable[EEPROM_VARIABLE_COUNT];		//EEPROM_SizeTable[i]: actual size of variable i (as EEPROM_Size)
static uint16_t EEPROM_Index[EEPROM_VARIABLE_COUNT];		//EEPROM_Index[i]: actual address of variable i (physical address = EEPROM_START_ADDRESS + EEPROM_Index[i])
															//if EEPROM_Index[i] = 0 variable i not assigned

static uint32_t EEPROM_ValidPage = EEPROM_PAGE_NONE;
static uint32_t EEPROM_ReceivingPage = EEPROM_PAGE_NONE;
static uint32_t EEPROM_ErasedPage = EEPROM_PAGE_NONE;

static uint32_t EEPROM_NextIndex = 0;


// initialize the EEPROM & restore the pages to a known good state in case of page's status corruption after a power loss
// - unlock flash
// - read each page status and check if valid
// - if invalid page status, format EEPROM
// - set global variables ValidPage, ReceivingPage and ErasedPage
// - build address index
// - resume page transfer if needed
//
// return: EEPROM_SUCCESS, EEPROM_NO_VALID_PAGE, EEPROM_FULL, EEPROM_ERROR, EEPROM_BUSY, EEPROM_TIMEOUT
EEPROM_Result EEPROM_Init()
{
	EEPROM_Result result;

	//unlock the flash memory
	HAL_FLASH_Unlock();

	//read each page status and check if valid
	EEPROM_PageStatus PageStatus0 = *((__IO uint16_t*) EEPROM_PAGE0);
	EEPROM_PageStatus PageStatus1 = *((__IO uint16_t*) EEPROM_PAGE1);
	uint8_t InvalidState = 0;
	if (PageStatus0 != EEPROM_VALID && PageStatus0 != EEPROM_RECEIVING && PageStatus0 != EEPROM_ERASED) InvalidState = 1;
	if (PageStatus1 != EEPROM_VALID && PageStatus1 != EEPROM_RECEIVING && PageStatus1 != EEPROM_ERASED) InvalidState = 1;
	if (PageStatus0 == PageStatus1) InvalidState = 1;

	// if invalid page status, format EEPROM (erase both pages and set page0 as valid)
	if (InvalidState)
	{
		FLASH_EraseInitTypeDef EraseDefinitions;
		EraseDefinitions.TypeErase = FLASH_TYPEERASE_PAGES;
		EraseDefinitions.Banks = FLASH_BANK_1;
		EraseDefinitions.PageAddress = EEPROM_PAGE0;
		EraseDefinitions.NbPages = 2;
		uint32_t PageError;

		result = HAL_FLASHEx_Erase(&EraseDefinitions, &PageError);
		if (result != EEPROM_SUCCESS) return result;

		result = HAL_FLASH_Program(EEPROM_SIZE16, EEPROM_PAGE0, EEPROM_VALID);
		if (result != EEPROM_SUCCESS) return result;

		PageStatus0 = EEPROM_VALID;
		PageStatus1 = EEPROM_ERASED;
	}

	//set global variables ValidPage, ReceivingPage and ErasedPage (one stays EEPROM_PAGE_NONE)
	if (PageStatus0 == EEPROM_VALID) EEPROM_ValidPage = EEPROM_PAGE0;
	if (PageStatus1 == EEPROM_VALID) EEPROM_ValidPage = EEPROM_PAGE1;
	if (PageStatus0 == EEPROM_RECEIVING) EEPROM_ReceivingPage = EEPROM_PAGE0;
	if (PageStatus1 == EEPROM_RECEIVING) EEPROM_ReceivingPage = EEPROM_PAGE1;
	if (PageStatus0 == EEPROM_ERASED) EEPROM_ErasedPage = EEPROM_PAGE0;
	if (PageStatus1 == EEPROM_ERASED) EEPROM_ErasedPage = EEPROM_PAGE1;

	//build address index (addresses from receiving page are dominant)
	EEPROM_PageToIndex(EEPROM_ValidPage);
	EEPROM_PageToIndex(EEPROM_ReceivingPage);

	//if needed, resume page transfer or just mark receiving page as valid
	if (EEPROM_ReceivingPage != EEPROM_PAGE_NONE)
	{
		if (EEPROM_ValidPage == EEPROM_PAGE_NONE)
		{
			result = EEPROM_SetPageStatus(EEPROM_ReceivingPage, EEPROM_VALID);
			if (result != EEPROM_SUCCESS) return result;
		}
		else
		{
			result = EEPROM_PageTransfer(EEPROM_ValidPage, EEPROM_ReceivingPage);
			if (result != EEPROM_SUCCESS) return result;
		}
	}
	
	return EEPROM_SUCCESS;
}


// returns the last stored variable value which correspond to the passed variable name
// - check if variable name exists
// - check if variable was assigned
// - read variable value from physical address with right size
//
// VariableName:	name (number) of the variable to read
// Value:			outputs the variable value
// return:			EEPROM_SUCCESS, EEPROM_INVALID_NAME, EEPROM_NOT_ASSIGNED
EEPROM_Result EEPROM_ReadVariable(uint16_t VariableName, EEPROM_Value* Value)
{
	//check if variable name exists
	if (VariableName >= EEPROM_VARIABLE_COUNT) return EEPROM_INVALID_NAME;

	//check if variable was assigned
	uint32_t Address = EEPROM_START_ADDRESS + EEPROM_Index[VariableName];
	if (Address == EEPROM_PAGE0) return EEPROM_NOT_ASSIGNED;

	//read variable value from physical address with right size
	switch (EEPROM_SizeTable[VariableName])
	{
		case EEPROM_SIZE16: (*Value).uInt16 = *((__IO uint16_t*) Address); break;
		case EEPROM_SIZE32: (*Value).uInt32 = *((__IO uint32_t*) Address); break;
		case EEPROM_SIZE64: (*Value).uInt64 = *((__IO uint32_t*) Address) | ((uint64_t) *((__IO uint32_t*) (Address + 4)) << 32); break;
		default: return EEPROM_NOT_ASSIGNED;
	}

	return EEPROM_SUCCESS;
}


// writes variable in EEPROM if page not full
// - get writing page's end address
// - calculate memory usage of variable
// - check if page full
//		- check if data is too much to store on one page
//		- mark the target page as receiving
//		- change next index to receiving page
//		- write the variable to target page
//		- do page transfer
// - else (if enough space)
//		- write variable value
//		- create and write variable header (size and name)
//		- update index & size table
//		- update next index
//
// VariableName:	name (number) of the variable to write
// Value:			value to be written
// Size:			size of "Value" as EEPROM_Size
// return:			EEPROM_SUCCESS, EEPROM_NO_VALID_PAGE, EEPROM_FULL, EEPROM_ERROR, EEPROM_BUSY, EEPROM_TIMEOUT
EEPROM_Result EEPROM_WriteVariable(uint16_t VariableName, EEPROM_Value Value, uint8_t Size)
{
	EEPROM_Result result;

	//get writing page's end address (prefer writing to receiving page)
	EEPROM_Page WritingPage = EEPROM_ValidPage;
	if (EEPROM_ReceivingPage != EEPROM_PAGE_NONE) WritingPage = EEPROM_ReceivingPage;
	if (WritingPage == EEPROM_PAGE_NONE) return EEPROM_NO_VALID_PAGE;
	uint32_t PageEndAddress = WritingPage + FLASH_PAGE_SIZE;

	//calculate memory usage of variable
	uint8_t Bytes = 2 + (1 << Size);
	if (Size == EEPROM_SIZE_DELETED) Bytes = 2;

	//check if enough free space or page full
	if (EEPROM_NextIndex == 0 || PageEndAddress - EEPROM_NextIndex < Bytes)
	{
		//check if data is too much to store on one page
		uint16_t RequiredMemory = 2;
		for (uint16_t i = 0; i < EEPROM_VARIABLE_COUNT; i++)
		{
			if (i == VariableName) RequiredMemory += 2 + (1 << Size);
			else if (EEPROM_SizeTable[i] != EEPROM_SIZE_DELETED) RequiredMemory += 2 + (1 << EEPROM_SizeTable[i]);
		}
		if (RequiredMemory > FLASH_PAGE_SIZE) return EEPROM_FULL;

		//mark the empty page as receiving
		result = EEPROM_SetPageStatus(EEPROM_ErasedPage, EEPROM_RECEIVING);
		if (result != EEPROM_SUCCESS) return result;

		//change next index to receiving page
		EEPROM_NextIndex = EEPROM_ReceivingPage + 2;

		//write the variable to receiving page (by calling this function again)
		result = EEPROM_WriteVariable(VariableName, Value, Size);
		if (result != EEPROM_SUCCESS) return result;

		//do page transfer
		result = EEPROM_PageTransfer();
		if (result != EEPROM_SUCCESS) return result;
	}

	//else (if enough space)
	else
	{
		//write variable value
		if (Size != EEPROM_SIZE_DELETED)
		{
			result = HAL_FLASH_Program(Size, EEPROM_NextIndex + 2, Value.uInt64);
			if (result != EEPROM_SUCCESS) return result;
		}

		//create and write variable header (size and name)
		uint16_t VariableHeader = VariableName + (Size << 14);
		result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, EEPROM_NextIndex, VariableHeader);
		if (result != EEPROM_SUCCESS) return result;

		//update index & size table
		EEPROM_Index[VariableName] = EEPROM_NextIndex + 2 - EEPROM_START_ADDRESS;
		EEPROM_SizeTable[VariableName] = Size;
		if (Size == EEPROM_SIZE_DELETED) EEPROM_Index[VariableName] = 0;

		//update next index
		EEPROM_NextIndex += Bytes;
		if (EEPROM_NextIndex >= PageEndAddress) EEPROM_NextIndex = 0;
	}

	return EEPROM_SUCCESS;
}


//marks a variable as deleted so it can't be read anymore and is discarded on next page transfer
// - call write variable with size EEPROM_SIZE_DELETED
//
// VariableName:	name (number) of the variable to delete
// return:			EEPROM_SUCCESS, EEPROM_NO_VALID_PAGE, EEPROM_FULL, EEPROM_ERROR, EEPROM_BUSY, EEPROM_TIMEOUT
EEPROM_Result EEPROM_DeleteVariable(uint16_t VariableName)
{
	return EEPROM_WriteVariable(VariableName, (EEPROM_Value) (uint16_t) 0, EEPROM_SIZE_DELETED);
}


// transfers latest variable values from valid page to receiving page
// - get start & end address of valid page (source)
// - copy each variable
//		- check if is stored on the source page
//		- read variable value
//		- write variable to receiving page
// - erase source page
// - mark receiving page as valid
//
// return: EEPROM_SUCCESS, EEPROM_NO_VALID_PAGE, EEPROM_FULL, EEPROM_ERROR, EEPROM_BUSY, EEPROM_TIMEOUT
static EEPROM_Result EEPROM_PageTransfer()
{
	EEPROM_Result result;
	EEPROM_Value Value;

	//get start & end address of valid page (source) (as offset to EEPROM start)
	uint16_t StartAddress = EEPROM_ValidPage - EEPROM_START_ADDRESS;
	uint16_t EndAddress = EEPROM_ValidPage - EEPROM_START_ADDRESS + FLASH_PAGE_SIZE;

	//copy each variable
	for (uint16_t i = 0; i < EEPROM_VARIABLE_COUNT; i++)
	{
		//check if is stored on the source page
		if (StartAddress < EEPROM_Index[i] && EEPROM_Index[i] < EndAddress)
		{
			//read variable value (if possible)
			if (EEPROM_ReadVariable(i, &Value) == EEPROM_SUCCESS)
			{
				//write variable to receiving page
				result = EEPROM_WriteVariable(i, Value, EEPROM_SizeTable[i]);
				if (result != EEPROM_SUCCESS) return result;
			}
		}
	}

	//erase source page
	result = EEPROM_SetPageStatus(EEPROM_ValidPage, EEPROM_ERASED);
	if (result != EEPROM_SUCCESS) return result;

	//mark receiving page as valid
	result = EEPROM_SetPageStatus(EEPROM_ReceivingPage, EEPROM_VALID);
	if (result != EEPROM_SUCCESS) return result;

	return EEPROM_SUCCESS;
}


// sets the page status and updates references from global variables
// - check if erase operation required
//		- remove every variable from index, that is stored on erase page
//		- setup erase definitions
//		- erase page
// - else write status to flash
// - update global page status variables
//
// Page:		page to change the status (as EEPROM_Page)
// PageStatus:	page status to set for page (as EEPROM_PageStatus)
// return:		EEPROM_SUCCESS, EEPROM_ERROR, EEPROM_BUSY or EEPROM_TIMEOUT
static EEPROM_Result EEPROM_SetPageStatus(EEPROM_Page Page, EEPROM_PageStatus PageStatus)
{
	EEPROM_Result result;

	//check if erase operation required
	if (PageStatus == EEPROM_ERASED)
	{
		//remove every variable from index, that is stored on erase page
		uint16_t StartAddress = Page - EEPROM_START_ADDRESS;
		uint16_t EndAddress = Page - EEPROM_START_ADDRESS + FLASH_PAGE_SIZE;
		for (uint16_t i = 0; i < EEPROM_VARIABLE_COUNT; i++)
		{
			if (StartAddress < EEPROM_Index[i] && EEPROM_Index[i] < EndAddress) EEPROM_Index[i] = 0;
		}

		//setup erase definitions
		FLASH_EraseInitTypeDef EraseDefinitions;
		EraseDefinitions.TypeErase = FLASH_TYPEERASE_PAGES;
		EraseDefinitions.Banks = FLASH_BANK_1;
		EraseDefinitions.PageAddress = Page;
		EraseDefinitions.NbPages = 1;
		uint32_t PageError;

		//erase page
		result = HAL_FLASHEx_Erase(&EraseDefinitions, &PageError);
		if (result != EEPROM_SUCCESS) return result;
	}

	//else write status to flash
	else
	{
		result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Page, PageStatus);
		if (result != EEPROM_SUCCESS) return result;
	}

	//update global page status variables (remove page from old status and attach to new status)
	if (EEPROM_ValidPage == Page) EEPROM_ValidPage = EEPROM_PAGE_NONE;
	else if (EEPROM_ReceivingPage == Page) EEPROM_ReceivingPage = EEPROM_PAGE_NONE;
	else if (EEPROM_ErasedPage == Page) EEPROM_ErasedPage = EEPROM_PAGE_NONE;

	if (PageStatus == EEPROM_VALID) EEPROM_ValidPage = Page;
	else if (PageStatus == EEPROM_RECEIVING) EEPROM_ReceivingPage = Page;
	else if (PageStatus == EEPROM_ERASED) EEPROM_ErasedPage = Page;

	return EEPROM_SUCCESS;
}


// reads the whole page, fills the index with variable addresses and the size table with variable sizes
// - declare variables
// - ignore call when Page is PAGE_NONE
// - get page addresses
// - loop through page addresses
// - read potential variable header
// - if no header written
//		- loop through next 4 halfword and check if there is anything written
//		- while looping count the size of written data
//		- if no data found, last variable of page was reached (return)
// - else (if header written)
//		- get size code
//		- check for valid name
//		- if everything valid, update the index and the size table
//		- calculate size in bytes from size code
// - go to next address on page
// - set next free flash address
// - return on loop end
//
// Page:	page to search for variables
// return:	EEPROM_SUCCESS
static EEPROM_Result EEPROM_PageToIndex(EEPROM_Page Page)
{
	//declare variables
	uint16_t VariableHeader;																			//header of current variable (first 2 bits size code, rest name)
	uint8_t SizeCode;																					//size of current variable as Size code
	uint8_t Size;																						//size of current variable in bytes
	uint16_t Name;																						//name of current variable

	//ignore call when Page is PAGE_NONE
	if (Page == EEPROM_PAGE_NONE) return EEPROM_SUCCESS;

	//get page addresses
	uint32_t Address = Page + 2;
	uint32_t PageEndAddress = Page + FLASH_PAGE_SIZE;

	//loop through page starting after page header
	while (Address < PageEndAddress)
	{
		//read potential variable header
		VariableHeader = *((__IO uint16_t*) Address);

		//if no header written (causes: end of data reached or reset while writing)
		if (VariableHeader == 0xFFFF)
		{
			//loop through next 4 halfword and check if there is anything written
			Size = 0;
			for (uint8_t i = 2; i <= 8; i += 2)
			{
				if (Address + i >= PageEndAddress) break;
				//while looping count the size of written data (resulting from reset while writing)
				if (*((__IO uint16_t*) (Address + i)) != 0xFFFF) Size = i;
			}
			//if no data found, last variable of page was reached (end loop)
			if (Size == 0) break;
		}

		//else (if header written, proper variable value is following)
		else
		{
			//get size code
			SizeCode = VariableHeader >> 14;

			//check for valid name (VARIABLE_COUNT might have been reduced between builds, but old variables are still in flash)
			Name = VariableHeader & 0b0011111111111111;
			if (Name < EEPROM_VARIABLE_COUNT)
			{
				//if everything valid, update the index and the size table
				EEPROM_Index[Name] = Address + 2 - EEPROM_START_ADDRESS;
				EEPROM_SizeTable[Name] = SizeCode;
				if (SizeCode == EEPROM_SIZE_DELETED) EEPROM_Index[Name] = 0;
			}

			//calculate size in bytes from size code
			Size = 1 << SizeCode;
			if (SizeCode == EEPROM_SIZE_DELETED) Size = 0;
		}

		//go to next address on page
		Address = Address + 2 + Size;
	}

	//set next free flash address
	EEPROM_NextIndex = Address;
	if (Address >= PageEndAddress) EEPROM_NextIndex = 0;

	//return on loop end
	return EEPROM_SUCCESS;
}
