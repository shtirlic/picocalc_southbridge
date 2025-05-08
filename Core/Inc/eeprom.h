#include "stm32f1xx_hal.h"
//EEPROM emulation library for STM32F1XX with HAL-Driver
//V2.0


//define to prevent recursive inclusion
#ifndef __EEPROM_H
#define __EEPROM_H

//---------------------------------------------Datas registration--------------------------------------------

#define EEPROM_VAR_ID	(0)			// 16b: Init ID: 0xCA1C
#define EEPROM_VAR_CFG	(1)			// 16b: CFG(15:8) + INT_CFG(7:0) reg
#define EEPROM_VAR_KBD	(2)		// 32b: DEB(15:0) + 0x00 + FRQ(7:0) regs
#define EEPROM_VAR_BCKL	(3)			// 16b: LCD(15:8) + KBD(7:0) backlight step indice

//-------------------------------------------library configuration-------------------------------------------

//number of variables (maximum variable name is EEPROM_VARIABLE_COUNT - 1)
//keep in mind it is limited by page size
//maximum is also determined by your variable sizes
//space utilization ratio X = (2 + 4*COUNT_16BIT + 6*COUNT_32BIT + 10*COUNT_64BIT) / PAGE_SIZE
//if X is high, variable changes more often require a page transfer --> lifetime of the flash can be reduced significantly
//depending on your variable change rate, X should be at least <50%
#define EEPROM_VARIABLE_COUNT	(uint16_t) 4

//flash size of used STM32F1XX device in KByte
#define EEPROM_FLASH_SIZE		(uint16_t) 64

//-------------------------------------------------constants-------------------------------------------------

//EEPROM emulation start address in flash: use last two pages of flash memory
#define EEPROM_START_ADDRESS	(uint32_t) (0x08000000 + 1024*EEPROM_FLASH_SIZE - 2*FLASH_PAGE_SIZE)

//used flash pages for EEPROM emulation
typedef enum
{
	EEPROM_PAGE0			= EEPROM_START_ADDRESS,						//Page0
	EEPROM_PAGE1			= EEPROM_START_ADDRESS + FLASH_PAGE_SIZE,	//Page1
	EEPROM_PAGE_NONE		= 0x00000000								//no page
} EEPROM_Page;

//page status
typedef enum
{
	EEPROM_ERASED			= 0xFFFF,									//Page is empty
	EEPROM_RECEIVING		= 0xEEEE,									//Page is marked to receive data
	EEPROM_VALID			= 0x0000									//Page containing valid data
} EEPROM_PageStatus;

//results
typedef enum
{
	EEPROM_SUCCESS			= 0x00,										//Method successful / HAL_OK
	EEPROM_ERROR			= 0x01,										//Error: HAL_ERROR occurred
	EEPROM_BUSY				= 0x02,										//Error: HAL_BUSY occurred
	EEPROM_TIMEOUT			= 0x03,										//Error: HAL_TIMEOUT occurred
	EEPROM_NO_VALID_PAGE	= 0x04,										//Error: no valid page found
	EEPROM_NOT_ASSIGNED		= 0x05,										//Error: variable was never assigned
	EEPROM_INVALID_NAME		= 0x06,										//Error: variable name to high for variable count
	EEPROM_FULL				= 0x07										//Error: EEPROM is full
} EEPROM_Result;

//sizes ( halfwords = 2 ^ (size-1) )
typedef enum
{
	EEPROM_SIZE_DELETED		= 0x00,										//variable is deleted (no size)
	EEPROM_SIZE16			= 0x01,										//variable size = 16 bit = 1 Halfword
	EEPROM_SIZE32			= 0x02,										//variable size = 32 bit = 2 Halfwords
	EEPROM_SIZE64			= 0x03										//variable size = 64 bit = 4 Halfwords
} EEPROM_Size;

typedef union
 {
	int16_t Int16;
	int32_t Int32;
	int64_t Int64;
	uint16_t uInt16;
	uint32_t uInt32;
	uint64_t uInt64;
	float Float;
	double Double;
 } EEPROM_Value;

//----------------------------------------------public functions---------------------------------------------

EEPROM_Result EEPROM_Init();
EEPROM_Result EEPROM_ReadVariable(uint16_t VariableName, EEPROM_Value* Value);
EEPROM_Result EEPROM_WriteVariable(uint16_t VariableName, EEPROM_Value Value, EEPROM_Size Size);
EEPROM_Result EEPROM_DeleteVariable(uint16_t VariableName);

#endif
