/* 
 * File:   I2C_4BUS_EEPROM_PIC32.h
 * Author: Jim
 *
 * Created on August 14, 2019, 8:52 AM
 */

#include "GenericTypeDefs.h"
#ifndef I2C_4BUS_EEPROM_PIC32_H
#define	I2C_4BUS_EEPROM_PIC32_H

// #define TEST_OUT LATEbits.LATE3
#define EEPROM_ID 0xA0
#define EEPROM_WP PORTAbits.RA5

#define UINT32 unsigned long
#define BOOL unsigned char
#define UINT8 unsigned char

unsigned char EepromWriteBlock (unsigned char busID, unsigned char device, unsigned short startAddress, unsigned char *ptrData, unsigned short numBytes);
unsigned char EepromReadBlock (unsigned char busID, unsigned char device, unsigned short startAddress, unsigned char *ptrData, unsigned char numBytes);
unsigned char EepromWriteByte (unsigned char busID, unsigned char device, unsigned short address, unsigned char data);
unsigned char EepromReadByte (unsigned char busID, unsigned char device, unsigned short address, unsigned char *ptrData);
unsigned char I2CReceiveByte(unsigned char busID, unsigned char NACKflag);

void initI2C(unsigned char busID);
BOOL StartTransfer(unsigned char busID, BOOL restart);
BOOL TransmitOneByte(unsigned char busID, unsigned char data);
void StopTransfer(unsigned char busID);
void print_status(unsigned char busID);
unsigned char I2CGetACK(unsigned char busID);


#endif	/* I2C_4BUS_EEPROM_PIC32_H */

