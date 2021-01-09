/* I2C_4BUS_EEPROM_PIC32.c - I2C routines for reading/writing 24LC256 EEprom 
 * 
 * Adapted from Super Copter code on Electro-Tech-Online.com SUPER COPTER
 * Key words: "PIC32 + C32 can't read I2C BUS"
 * SOURCE: https://www.electro-tech-online.com/threads/pic32-c32-cant-read-i2c-bus.145272/
 * 8-14-19 JBS
 */

// #include "Defs.h"
#include "I2C_4BUS_EEPROM_PIC32.h"
// #include "HardwareProfile.h"
#include <plib.h>
#include <xc.h>

#define true TRUE
#define false FALSE
#define RD 1
#define WR 0

#define SYS_FREQ 80000000
#define GetPeripheralClock() SYS_FREQ

void initI2C(unsigned char busID)
{                   
    I2CConfigure(busID, I2C_ENABLE_HIGH_SPEED | I2C_STOP_IN_IDLE | I2C_ENABLE_SMB_SUPPORT);
    I2CSetFrequency(busID, GetPeripheralClock(), 400000);
    I2CEnable(busID, TRUE);
}


//START TRANSFER
BOOL StartTransfer (unsigned char busID, BOOL restart)
{
    static unsigned flag = false;
    I2C_STATUS  status;  
    
    if(restart) I2CRepeatStart(busID);   // Start = 0;  Restart = 1
    else
    {
        // Wait for the bus to be idle, then start the transfer
        while( !I2CBusIsIdle(busID) );
        if(I2CStart(busID) != I2C_SUCCESS)
        {
            printf("\rStart Error");
            // DelayMs(1000);            
            return FALSE;
        }
    }
    // Wait for START to complete
    do {
        status = I2CGetStatus(busID);
    } while (!(status & I2C_START));    
    // Alternate?
    // while(!INTGetFlag(INT_SOURCE_I2C(busID)) );
    // INTClearFlag(INT_SOURCE_I2C(busID));        
    return TRUE;
}

//TRANSMIT ONE BYTE
BOOL TransmitOneByte(unsigned char busID, unsigned char data)
{
    // Wait for the transmitter to be ready   
    while(!I2CTransmitterIsReady(busID));
     
    //Transmit the byte
    if(I2CSendByte(busID, data) == I2C_MASTER_BUS_COLLISION)
    {
        printf("\rERROR: Master Bus Collision");
        return FALSE;
    }    
    // Wait for the transmission to finish
    while(!I2CTransmissionHasCompleted(busID));    
    return TRUE;
}

//STOP TRANSFER
void StopTransfer(unsigned char busID)
{
    I2C_STATUS  status;
    // Send the Stop signal
    I2CStop(busID);
    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(busID);
        //char chip[16];
        //sprintf(chip,"Stop: %x",status);
        DelayMs(1);
        // print_error(chip);

    } while ( !(status & I2C_STOP) );
}



void print_status(unsigned char busID)
{
    I2C_STATUS  status;
    status = I2CGetStatus(busID);    
    printf("STATUS: %x",status);    
    DelayMs(500);
    I2CClearStatus (busID, I2C_ARBITRATION_LOSS);
}


unsigned char I2CGetACK(unsigned char busID)
{
    switch (busID)
    {
        case I2C1:
            return(I2C1STATbits.ACKSTAT);
            break;
        case I2C2:
            return(I2C2STATbits.ACKSTAT);
            break;
        case I2C3:
            return(I2C3STATbits.ACKSTAT);
            break;
        case I2C4:
            return(I2C4STATbits.ACKSTAT);
            break;
        case I2C5:
            return(I2C5STATbits.ACKSTAT);
            break;
        default:
            return 0;
    }    
}

unsigned char I2CReceiveByte(unsigned char busID, unsigned char NACKflag)
{
unsigned char dataByte = 0;
    switch (busID)
    {
        case I2C1:
            I2C1CONbits.RCEN = 1;
            while (!I2CReceivedDataIsAvailable(busID)); 
            dataByte = I2C1RCV; // Read data byte from buffer
            if (NACKflag) I2C1CONbits.ACKDT = 1; // Send NACK to EEPROM
            else I2C1CONbits.ACKDT = 0; // Send ACK to EEPROM
            I2C1CONbits.ACKEN = 1; 
            while (I2C1CONbits.ACKEN == 1); // Wait till ACK/NACK sequence is over 
            break;
            
        case I2C2:
            I2C2CONbits.RCEN = 1;
            while (!I2CReceivedDataIsAvailable(busID)); 
            dataByte = I2C2RCV; // Read data byte from buffer
            if (NACKflag) I2C2CONbits.ACKDT = 1; // Send NACK to EEPROM
            else I2C2CONbits.ACKDT = 0; // Send ACK to EEPROM
            I2C2CONbits.ACKEN = 1; 
            while (I2C2CONbits.ACKEN == 1); // Wait till ACK/NACK sequence is over 
            break;
            
        case I2C3:
            I2C3CONbits.RCEN = 1;
            while (!I2CReceivedDataIsAvailable(busID)); 
            dataByte = I2C3RCV; // Read data byte from buffer
            if (NACKflag) I2C3CONbits.ACKDT = 1; // Send NACK to EEPROM
            else I2C3CONbits.ACKDT = 0; // Send ACK to EEPROM
            I2C3CONbits.ACKEN = 1; 
            while (I2C3CONbits.ACKEN == 1); // Wait till ACK/NACK sequence is over 
            break;
            
        case I2C4:
            I2C4CONbits.RCEN = 1;
            while (!I2CReceivedDataIsAvailable(busID)); 
            dataByte = I2C4RCV; // Read data byte from buffer
            if (NACKflag) I2C4CONbits.ACKDT = 1; // Send NACK to EEPROM
            else I2C4CONbits.ACKDT = 0; // Send ACK to EEPROM
            I2C4CONbits.ACKEN = 1; 
            while (I2C4CONbits.ACKEN == 1); // Wait till ACK/NACK sequence is over 
            break;
            
        case I2C5:
            I2C5CONbits.RCEN = 1;
            while (!I2CReceivedDataIsAvailable(busID)); 
            dataByte = I2C5RCV; // Read data byte from buffer
            if (NACKflag) I2C5CONbits.ACKDT = 1; // Send NACK to EEPROM
            else I2C5CONbits.ACKDT = 0; // Send ACK to EEPROM
            I2C5CONbits.ACKEN = 1; 
            while (I2C5CONbits.ACKEN == 1); // Wait till ACK/NACK sequence is over 
            break;
        default:
            break;            
    }
    return dataByte;
}

/*******************************************************************
 *	Name:	EepromWriteBlock()
 *	
 *	Inputs:
 *	device - I2C address of the chip
 *	startAddress - 16-bit memory address to write the data to
 *	*ptrData - pointer to data string to be stored
 *	numBytes - numBytes - Number of bytes to be written
 *	
 *  Returns: 1 if successful, 0 if EEPROM does not ACKnowledge a command.
 * 
 *	A block of data with length numBytes is sent to the EEPROM.  
 *  Data is written one byte at a time
 *	beginning at startAddress using function I2CSendByte(I2C1, ).
 *	As each byte is written, an ACKnowledge bit is read from EEPROM
 *	
 *******************************************************************/
unsigned char EepromWriteBlock(unsigned char busID, unsigned char device, unsigned short startAddress, unsigned char *ptrData, unsigned short numBytes) 
{
    unsigned char addressHigh, addressLow;
    unsigned short i;    
    
    EEPROM_WP = 0;
    DelayMs(10);

    addressHigh = (unsigned char) ((startAddress & 0xFF00) >> 8);
    addressLow = (unsigned char) (startAddress & 0x00FF);

    StartTransfer(busID, false);

    if (!TransmitOneByte(busID, device | WR)) return 0;
    if (I2CGetACK(busID)) return 0; // Get ACK from EEPROM
    //if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM

    if (!TransmitOneByte(busID, addressHigh)) return 0;
    if (I2CGetACK(busID)) return 0; // Get ACK from EEPROM
    //if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM    

    if (!TransmitOneByte(busID, addressLow)) return 0;
    if (I2CGetACK(busID)) return 0; // Get ACK from EEPROM
    //if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM    


    // Now send block of data
    for (i = 0; i < numBytes; i++) {
        if (!TransmitOneByte(busID, ptrData[i])) return 0;
        if (I2CGetACK(busID)) return 0; // Get ACK from EEPROM
        //if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM        
    }
    StopTransfer(busID);
    EEPROM_WP = 1;

    return (1);
}

/* Reads a block of bytes from EEPROM beginning at startAddress
 *	Inputs:
 *	device - I2C address of the chip
 *	startAddress - 16-bit memory address to write the data to 	
 *	numBytes - numBytes - Number of bytes to be written
 * 
 *  Output: *ptrData - pointer to data read from EEPROM 
 *  Returns: 1 if successful, 0 if EEPROM does not ACKnowledge a command.
 */
unsigned char EepromReadBlock(unsigned char busID, unsigned char device, unsigned short startAddress, unsigned char *ptrData, unsigned char numBytes) 
{
    unsigned char addressHigh, addressLow;
    unsigned char i;

    addressHigh = (unsigned char) ((startAddress & 0xFF00) >> 8);
    addressLow = (unsigned char) (startAddress & 0x00FF);    

    INTClearFlag(INT_SOURCE_I2C(busID));
    
    StartTransfer(busID, false);

    if (!TransmitOneByte(busID, device | WR)) return 0; // Send WRITE command and I2C device address
    if (I2CGetACK(busID)) return 0; // Get ACK from EEPROM
    //if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM

    if (!TransmitOneByte(busID,  addressHigh)) return 0; // Send EEPROM high address byte
    if (I2CGetACK(busID)) return 0; // Get ACK from EEPROM
    //if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM    

    if (!TransmitOneByte(busID, addressLow)) return 0; // Send EEPROM low address byte
    if (I2CGetACK(busID)) return 0; // Get ACK from EEPROM
    //if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM    

    StartTransfer(busID, true); // Now send START sequence again:
    //while(!INTGetFlag(INT_SOURCE_I2C(busID)) );
    //INTClearFlag(INT_SOURCE_I2C(busID));
    
    if (!TransmitOneByte(busID, device | RD)) return 0; // Now send ID with READ Command    
    if (I2CGetACK(busID)) return 0; // Get ACK from EEPROM
    // if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM

    // Now receive block of data:
    for (i = 0; i < numBytes; i++) 
    {
        if (i < numBytes - 1) ptrData[i] = I2CReceiveByte(busID, false);            
        else ptrData[i] = I2CReceiveByte(busID, true);
    }
    StopTransfer(busID);
    return (1); // Return 1 to indicate successful read operation
}

/*******************************************************************
 *	Name:	EepromWriteByte()
 *	
 *	Inputs:
 *	device - I2C address of the chip
 *	startAddress - 16-bit memory address to write the data to
 *	*ptrData - pointer to data string to be stored
 *	numBytes - numBytes - Number of bytes to be written
 *	
 *  Returns: 1 if successful, 0 if EEPROM does not ACKnowledge a command.
 * 
 *	A block of data with length numBytes is sent to the EEPROM.  
 *  Data is written one byte at a time
 *	beginning at startAddress using function I2CSendByte(I2C1, ).
 *	As each byte is written, an ACKnowledge bit is read from EEPROM
 *	
 *******************************************************************/
unsigned char EepromWriteByte(unsigned char busID, unsigned char device, unsigned short address, unsigned char data)
{
    unsigned char addressHigh, addressLow;    
    
    EEPROM_WP = 0;
    DelayMs(10);

    addressHigh = (unsigned char) ((address & 0xFF00) >> 8);
    addressLow = (unsigned char) (address & 0x00FF);
   
    StartTransfer(busID, false);

    if (!TransmitOneByte(busID, device | WR)) return 0;
    if (I2CGetACK(busID)) return 0; // Get ACK from EEPROM
    // if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM
    
    if (!TransmitOneByte(busID, addressHigh)) return 0; // Send EEPROM high address byte
    if (I2CGetACK(busID)) return 0; // Get ACK from EEPROM
    // if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM    

    if (!TransmitOneByte(busID, addressLow)) return 0; // Send EEPROM low address byte
    if (I2CGetACK(busID)) return 0; // Get ACK from EEPROM
    // if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM    
    
    if (!TransmitOneByte(busID, data)) return 0;
    while(!I2CTransmissionHasCompleted(I2C1));
    if (I2CGetACK(busID)) return 0; // Get ACK from EEPROM
    // if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM        

    StopTransfer(busID);
    EEPROM_WP = 1;
    
    return (1);
}

/*  Reads a single byte from EEPROM at address
 *	Inputs:
 *	device - I2C address of the chip
 *	address - 16-bit memory address to write the data to 	
 * 
 *  Output: *ptrData - pointer to data read from EEPROM 
 *  Returns: 1 if successful, 0 if EEPROM does not ACKnowledge a command.
 */

unsigned char EepromReadByte (unsigned char busID, unsigned char device, unsigned short address, unsigned char *ptrData) {
    unsigned char addressHigh, addressLow;
    unsigned char i;
    

    addressHigh = (unsigned char) ((address & 0xFF00) >> 8);
    addressLow = (unsigned char) (address & 0x00FF);

    StartTransfer(busID, false); // Send I2C START
    
    if (!TransmitOneByte(busID, device | WR)) return 0; 
    if (I2CGetACK(busID)) return 0; // Get ACK from EEPROM
    //if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM
    
    if (!TransmitOneByte(busID, addressHigh)) return 0; // Send EEPROM high address byte
    if (I2CGetACK(busID)) return 0; // Get ACK from EEPROM
    //if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM    
    

    if (!TransmitOneByte(busID, addressLow)) return 0; // Send EEPROM low address byte
    if (I2CGetACK(busID)) return 0; // Get ACK from EEPROM
    //if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM    

    StartTransfer(busID, true); // Send I2C RESTART

    if (!TransmitOneByte(busID, device | RD)) return 0; // Now send READ command and I2C device address
    if (I2CGetACK(busID)) return 0; // Get ACK from EEPROM
    //if (I2CSTATbits.ACKSTAT) return (0); // Get ACK from EEPROM
    
    *ptrData = I2CReceiveByte(busID, true);

    StopTransfer(busID);
    return (1); // Return 1 to indicate successful read operation
}

