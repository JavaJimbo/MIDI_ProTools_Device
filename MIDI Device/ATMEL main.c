/*********************************************************************************************************************
 * FileName:    main.c LED USB Robotnik
 *              Compiled with XC32 V1.33
 *
 * 2-21-15  JBS: Adapted from Microchip Solutions v2012-02-15
 *              USB/Device - CDC - Serial Emulator for PIC32 MX795 PIM
 * 2-23-15  JBS: Got Timer 1 interrupts working with USB - colorbars look OK.
 * 2-23-15  JBS: Got SD card routines working - can read Marilyn from SD card.
 * 2-23-15  JBS: USB working
 * 2-25-15  JBS: Works with USB & CRC & Decoding, displays video images!!!!
 * 4-22-15  JBS: Swapped in Timer 2-3. Renamed RESOLUTION as COLORDEPTH.
 * 4-28-15  JBS: Removed MATRIX routines
 * 5-1-15:  Tested Atmel. Send RC servo commands
 * 5-5-15:  Pretty much back to the way it was. RECORD, PLAY, PAUSE. STOP TICK memorized.
 * 5-8-15:  Spacebar pause during record writes to Atmel.
 *          Version 1 for commands and modes works fine.
 *          Straigtened out state machine.
 *          Added doCommand flag. Modes: STANDBY, RECORD, RECORD_PAUSE, PLAY, PLAY_PAUSE
 *          Renamed Atmel functions to RAM and FLASH
 *          Changed boardID and deviceID to boardNum and deviceNum
 *          Used pointers for pots
 *          Added REHEARSE mode.
 * 5-9-15:  Store pot config data.
 * 5-11-15: Finished up pot assignment storage.
 * 5-13-15  New Atmel reads/writes works for any size LINELENGTH and uses split pages, no headers.
 *          Uses RAM1 and RAM2 arrays instead of single circular buffer, no even / odd flags.
 *          Atmel buffer #2 implemented but not used.
 *          Fixed fetch/store rollover issue.
 *          LAST ATTEMPT: Cleaned up fetch/store use byteIndex == LINELENGTH as outer loop.
 * 5-14-15  Switched ADC to polling, eliminated arrADdata[].
 *          Added routines for arming pots.
 * 5-15-15  Use Timer 3 as 120 Hz clock. Divide by four to get tickCount
 *          Timer 3 disabled for now. Using MIDI MTC from TRAKTION 4 as 120 Hz clock.
 * 5-17-15  `A Created global MIDIclockSource flag.
 * 5-26-15  Modified buildXBEEpacket() and sendXBEEpacket() to send individual packets for each board.
 *          AD converter creates interrupts again.
 *          MAXDEVICES 4, MAXBOARDS 4, MAXPOTS 4
 * 5-27-15  Substituted numDevices for MAXDEVICES. Defined LINELENGTH as a constant variable.
 *
 *          Byte #0: BOARD ID
 *          Byte #1: First device being updated
 *          Byte #2: Number of devices being updated
 *          Byte #3: First data byte
 *
 *          Works well with only Board #1 in use with four pots.
 *
 * 5-28-15  Uses Obey input in place of pots. MAXDEVICES set to 6.
 *          Works well with RC Servo and DMX Xbee Controller.
 *          Record, Play, Rehearse seem to work.
 *          buildXBEEpacket() sends updated servos and minimum devices as necessary.
 * 
 **********************************************************************************************************************/
#include "usb.h"
#include "usb_function_cdc.h"
#include "HardwareProfile.h"
#include "uart2.h"
#include "Delay.h"
#include "Drawing.h"
#include "TimeDisplay.h"
#include "FSIO.h"
#include "Defs.h"
#include "AT45DB161.h"
#include "MiscFunctions.h"
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "usb_device.h"
#include "usb.h"
#include <ctype.h>
#include <string.h>

#pragma config UPLLEN   = ON        // USB PLL Enabled
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx2      // ICE/ICD Comm Channel Select

#define START 1
#define STOP 2
#define RUN 0

const unsigned char arrDeviceCount[MAXBOARDS+1] = {0, MAXDEVICES, 0, 0, 0};
const unsigned short LINELENGTH = 0 + MAXDEVICES + 0 + 0;

void CheckPots(void);


unsigned char frames = 0, seconds = 0, minutes = 0;
unsigned char MIDIcommandChar = 0;
unsigned char MIDIclockSource = FALSE;  // Selects MIDI or TIMER 3 fot tick control source. `A
ticktype tickCounter = 0;

unsigned char AtmelRAM1[PAGESIZE];
unsigned char AtmelRAM2[PAGESIZE];

extern unsigned char commandChar;

ticktype stopTick = 0x0000;
extern unsigned char tick;
unsigned char mode = STANDBY;
unsigned char previousMode = 0;
#define MAXVALUES 16
unsigned short commandValues[MAXVALUES];
unsigned short numValues = 0;

#define RS485uart UART5
#define ENDSECTOR (SECTOR_SIZE * 2)

/** V A R I A B L E S ********************************************************/
unsigned long redAdjust, greenAdjust, blueAdjust, brightAdjust;


unsigned short crc_tab16[256] = {0x0, 0xC0C1, 0xC181, 0x140, 0xC301, 0x3C0, 0x280, 0xC241,
    0xC601, 0x6C0, 0x780, 0xC741, 0x500, 0xC5C1, 0xC481, 0x440,
    0xCC01, 0xCC0, 0xD80, 0xCD41, 0xF00, 0xCFC1, 0xCE81, 0xE40,
    0xA00, 0xCAC1, 0xCB81, 0xB40, 0xC901, 0x9C0, 0x880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};

unsigned long getLongInteger(unsigned char b0, unsigned char b1, unsigned char b2, unsigned char b3) {

    union {
        unsigned char byte[4];
        unsigned long lngInteger;
    } convert;

    convert.byte[0] = b0;
    convert.byte[1] = b1;
    convert.byte[2] = b2;
    convert.byte[3] = b3;

    return (convert.lngInteger);
}

unsigned short getShort(unsigned char b0, unsigned char b1) {

    union {
        unsigned char byte[2];
        unsigned short integer;
    } convert;

    convert.byte[0] = b0;
    convert.byte[1] = b1;
    return (convert.integer);
}



unsigned short decodePacket(unsigned char *ptrPacket, unsigned char *ptrBitmap);
void processPythonPacket(unsigned char *ptrBuffer);
unsigned short processPacket(unsigned char *ptrBuffer, unsigned short packetLength);
void SendXBEEcommand(unsigned char command, unsigned char subcommand, unsigned short numBytes);
unsigned short update_crc_16(unsigned short crc, unsigned char c);
unsigned char checkCRC(unsigned char *bufferPtr, unsigned short packetLength);
unsigned long getLongInteger(unsigned char b0, unsigned char b1, unsigned char b2, unsigned char b3);
unsigned short getShort(unsigned char b0, unsigned char b1);
unsigned char getBITMAPHeaders(unsigned char *bufferPtr);

unsigned char USBtimeout = 0;
unsigned char USBInStatus = 0;

unsigned char HOURpushFlag = FALSE;
unsigned char MINUTEpushFlag = FALSE;
unsigned char RELEASEDFlag = FALSE;

unsigned short RS485timer = 0;

unsigned char USBOutBuffer[MAXUSBBUFFER];
unsigned char USBInBuffer[MAXBITMAP];
unsigned char USBTimeoutError = FALSE;

unsigned char XBEETxBuffer[MAXBUFFER] = ">10abcd\r";
unsigned char XBEEdataBuffer[MAXBUFFER];
unsigned short XBEERxLength = 0;
unsigned short XBEETxLength = 0;
unsigned char MIDIRxIndex = 0;
unsigned char MIDItimeout = 0;
unsigned char MIDIflag = FALSE;

unsigned char MIDIRxBuffer[MAXBUFFER];
unsigned char RS485TxBuffer[MAXBUFFER];
unsigned char RS485RxBuffer[MAXBUFFER];
unsigned short RS485TxLength = FALSE;
unsigned short RS485RxLength = 0;
unsigned char PrintfBuffer[MAXBUFFER + 1];
unsigned char HOSTTxBuffer[MAXBUFFER];
unsigned char HOSTRxBuffer[MAXBUFFER];
unsigned char HOSTRxBufferFull = FALSE;
unsigned short HOSTTxLength = 0;
unsigned short HOSTRxLength = 0;
unsigned char UARTtimeout = 0;


unsigned char DMXRxBuffer[MAXBUFFER];
unsigned char DMXflag = true;

unsigned char XBEERxBuffer[MAXBUFFER];

char USB_Out_Buffer[CDC_DATA_OUT_EP_SIZE];
char RS232_Out_Data[CDC_DATA_IN_EP_SIZE];

unsigned char NextUSBOut;
unsigned char NextUSBOut;
unsigned char LastRS232Out; // Number of characters in the buffer
unsigned char RS232cp; // current position within the buffer
unsigned char RS232_Out_Data_Rdy = 0;
USB_HANDLE lastTransmission;

/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
void ProcessIO(void);
void ProcessUSBInData(void);
void USBDeviceTasks(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
void BlinkUSBStatus(void);
void UserInit(void);
void InitializeUSART(void);
unsigned char initializeHardware(void);

void putcUSART(char c);
unsigned char getcUSART();

unsigned char *ptrPot[MAXPOTS];
potType arrPots[MAXPOTS];
boardType arrBoards[MAXBOARDS+1];

unsigned short getPageNum(ticktype tickNum) {
    return (((tickNum * LINELENGTH) / PAGESIZE) + PAGEOFFSET);
}

unsigned short getByteIndex(ticktype tickNum) {
    long totalBytes;
    unsigned short byteIndex;

    totalBytes = (tickNum * LINELENGTH);
    byteIndex = (unsigned short) (totalBytes % PAGESIZE);
    return (byteIndex);
}

void initBoards(void) {
    short boardNum, deviceNum, i;

    for (i = 0; i < PAGESIZE; i++)
        AtmelRAM1[i] = AtmelRAM2[i] = 0x00;

    for (boardNum = 0; boardNum < MAXBOARDS; boardNum++) {
        arrBoards[boardNum].numDevices = arrDeviceCount[boardNum];
        for (deviceNum = 0; deviceNum < arrBoards[boardNum].numDevices; deviceNum++) {
            arrBoards[boardNum].arrDevice[deviceNum].position = 127;
            arrBoards[boardNum].arrDevice[deviceNum].previous = 0;
            arrBoards[boardNum].arrDevice[deviceNum].newdata = FALSE;
        }
    }

    ptrPot[0] = &(arrBoards[0].arrDevice[0].position);
    ptrPot[1] = &(arrBoards[0].arrDevice[1].position);
    ptrPot[2] = &(arrBoards[0].arrDevice[2].position);
    ptrPot[3] = &(arrBoards[0].arrDevice[3].position);

    boardNum = 0;
}

unsigned char writeShortToAtmel(unsigned short pageNum, unsigned short address, unsigned short intShort) {

    union {
        unsigned char byte[2];
        unsigned short integer;
    } convert;
    if (address < (PAGESIZE - 1)) {
        TransferFLASHtoRAM(pageNum, 1); // Copy Atmel flash memory to Atmel RAM buffer #1
        ReadAtmelRAM(AtmelRAM1, 1); // Copy Atmel RAM to data array
        convert.integer = intShort;
        AtmelRAM1[address] = convert.byte[0];
        AtmelRAM1[address + 1] = convert.byte[1];
        EraseFLASHpage(pageNum);
        WriteAtmelRAM(AtmelRAM1, 1);
        ProgramFLASH(pageNum, 1);
        return (TRUE);
    } else return (FALSE);
}

unsigned short readShortFromAtmel(unsigned short pageNum, unsigned short address) {
    unsigned short intShort;

    if (address < (PAGESIZE - 1)) {
        TransferFLASHtoRAM(pageNum, 1); // Copy Atmel flash memory to Atmel RAM
        ReadAtmelRAM(AtmelRAM1, 1); // Copy Atmel RAM to data array
        intShort = getShort(AtmelRAM1[address], AtmelRAM1[address + 1]);
        return (intShort);
    } else return (0x0000);
}



/* Retrieves device data from Atmel memory
 *
 */
void fetchMemoryData(unsigned short pageNum, unsigned short firstByte, unsigned char startStop) {
    unsigned short i, byteIndex, boardNum, deviceNum;
    static unsigned char *ptrFetchRAM = AtmelRAM1;

    if (startStop == START || firstByte == 0) {
        ptrFetchRAM = AtmelRAM1; // If starting play or record, begin with data array #1
        TransferFLASHtoRAM(pageNum, 1); // Transfer Atmel flash memory to Atmel RAM
        ReadAtmelRAM(ptrFetchRAM, 1); // Copy Atmel RAM to data array
        // printf("\rSTART LOAD page %d", pageNum);
    }

    boardNum = 0;
    deviceNum = 0;
    byteIndex = firstByte;
    for (i = 0; i < LINELENGTH; i++) {
        if (byteIndex == PAGESIZE) { // If end of page is reached,
            byteIndex = 0;
            if (ptrFetchRAM == AtmelRAM1) ptrFetchRAM = AtmelRAM2;
            else ptrFetchRAM = AtmelRAM1;
            TransferFLASHtoRAM(pageNum + 1, 1); // Transfer Atmel flash memory to Atmel RAM
            ReadAtmelRAM(ptrFetchRAM, 1); // Copy Atmel RAM to data array
            // printf("\rLOAD page %d", pageNum + 1);
        }
        while (boardNum < MAXBOARDS) {
            if (deviceNum >= arrBoards[boardNum].numDevices) {
                deviceNum = 0;
                boardNum++;
            }
            else {
                arrBoards[boardNum].arrDevice[deviceNum++].position = ptrFetchRAM[byteIndex++];
                break;
            }
        }
    }
}


/* Stores new device data in Atmel memory
 *
 */
void storeDeviceData(unsigned short pageNum, unsigned short firstByte, unsigned char startStop) {
    unsigned short i, byteIndex, boardNum, deviceNum;
    static unsigned char *ptrStoreRAM = AtmelRAM1;

    if (startStop == START || firstByte == 0) ptrStoreRAM = AtmelRAM1;

    boardNum = 0;
    deviceNum = 0;
    byteIndex = firstByte;
    for (i = 0; i < LINELENGTH; i++) {
        if (byteIndex == PAGESIZE) { // If end of page is reached,
            byteIndex = 0;
            EraseFLASHpage(pageNum); // If recording, erase flash before programing
            WriteAtmelRAM(ptrStoreRAM, 1); // Copy data array #1 to Atmel RAM
            ProgramFLASH(pageNum, 1); // Program buffer #1 to flash
            // printf("\rSTORE page %d", pageNum);
            if (ptrStoreRAM == AtmelRAM1) ptrStoreRAM = AtmelRAM2;
            else ptrStoreRAM = AtmelRAM1;
        }
        while (boardNum < MAXBOARDS) {            
            if (deviceNum >= arrBoards[boardNum].numDevices) {
                deviceNum = 0;
                boardNum++;
            }
            else {
                ptrStoreRAM[byteIndex++] = arrBoards[boardNum].arrDevice[deviceNum++].position;
                break;
            }
        }        
    }

    if (startStop == STOP || byteIndex == PAGESIZE) {
        EraseFLASHpage(pageNum); // If recording, erase flash before programing
        WriteAtmelRAM(ptrStoreRAM, 1); // Copy data array #1 to Atmel RAM
        ProgramFLASH(pageNum, 1); // Program buffer #1 to flash
        // printf("\rSTORE page %d", pageNum);
    }
}

// This routine records new data.
// Page number and byte location is calculated
// from current tick, existing data is fetched,
// data is overwritten with new pot values,
// packet uis constructed and sent to boards.
// The startStop input can be set to STOP to
// make sure most recent data is memorized.

unsigned char recordDeviceData(ticktype tickNum, unsigned char startStop) {
    unsigned short i;
    unsigned short byteNum, pageNum = 0;

    pageNum = getPageNum(tickNum);
    byteNum = getByteIndex(tickNum);
    
    fetchMemoryData(pageNum, byteNum, startStop);
    CheckPots();    
    
    storeDeviceData(pageNum, byteNum, startStop);
    return (TRUE);
}

// This routine plays back recorded data.
// Page number and byte location is calculated
// from current tick, existing data is fetched,
// packet is constructed and sent to boards.
// The startStop input can be set to START to
// load preset data before play back begins.
unsigned char playDeviceData(ticktype tickNum, unsigned char startStop) {
    unsigned short i, byteNum, pageNum = 0;

    pageNum = getPageNum(tickNum);
    byteNum = getByteIndex(tickNum);

    fetchMemoryData(pageNum, byteNum, startStop);
    return (TRUE);
}


// Parse input string for commands and values
unsigned char getCommand(unsigned char *ptrString) {
    unsigned char command = 0, *ptrCommand[MAXVALUES + 1];
    unsigned short i = 0, j = 0;
    char *sep_tok = " \r\0";

    // Check whether input string exists
    if (ptrString == NULL) return (0);
    i = 0;
    numValues = 0;

    // Copy command string and values, if any
    ptrCommand[i] = strtok(ptrString, sep_tok);
    while (ptrCommand[i] != NULL && i < MAXVALUES) {
        i++;
        ptrCommand[i] = strtok(NULL, sep_tok);
    }
    // Check for valid command. If there isn't one, quit.
    if (!strcmp(ptrCommand[0], "POT"))
        command = SETPOT;
    else return (0);
    j = 1;
    // If there are values after the command, convert them to integers:
    while (j < i) {
        commandValues[j - 1] = atoi(ptrCommand[j]);
        j++;
    }
    // Record number of value arguments
    numValues = j;
    // Return command
    return (command);
}

void writeAtmelDefaultData(void) {
    unsigned short i = 0;

    for (i = 0; i < PAGESIZE; i++) AtmelRAM1[i] = ' ';

    strcpy(AtmelRAM1, ">POT 0 0 0 1\r");
    strcat(AtmelRAM1, ">POT 1 0 1 1\r");
    strcat(AtmelRAM1, ">POT 2 0 2 1\r");
    strcat(AtmelRAM1, ">POT 3 0 3 1\r");

    EraseFLASHpage(CONFIG_PAGE1);
    WriteAtmelRAM(AtmelRAM1, 1);
    ProgramFLASH(CONFIG_PAGE1, 1);
}

void loadAtmelConfigData(void) {
    unsigned short i = 0, j = 0;
    unsigned char *ptrCommand[MAXVALUES + 1];
    unsigned char command;
    char *sep_tok = ">";
    unsigned short potNum, boardNum, deviceNum;
    unsigned char armed = 0;

    stopTick = readShortFromAtmel(CONFIG_PAGE, STOP_ADDR);
    TransferFLASHtoRAM(CONFIG_PAGE1, 1);
    ReadAtmelRAM(AtmelRAM1, 1);

    i = 0;
    ptrCommand[i] = strtok(AtmelRAM1, sep_tok);
    while (ptrCommand[i] != NULL && i < MAXPOTS) {
        ptrCommand[++i] = strtok(NULL, sep_tok);
    }

    for (j = 0; j < i; j++) {
        command = getCommand(ptrCommand[j]);
        potNum = commandValues[0];
        boardNum = commandValues[1];
        deviceNum = commandValues[2];
        armed = commandValues[3];
        if (boardNum < MAXBOARDS && deviceNum < arrBoards[boardNum].numDevices && potNum < MAXPOTS) {
            ptrPot[potNum] = &(arrBoards[boardNum].arrDevice[deviceNum].position);
            arrPots[potNum].boardNum = boardNum;
            arrPots[potNum].deviceNum = deviceNum;
            arrPots[potNum].armed = armed;
        } else printf("\rPOT LOAD ERROR");
    }
}



int main(void) {
    unsigned char command = 0;
    unsigned short i;    
    unsigned short boardNum = 1, potNum, deviceNum;
    unsigned char strText[MAXSTRING];
    unsigned char previousMode = 0;
    unsigned char armed = 0;
    unsigned char startFlag = TRUE;
        
    for (i = 0; i < MAXPOTS; i++)
        ptrPot[i] = NULL;

    InitializeSystem();
    initializeHardware();
    initBoards();

#if defined(USB_INTERRUPT)
    USBDeviceAttach();
#endif

    DelayMs(100);
    initAtmelSPI();
    AtmelBusy(1);  

    /*
    printf("\rTESTING MIDI");
    while(1){
        if (MIDIflag){
            MIDIflag = FALSE;
            putchar ('\r');
            for (i = 0; i < MIDIRxIndex; i++){
                ch = MIDIRxBuffer[i];
                if ((ch & 0x80) != 0) putchar('\r');
                printf("%x ", ch);
            }
            MIDIRxIndex = 0;
        }
    }*/
    
    // writeAtmelDefaultData();
    loadAtmelConfigData();    

    mode = STANDBY;
    previousMode = STANDBY;
    // printf ("\rSTART - STANDBY MODE");
    printf ("\r\r\rTESTING DMX POTS");

    /*
    while (1) {
        if (DMXflag){
            DMXflag = FALSE;
            printf ("\r");
            for (i = 0; i < DMXLENGTH; i++){
                ch = DMXRxBuffer[i];
                printf ("%d ", ch);
            }
        }
    }
    */
    
    while (1) {

        if (HOSTRxBufferFull) {
            HOSTRxBufferFull = FALSE;
            commandChar = 0;
            printf("\rReceived: %s", HOSTRxBuffer);
            command = getCommand(HOSTRxBuffer);
            if (command) {
                switch (command) {
                    case SETPOT:
                        potNum = commandValues[0];
                        boardNum = commandValues[1];
                        deviceNum = commandValues[2];
                        armed = commandValues[3];
                        
                        if (boardNum < MAXBOARDS && deviceNum < arrBoards[boardNum].numDevices && potNum < MAXPOTS) {
                            ptrPot[potNum] = &(arrBoards[boardNum].arrDevice[deviceNum].position);
                            arrPots[potNum].boardNum = boardNum;
                            arrPots[potNum].deviceNum = deviceNum;
                            arrPots[potNum].armed = armed;

                            for (i = 0; i < PAGESIZE; i++) AtmelRAM1[i] = '\0';
                            for (i = 0; i < MAXPOTS; i++) {
                                sprintf(strText, ">POT %d %d %d %d\r", i, arrPots[i].boardNum, arrPots[i].deviceNum, arrPots[i].armed);
                                strcat(AtmelRAM1, strText);
                                if (arrPots[i].armed) printf("\rPOT %d ARMED: BOARD #%d DEVICE #%d", i, arrPots[i].boardNum, arrPots[i].deviceNum);
                                else printf("\rPOT %d NOT ARMED: BOARD #%d DEVICE #%d", i, arrPots[i].boardNum, arrPots[i].deviceNum);
                            }
                            // printf("\rATMEL RAM CONFIG PAGE 1: %s", AtmelRAM1);
                            EraseFLASHpage(CONFIG_PAGE1);
                            WriteAtmelRAM(AtmelRAM1, 1);
                            ProgramFLASH(CONFIG_PAGE1, 1);
                        } else printf("\rERROR STRTEXT LENGTH EXCEEDED");
                        break;
                    default:
                        printf("\rCommand not recognized");
                        break;
                }
            } else printf("\rCommand not recognized");
        } else if (commandChar) {
            switch (commandChar) {
                case 'A':
                    potNum = HOSTRxBuffer[0] - '0';
                    if (potNum >= 0 && potNum < MAXPOTS){
                        arrPots[potNum].armed = 1;
                        printf("\rPOT #%d ARMED", potNum);
                    }
                    else printf("\rBAD COMMAND");
                    break;

                case 'C': // CLOCK SOURCE COMMAND   `A
                    // If C command includes an "M", enabled MIDI clock
                    // and disable Timer 3 interrupts
                    if (HOSTRxBuffer[0]=='M'){
                        printf("\rMIDI clock enabled");
                        MIDIclockSource = TRUE;
                        ConfigIntTimer3(T3_INT_OFF | T3_INT_PRIOR_2);
                        OpenTimer3(T3_OFF | T3_SOURCE_INT | T3_PS_1_16, 41667);
                    }
                    
                    // and enable Timer 3 interrupts
                    else if (HOSTRxBuffer[0]=='T'){
                        printf("\rTIMER clock enabled");
                        MIDIclockSource = FALSE;
                        ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2);
                        OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_16, 41667);
                    }
                    // If C command includes neither "M" nor "T",
                    // just display clock source, don't change it:
                    else if (MIDIclockSource)
                        printf("\rMIDI CLOCK SOURCE ENABLED");
                    else printf("\rTIMER 3 CLOCK SOURCE ENABLED");
                    break;

                case 'D':
                    potNum = HOSTRxBuffer[0] - '0';
                    if (potNum >= 0 && potNum < MAXPOTS){
                        arrPots[potNum].armed = 0;
                        printf("\rPOT #%d NOT ARMED", potNum);
                    }
                    else printf("\rBAD COMMAND");
                    break;

                case 'Q':
                    printf ("\r\r");
                    for (i = 0; i < MAXPOTS; i++) {                        
                        if (arrPots[i].armed) printf("\rPOT %d ARMED: BOARD #%d DEVICE #%d", i, arrPots[i].boardNum, arrPots[i].deviceNum);
                        else printf("\rPOT %d NOT ARMED: BOARD #%d DEVICE #%d", i, arrPots[i].boardNum, arrPots[i].deviceNum);
                    }
                    break;

                case 'K':
                    for (potNum = 0; potNum < MAXPOTS; potNum++)
                        arrPots[potNum].armed = 0;
                    printf("\rALL POTS OFF");
                    break;

                case 'R':
                    if (mode != PLAY) {
                        mode = RECORD_PAUSE;
                        printf("\rRECORD PAUSE");
                        tickCounter = 0;
                        frames = seconds = minutes = 0;
                    }
                    break;
                    
                case 'O':
                    break;

                case 'P':
                    if (mode != RECORD) {
                        mode = PLAY_PAUSE;
                        printf("\rPLAY PAUSE");
                        tickCounter = 0;
                        frames = seconds = minutes = 0;
                    }
                    break;
                case 'X':
                    if (mode == PLAY_PAUSE || mode == RECORD_PAUSE || mode == REHEARSE) {
                        mode = STANDBY;
                        printf("\rSTANDBY");
                    }
                    break;

                case 'S':
                    if (mode != RECORD && mode != PLAY) {
                        mode = REHEARSE;
                        printf("\rREHEARSE");
                    }
                    break;

                case SPACEBAR:
                    if (mode == RECORD) {
                        mode = RECORD_PAUSE;
                        printf("\rRECORD PAUSE");
                    } else if (mode == RECORD_PAUSE) {
                        mode = RECORD;
                        printf("\rRECORD");
                    } else if (mode == PLAY) {
                        mode = PLAY_PAUSE;
                        printf("\rPLAY PAUSE");
                    } else if (mode == PLAY_PAUSE) {
                        mode = PLAY;
                        printf("\rPLAY");
                    }
                    break;

                case 'E':
                    if (mode == STANDBY) {
                        printf("\rErasing Sector #1. Please Wait...");
                        EraseFLASHsector(1);
                        stopTick = 0x0000;
                        writeShortToAtmel(CONFIG_PAGE, STOP_ADDR, stopTick);
                        printf("DONE");
                    }
                    break;
                case ESCAPE:
                    printf ("\rCommand aborted.");
                    break;
                default:
                    printf("\rUnused command: %c", commandChar);
                    break;
            } // end switch
            commandChar = 0;
        } // end if (commandChar){
        else if (MIDIcommandChar){
            switch (MIDIcommandChar) {
                case MIDISTOP:
                    if (mode == RECORD)
                        mode = RECORD_PAUSE;
                    else if (mode == PLAY)
                        mode = PLAY_PAUSE;
                    break;
                case MIDISTART:
                    if (mode == RECORD_PAUSE)
                        mode = RECORD;
                    else if (mode == PLAY_PAUSE)
                        mode = PLAY;
                    break;
            }
            MIDIcommandChar = 0;
        }

        if (tick || previousMode != mode) {
            tick = FALSE;
            switch (mode) {
                case RECORD_PAUSE:
                    if (previousMode != RECORD_PAUSE) {
                        if (previousMode == RECORD) recordDeviceData(tickCounter, STOP);
                        else {
                            recordDeviceData(tickCounter, START);
                            startFlag = TRUE;
                        }
                        writeShortToAtmel(CONFIG_PAGE, STOP_ADDR, stopTick);
                        boardNum = 1;
                    }
                    break;
                case RECORD:
                    if (tickCounter > stopTick) stopTick = tickCounter;
                    // if (previousMode != RECORD) recordDeviceData(tickCounter, START);
                    // else
                    if (tickCounter == MAXTICK){
                        recordDeviceData(tickCounter, STOP);
                        writeShortToAtmel(CONFIG_PAGE, STOP_ADDR, stopTick);
                        mode = STANDBY;
                        printf ("\rHALTED - MEMORY END.");
                    }
                    else recordDeviceData(tickCounter, RUN);
                    boardNum = 1;
                    break;
                case PLAY_PAUSE:
                    if (previousMode != PLAY_PAUSE) {
                        if (previousMode == PLAY) playDeviceData(tickCounter, STOP);
                        else {
                            playDeviceData(tickCounter, START);
                            startFlag = TRUE;
                        }
                        boardNum = 1;
                    }
                    break;
                case PLAY:
                    playDeviceData(tickCounter, RUN);                    
                    if (tickCounter >= stopTick) {
                        mode = STANDBY;
                        printf("\rEND PLAYBACK");
                    }
                    boardNum = 1;
                    break;

                case REHEARSE:
                    if (previousMode != REHEARSE) startFlag = TRUE;
                    CheckPots();
                    boardNum = 1;
                    break;

                default:
                    break;
            } // end switch
            
            previousMode = mode;            
        } // if (tick){

        if (boardNum){            
            if (!XBEETxLength){
                XBEETxLength = buildXBEEpacket(boardNum++, startFlag);                
                if (XBEETxLength) sendXBEEpacket();                
                if (boardNum > MAXBOARDS) {
                    boardNum = 0;
                    startFlag = FALSE;
                }
            }
        }

#if defined(USB_POLLING)        
        // Check bus status and service USB interrupts.
        USBDeviceTasks();
#endif

        ProcessUSBInData();
    }//end while
}//end main

/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void) {

    AD1PCFG = 0xFFFF;

    // Configure the PIC32 core for the best performance
    // at the operating frequency. The operating frequency is already set to 
    // 60MHz through Device Config Registers
    // SYSTEMConfigPerformance(60000000);

#if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
#endif

#if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN; // See HardwareProfile.h
#endif

    UserInit();

    USBDeviceInit(); //usb_device.c.  Initializes USB module SFRs and firmware
    //variables to known states.
}//end InitializeSystem

/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 * Note:            
 *
 *****************************************************************************/
void UserInit(void) {
    unsigned char i;
    // InitializeUSART(); 

    // 	 Initialize the arrays
    for (i = 0; i<sizeof (USB_Out_Buffer); i++) {
        USB_Out_Buffer[i] = 0;
    }

    NextUSBOut = 0;
    LastRS232Out = 0;
    lastTransmission = 0;


}//end UserInit

/******************************************************************************
 * Function:        void InitializeUSART(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine initializes the UART to 19200
 *
 * Note:            
 *
 *****************************************************************************/
void InitializeUSART(void) {

    UART2Init();

}//end InitializeUSART

/******************************************************************************
 * Function:        void putcUSART(char c)
 *
 * PreCondition:    None
 *
 * Input:           char c - character to print to the UART
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Print the input character to the UART
 *
 * Note:            
 *
 *****************************************************************************/
void putcUSART(char c) {

    UART2PutChar(c);
}


/******************************************************************************
 * Function:        void mySetLineCodingHandler(void)
 *
 * PreCondition:    USB_CDC_SET_LINE_CODING_HANDLER is defined
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function gets called when a SetLineCoding command
 *                  is sent on the bus.  This function will evaluate the request
 *                  and determine if the application should update the baudrate
 *                  or not.
 *
 * Note:            
 *
 *****************************************************************************/
#if defined(USB_CDC_SET_LINE_CODING_HANDLER)

void mySetLineCodingHandler(void) {
    //If the request is not in a valid range
    if (cdc_notice.GetLineCoding.dwDTERate.Val > 115200) {
        //NOTE: There are two ways that an unsupported baud rate could be
        //handled.  The first is just to ignore the request and don't change
        //the values.  That is what is currently implemented in this function.
        //The second possible method is to stall the STATUS stage of the request.
        //STALLing the STATUS stage will cause an exception to be thrown in the 
        //requesting application.  Some programs, like HyperTerminal, handle the
        //exception properly and give a pop-up box indicating that the request
        //settings are not valid.  Any application that does not handle the
        //exception correctly will likely crash when this requiest fails.  For
        //the sake of example the code required to STALL the status stage of the
        //request is provided below.  It has been left out so that this demo
        //does not cause applications without the required exception handling
        //to crash.
        //---------------------------------------
        //USBStallEndpoint(0,1);
    } else {
        //Update the baudrate info in the CDC driver
        CDCSetBaudRate(cdc_notice.GetLineCoding.dwDTERate.Val);

        //Update the baudrate of the UART

        U2BRG = ((GetPeripheralClock()+(BRG_DIV2 / 2 * line_coding.dwDTERate.Val)) / BRG_DIV2 / line_coding.dwDTERate.Val - 1);
        //U2MODE = 0;
        U2MODEbits.BRGH = BRGH2;
        //U2STA = 0;


    }
}
#endif

/******************************************************************************
 * Function:        void putcUSART(char c)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          unsigned char c - character to received on the UART
 *
 * Side Effects:    None
 *
 * Overview:        Print the input character to the UART
 *
 * Note:            
 *
 *****************************************************************************/

unsigned char getcUSART() {
    char c;


    c = UART2GetChar();

    return c;
}


// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************

void USBCBSuspend(void) {

}

void USBCBWakeFromSuspend(void) {

}

void USBCB_SOF_Handler(void) {
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
}

void USBCBErrorHandler(void) {

}

void USBCBCheckOtherReq(void) {
    USBCheckCDCRequest();
}//end

void USBCBStdSetDscHandler(void) {
    // Must claim session ownership if supporting this request
}//end

void USBCBInitEP(void) {
    CDCInitEP();
}

void USBCBSendResume(void) {
    static WORD delay_count;


    if (USBGetRemoteWakeupStatus() == TRUE) {
        //Verify that the USB bus is in fact suspended, before we send
        //remote wakeup signalling.
        if (USBIsBusSuspended() == TRUE) {
            USBMaskInterrupts();

            //Clock switch to settings consistent with normal USB operation.
            USBCBWakeFromSuspend();
            USBSuspendControl = 0;
            USBBusIsSuspended = FALSE; //So we don't execute this code again,
            //until a new suspend condition is detected.


            delay_count = 3600U;
            do {
                delay_count--;
            } while (delay_count);

            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1; // Start RESUME signaling
            delay_count = 1800U; // Set RESUME line for 1-13 ms
            do {
                delay_count--;
            } while (delay_count);
            USBResumeControl = 0; //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}

BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size) {
    switch ((INT) event) {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED:
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:

            break;
        default:
            break;
    }
    return TRUE;
}

unsigned char initializeHardware(void) {
    //SYSTEMConfigPerformance(GetSystemClock());

    // Turn off JTAG so we get the pins back
    mJTAGPortEnable(false);


    PORTD = 0x0000; // Set matrix output enable high. All other outputs low.
    OEpin = 1;
    PORTSetPinsDigitalOut(IOPORT_D, BIT_0 | BIT_1 | BIT_2 | BIT_3 | BIT_4 | BIT_5 | BIT_6 | BIT_7 | BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_12 | BIT_13);
    DISABLE_RS485();

    // Configure Timer 1 using internal clock, 1:1 prescale
    // If Postscale = 1600 interrupts occur about every 33 uS
    // This yields a refresh rate of about 59 hz for entire display
    // The flicker seems pretty minimal at this rate
    // 1 / 33 us x 8 x 2^6 = 59
    // 8 lines x 32 columns x 6 panels x 6 bit resolution = 9216 writes to PORT D for each refresh!

    // Set up Timer 1 interrupt with a priority of 2
    // ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);
    // OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_1, TIMER1_ROLLOVER);

    // Set up Timer 2 for interrupts every millisecond
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_8, 10000);
    
    // Set up Timer 3 for 120 Hz interrupts
    if (!MIDIclockSource){
        ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2);
        OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_16, 41667);
    }

    PORTSetPinsDigitalOut(IOPORT_B, BIT_0 | BIT_1 | BIT_15);
    PORTClearBits(IOPORT_B, BIT_0 | BIT_1);
    // XBEE_SLEEP_OFF;


    PORTSetPinsDigitalIn(IOPORT_B, BIT_4 | BIT_5);
    mCNOpen(CN_ON, CN6_ENABLE | CN7_ENABLE, CN6_PULLUP_ENABLE | CN7_PULLUP_ENABLE);
    ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);

    PORTSetPinsDigitalOut(IOPORT_E, BIT_8);
    PORTSetBits(IOPORT_E, BIT_8);

    // Set up power down interrupts
    PORTSetPinsDigitalIn(IOPORT_E, BIT_9);
    ConfigINT2(EXT_INT_PRI_2 | RISING_EDGE_INT | EXT_INT_DISABLE);


    // Set up Port C outputs:
    PORTSetPinsDigitalOut(IOPORT_C, BIT_3);
    ATMEL_CS = 1;
    SD_CS = 1;

    // Set up Port G outputs:
    // PORTSetPinsDigitalOut(IOPORT_G, BIT_0);
    // RS485_CTRL = 0;  

    // Set up XBEE at 57600 baud
    UARTConfigure(XBEEuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(XBEEuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(XBEEuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_2);
#define SYS_FREQ 80000000
    UARTSetDataRate(XBEEuart, SYS_FREQ, 57600); 
    UARTEnable(XBEEuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure XBEE Interrupts
    INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(XBEEuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(XBEEuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(XBEEuart), INT_SUB_PRIORITY_LEVEL_0);

    // Set up main UART
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(HOSTuart, SYS_FREQ, 57600); // `A
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART #2 Interrupts
    INTEnable(INT_U2TX, INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);

    // Set up MIDI UART
    UARTConfigure(MIDIuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(MIDIuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(MIDIuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(MIDIuart, SYS_FREQ, 31250); // `A
    UARTEnable(MIDIuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure MIDI Interrupts
    INTEnable(INT_U1TX, INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(MIDIuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(MIDIuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(MIDIuart), INT_SUB_PRIORITY_LEVEL_0);
    
    // Set up RS485 UART
    UARTConfigure(RS485uart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(RS485uart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(RS485uart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_2);
    UARTSetDataRate(RS485uart, SYS_FREQ, 250000);
    UARTEnable(RS485uart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure RS485 Interrupts
    INTEnable(INT_SOURCE_UART_TX(RS485uart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(RS485uart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(RS485uart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(RS485uart), INT_SUB_PRIORITY_LEVEL_0);
    

    // Set up AD converters
    ConfigAd();

    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();

}

/********************************************************************
 * Function:        void ProcessUSBInData(void)
 *
 *******************************************************************/
void ProcessUSBInData(void) {
    unsigned char USBpacketframe[MAXUSBBUFFER];
    unsigned char USBInData[MAXUSBBUFFER];
    unsigned char dataByte;
    static unsigned short numUSBInbytes;    static unsigned short numUSBOutBytes;
    static BOOL testFlag = TRUE;
    short numBytes;
    static short inBufferIndex = 0;
    short i = 0;


    if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) return;

    numUSBInbytes = getsUSBUSART(USBpacketframe, MAXUSBBUFFER);

    if (numUSBInbytes) {
        USBtimeout = 2;
        if (!USBInStatus) inBufferIndex = 0;
        numBytes = decodePacket(USBpacketframe, USBInData);
        if (USBInStatus) {
            for (i = 0; i < numBytes; i++) {
                dataByte = USBInData[i];
                if (inBufferIndex < MAXBITMAP)
                    USBInBuffer[inBufferIndex++] = dataByte;
            }
            if (USBInStatus == USB_DONE) {
                USBtimeout = 0;
                // USBInStatus=processPacket(USBInBuffer, inBufferIndex);
                if (USBInStatus == USB_PACKET_OK) numUSBOutBytes = sprintf(USBOutBuffer, "\rOK!");
                else numUSBOutBytes = sprintf(USBOutBuffer, "\rUSB ERROR: %d, # BYTES: %d", USBInStatus, inBufferIndex);
                inBufferIndex = 0;
            }
        } else numUSBOutBytes = sprintf(USBOutBuffer, "\rPACKET ERROR");

        if (testFlag) {
            testFlag = FALSE;
            TEST_HI;
        } else {
            testFlag = TRUE;
            TEST_LOW;
        }
    }

    if (USBInStatus == USB_TIMEOUT_ERROR)
        numUSBOutBytes = sprintf(USBOutBuffer, "\rPACKET ERROR");

    if (USBUSARTIsTxTrfReady() && numUSBOutBytes) {
        putUSBUSART(USBOutBuffer, numUSBOutBytes);
        numUSBOutBytes = 0;
        USBInStatus = USB_STANDBY;
    }

    CDCTxService();

} //end ProcessUSBInData()

unsigned short update_crc_16(unsigned short crc, unsigned char c) {
    unsigned short tmp, short_c;

    short_c = 0x00ff & (unsigned short) c;
    tmp = crc ^ short_c;
    crc = (crc >> 8) ^ crc_tab16 [tmp & 0xff];

    return crc;
}

unsigned char checkCRC(unsigned char *bufferPtr, unsigned short packetLength) {
    unsigned short i = 0, CRCint;
    unsigned char CRClowByte, CRChighByte;
    unsigned short crcCheck, CRClength;

    if (packetLength >= MAXBITMAP) return (FALSE);

    CRClowByte = bufferPtr[packetLength - 2];
    CRChighByte = bufferPtr[packetLength - 1];
    CRCint = getShort(CRClowByte, CRChighByte);
    CRClength = packetLength - 2;

    crcCheck = 0;
    for (i = 0; i < CRClength; i++)
        crcCheck = update_crc_16(crcCheck, bufferPtr[i]);

    if (crcCheck == CRCint) return (TRUE);
    else return (FALSE);
}

unsigned short decodePacket(unsigned char *ptrPacket, unsigned char *ptrBitmap) {
    unsigned short i, j;
    static unsigned char escapeFlag = FALSE;
    unsigned char ch;

    j = 0;
    for (i = 0; i < MAXUSBBUFFER; i++) {
        ch = ptrPacket[i];
        if (!USBInStatus) {
            if (ch == STX) {
                USBInStatus = USB_INCOMING;
                escapeFlag = FALSE;
            }
        } else {
            if (!escapeFlag) {
                if (ch == STX) {
                    USBInStatus = USB_PACKETERROR;
                    escapeFlag = FALSE;
                    return (0);
                } else if (ch == ETX) {
                    USBInStatus = USB_DONE;
                    return (j);
                } else if (ch == DLE)
                    escapeFlag = TRUE;
                else if (j < MAXBITMAP)
                    ptrBitmap[j++] = ch;
                else {
                    USBInStatus = USB_PACKETERROR;
                    return (0);
                }
            } else {
                escapeFlag = FALSE;
                if (j < MAXBITMAP)
                    ptrBitmap[j++] = ch;
                else {
                    USBInStatus = USB_PACKETERROR;
                    j = 0;
                }
            }
        }
    }

    return (j);
}

/*
void CheckPots(void) {
static unsigned char pot1 = 0;
static unsigned char pot2 = 64;
static unsigned char pot3 = 128;
static unsigned char pot4 = 192;

static unsigned char pot1UP = TRUE;
static unsigned char pot2UP = TRUE;
static unsigned char pot3UP = TRUE;
static unsigned char pot4UP = TRUE;

        if (pot1UP){
            if (pot1 < 255) pot1++;
            else pot1UP = FALSE;
        }
        else {
            if (pot1 > 0) pot1--;
            else pot1UP = TRUE;
        }

        if (pot2UP){
            if (pot2 < 255) pot2++;
            else pot2UP = FALSE;
        }
        else {
            if (pot2 > 0) pot2--;
            else pot2UP = TRUE;
        }

        if (pot3UP){
            if (pot3 < 255) pot3++;
            else pot3UP = FALSE;
        }
        else {
            if (pot3 > 0) pot3--;
            else pot3UP = TRUE;
        }

        if (pot4UP){
            if (pot4 < 255) pot4++;
            else pot4UP = FALSE;
        }
        else {
            if (pot4 > 0) pot4--;
            else pot4UP = TRUE;
        }

        *(ptrPot[0]) = pot1;
        *(ptrPot[1]) = pot2;
        *(ptrPot[2]) = pot3;
        *(ptrPot[3]) = pot4;

        // printf("\r#%d - POT 0: %d, POT 1: %d, POT 2: %d, POT 3: %d", tickCounter, arrPots[0].data, arrPots[1].data, arrPots[2].data, arrPots[3].data);
        // printf("\r#%d - POT 0: %d, POT 1: %d, POT 2: %d, POT 3: %d", tickCounter, *(ptrPot[0]), *(ptrPot[1]), *(ptrPot[2]), *(ptrPot[3]));
        mAD1IntEnable(INT_ENABLED);
} */

void CheckPots(void) {
unsigned short i;

        for (i = 0; i < MAXPOTS; i++) {
            arrPots[i].armed = TRUE;
            if (ptrPot[i] != NULL && arrPots[i].armed)
                // *(ptrPot[i]) = arrPots[i].data;
                *(ptrPot[i]) = DMXRxBuffer[i];
        }
        // printf("\r#%d - POT 0: %d, POT 1: %d, POT 2: %d, POT 3: %d", tickCounter, arrPots[0].data, arrPots[1].data, arrPots[2].data, arrPots[3].data);
        // printf("\r#%d - POT 0: %d, POT 1: %d, POT 2: %d, POT 3: %d", tickCounter, *(ptrPot[0]), *(ptrPot[1]), *(ptrPot[2]), *(ptrPot[3]));
        mAD1IntEnable(INT_ENABLED);
}


/** EOF main.c *************************************************/


