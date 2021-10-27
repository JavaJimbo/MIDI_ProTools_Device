/***********************************************************************************************************
 * ATMEL TEST
 * 6-14-17:     Got Atmel memory working properly.
 * 
 *              
 ************************************************************************************************************/

#ifndef MAIN_C
#define MAIN_C


#define true TRUE
#define false FALSE


/** INCLUDES *******************************************************/
#include <XC.h>

#include "usb.h"
#include "HardwareProfile.h"
#include "Delay.h"
#include "AT45DB161.h"
#include "GenericTypeDefs.h"
#include "Compiler.h"

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

/** CONFIGURATION **************************************************/

#pragma config UPLLEN   = ON        // USB PLL Enabled
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier $$$$
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


/** PRIVATE PROTOTYPES *********************************************/
static void InitializeSystem(void);
void UserInit(void);

#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR


#define MAXBUFFER 128
unsigned char HOSTRxBuffer[MAXBUFFER];
unsigned char HOSTRxBufferFull = false;

int main(void) {
    short i = 0;
    unsigned char outTest[] = "YEH YEH YEH Whatsis do may do but not to you";
    short outTestLength;

    outTestLength = strlen(outTest);
    
    UserInit();     
    
    #define ATMEL_BUFFER 1
    short pageNum = 333;
    short byteAddress = 44;
        
    DelayMs(100);
    printf("\r\rNO WRITE: BUFFER #%d, START BYTE #%d, PAGE #%d", ATMEL_BUFFER, byteAddress, pageNum);
    initAtmelSPI();
    AtmelBusy(1);  
   
    #define PAGESIZE 528    
    unsigned char AtmelRAM1[PAGESIZE];
        
    printf("\rErasing FLASH");
    EraseFLASHpage(pageNum);
    printf("\rWriting to buffer");
    WriteAtmelBytes(ATMEL_BUFFER, outTest, byteAddress , outTestLength);
    printf("\rProgramming flash");
    ProgramFLASH (ATMEL_BUFFER, pageNum);
    
    DelayMs(100);
    printf("\rTransferring flash");
    TransferFLASH(ATMEL_BUFFER, pageNum);
    printf("\rReading buffer");
    ReadAtmelBytes(ATMEL_BUFFER, AtmelRAM1, byteAddress, outTestLength);
    
    printf("\rATMEL RAM1: ");
    for(i = 0; i < outTestLength; i++) printf("%c", AtmelRAM1[i]);
    
    while(1);    
}    



void UserInit(void) {
    //Initialize all of the LED pins
    mInitAllLEDs();

    //Initialize all of the push buttons
    mInitAllSwitches();


    PORTSetPinsDigitalIn(IOPORT_C, BIT_14 | BIT_13);
    mCNOpen(CN_ON, CN0_ENABLE | CN1_ENABLE, CN0_PULLUP_ENABLE | CN1_PULLUP_ENABLE);
    ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);

    // Set up main UART
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
#define SYS_FREQ 80000000
    UARTSetDataRate(HOSTuart, SYS_FREQ, 57600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART #2 Interrupts
    INTEnable(INT_U2TX, INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_DISABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);

    
    PORTSetPinsDigitalOut(IOPORT_G, BIT_0);
    PORTClearBits(IOPORT_G, BIT_0);  // RS485 control pin should be set to receive
    
    PORTSetPinsDigitalIn(IOPORT_C, BIT_14 | BIT_13);
    PORTSetPinsDigitalOut(IOPORT_B, BIT_2);
    PORTSetBits(IOPORT_B, BIT_2);
    
    // Set up Port C outputs:
    PORTSetPinsDigitalOut(IOPORT_C, BIT_3);
    PORTSetBits(IOPORT_C, BIT_3);  
    
    // Set up Port E outputs:
    PORTSetPinsDigitalOut(IOPORT_E, BIT_8);
    PORTSetBits(IOPORT_E, BIT_8);    

    // Turn on the interrupts
    // INTEnableSystemMultiVectoredInt();
    
}//end UserInit

/** EOF main.c *************************************************/
#endif
