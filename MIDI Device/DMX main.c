/**************************************************************************************
 * main.c - DMX XBEE CONTROLLER
 * For PIC32MX340F512H Olimex board
 *
 * 5-20-15 Reads pots. Writes fixed values to DMX dimmers.
 * 5-21-15 
 * 5-28-15 Reads pots, switches to REMOTE when valid XBEE data is recieved.
 *
 * 5-28-15  Simplified interrupt routine, moved stuff into processInBuffer().
 *          Copied interrupt and processInBuffer() RC Servo.
 * 
 **************************************************************************************/
#define _SUPPRESS_PLIB_WARNING
#include "plib.h"
#include "rtcc.h"
#include <stdlib.h> 
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "Delay.h"
#include "HardwareProfile EXP16.h"

#define BOARD_ID 1
#define ANY_BOARD 0x00
#define STX '>'
#define ETX '\r'
#define DLE '/'
#define PLUS  '+'

#define MAXDEVICES 6

#define	DMX_STANDBY	0
#define	DMX_BREAK 	1
#define DMX_MARK 	2
#define DMX_DATA	3

#define MAXPOTS 5

unsigned short inLength = 0;
unsigned char arrPotData[MAXPOTS];
unsigned char DMXstate = DMX_STANDBY;
#define LOCAL 0
#define REMOTE 1
unsigned char mode = LOCAL;

#define STARTBYTE 0x100
#define UARTMASK STARTBYTE

#define true	TRUE
#define false 	FALSE

#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
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

#define MAXBUFFER 64
unsigned char UARTbuffer[MAXBUFFER];
unsigned char DMXTxBuffer[MAXBUFFER] = {64, 128, 192, 255};
unsigned char XBEERxBuffer[MAXBUFFER];
unsigned char INbuffer[MAXBUFFER];
unsigned char servoBuffer[MAXBUFFER];

void ConfigAd(void);
unsigned char processInBuffer(unsigned short inLength);

void CheckPots(void) {
    unsigned short offSet;
    unsigned char i;

    while (!mAD1GetIntFlag());
    mAD1ClearIntFlag();

    // Determine which buffer is idle and create an offset
    offSet = 8 * ((~ReadActiveBufferADC10() & 0x01));

    for (i = 0; i < MAXPOTS; i++)
        arrPotData[i] = (unsigned char) (ReadADC10(offSet + i) / 4); // read the result of channel 0 conversion from the idle buffer
}

union tag {
    unsigned char byte[2];
    unsigned short integer;
} convert;

unsigned char processInBuffer(unsigned short inLength) {
    unsigned char i, j, k, firstServo, numServosUpdated = 0;

    numServosUpdated = XBEERxBuffer[2];
    firstServo = XBEERxBuffer[1];
    for (i = 0; i < numServosUpdated; i++) {
        j = i + 3;
        k = i + firstServo;
        if (j < MAXBUFFER && k < MAXBUFFER)
            servoBuffer[k] = XBEERxBuffer[j];
        else {
            numServosUpdated = 0;
            break;
        }
    }

    if (numServosUpdated) {
        printf("\rUpdated: %d servos", numServosUpdated);
        for (i = 0; i < MAXDEVICES; i++) {
            DMXTxBuffer[i] = servoBuffer[i];
        }
    }

    /*
    unsigned short result;
    unsigned char numBytes;

    convert.byte[0] = INbuffer[inLength - 2];
    convert.byte[1] = INbuffer[inLength - 1];

    numBytes = inLength - 2;
    result = CRCcalculate(INbuffer, numBytes);

    if (result != convert.integer) return (FALSE);  // ERROR: CRC doesn't match
     */
    return (numServosUpdated);
}

int main(void) {
    short i;

    // Enable optimal performance
    SYSTEMConfigPerformance(GetSystemClock());
    mOSCSetPBDIV(OSC_PB_DIV_1); // Use 1:1 CPU Core:Peripheral clocks

    // Turn off JTAG so we get the pins back
    mJTAGPortEnable(false);

    // Set up Timer 2 with interrupts
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_256, 7102);

    // Set up Timer 3 with 1 ms interrupts
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2);
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_4, 10000);

    PORTSetPinsDigitalOut(IOPORT_D, BIT_2);
    PORTSetBits(IOPORT_D, BIT_2);


#define XBEEuart UART2
    // Set up main UART for 57600 baud
    UARTConfigure(XBEEuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(XBEEuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(XBEEuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(XBEEuart, GetPeripheralClock(), 57600);
    UARTEnable(XBEEuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART #2 Interrupts
    INTEnable(INT_U2TX, INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(XBEEuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(XBEEuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(XBEEuart), INT_SUB_PRIORITY_LEVEL_0);

#define DMXuart UART1
    // Set up DMX UART for 250000 baud
    UARTConfigure(DMXuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(DMXuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(DMXuart, UART_DATA_SIZE_9_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(DMXuart, GetPeripheralClock(), 250000);
    UARTEnable(DMXuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure DMX UART Interrupts
    INTEnable(INT_U1TX, INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(DMXuart), INT_DISABLED);
    INTSetVectorPriority(INT_VECTOR_UART(DMXuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(DMXuart), INT_SUB_PRIORITY_LEVEL_0);

    //SYSTEMConfigPerformance(GetSystemClock());
    //mOSCSetPBDIV(OSC_PB_DIV_2);

    // Enable multi-vectored interrupts
    INTEnableSystemMultiVectoredInt();

    // Enable interrupts
    INTEnableInterrupts();

    DelayMs(100);

    PORTSetPinsDigitalOut(IOPORT_B, BIT_0);
    ConfigAd();

    printf("\rTesting XBEE");

    while (1) {
        if (mode == LOCAL) {
            DelayMs(10);
            CheckPots();
            for (i = 0; i < MAXDEVICES; i++) {
                DMXTxBuffer[i] = arrPotData[i];
            }
        } else if (inLength) {
            if (processInBuffer(inLength))
                printf("\r>%d, %d, %d, %d, %d, %d", DMXTxBuffer[0], DMXTxBuffer[1], DMXTxBuffer[2], DMXTxBuffer[3], DMXTxBuffer[4], DMXTxBuffer[5]);
            inLength = 0;
        }
        /*
        } else if (numServosUpdated) {
            printf("\rUpdated: %d servos", numServosUpdated);
            for (i = 0; i < MAXDEVICES; i++) {
                DMXTxBuffer[i] = servoBuffer[i];
            }
            numServosUpdated = 0;
        }
         */
        // printf("\rPOTS: P0: %d, P1: %d, P2: %d, P3: %d", arrPotData[0], arrPotData[1], arrPotData[2], arrPotData[3]);
    }

    return (0);
}

void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void) {

    mT2ClearIntFlag();

    if (DMXstate == DMX_STANDBY) {
        DMXstate = DMX_BREAK; // This is the beginnning of the BREAK
        OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, 9600);
        PORTClearBits(IOPORT_D, BIT_2);
    } else if (DMXstate == DMX_BREAK) { // MARK before transmitting data:
        DMXstate = DMX_MARK;
        OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, 960);
        PORTSetBits(IOPORT_D, BIT_2); // Release transmit line, let it go high to mark end of break
    } else if (DMXstate == DMX_MARK) {
        DMXstate = DMX_DATA;
        INTEnable(INT_U1TX, INT_ENABLED);
        OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_256, 7102);
        while (!UARTTransmitterIsReady(DMXuart));
        U1TXREG = STARTBYTE;
    } else {
        DMXstate = DMX_BREAK; // This is the beginnning of the BREAK
        OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, 9600);
        PORTClearBits(IOPORT_D, BIT_2);
    }
}


//	DMX512 INTERRUPTS
// 	UART 1 interrupt handler for receiving and transmitting DMX512 data
// 	Priority level 2

void __ISR(_UART1_VECTOR, ipl2) IntDMXHandler(void) {
    static unsigned short DMXTxPtr = 0;
    unsigned int ch;

    // RX interrupts
    if (INTGetFlag(INT_SOURCE_UART_RX(DMXuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(DMXuart));
    }

    // TX interrupts:
    if (INTGetFlag(INT_SOURCE_UART_TX(DMXuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(DMXuart));
        if (DMXstate == DMX_DATA) {
            if (DMXTxPtr < MAXDEVICES) {
                ch = DMXTxBuffer[DMXTxPtr];
                DMXTxPtr++;
                while (!UARTTransmitterIsReady(DMXuart));
                U1TXREG = (ch | UARTMASK);
            } else {
                DMXstate = DMX_STANDBY;
                INTEnable(INT_U1TX, INT_DISABLED);
                DMXTxPtr = 0;
            }
        }
    }
}

void ConfigAd(void) {

    mPORTBSetPinsAnalogIn(BIT_3);
    mPORTBSetPinsAnalogIn(BIT_8);
    mPORTBSetPinsAnalogIn(BIT_9);
    mPORTBSetPinsAnalogIn(BIT_10);

    // ---- configure and enable the ADC ----

    // ensure the ADC is off before setting the configuration
    CloseADC10();

    // define setup parameters for OpenADC10
    //                 Turn module on | ouput in integer | trigger mode auto | enable autosample
#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

    // ADC ref external    | disable offset test    | enable scan mode | perform  samples | use dual buffers | use only mux A
#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_5 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF

    //                   use ADC internal clock | set sample time
#define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_31

    //  set inputs to analog
#define PARAM4    ENABLE_AN0_ANA | ENABLE_AN1_ANA| ENABLE_AN2_ANA | ENABLE_AN3_ANA | ENABLE_AN4_ANA

    // Only scan AN0 to AN4, skip the rest
#define PARAM5   SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15

    // set negative reference to Vref for Mux A
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF);

    // open the ADC
    OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5);

    // DO NOT USE INTERRUPTS!!!
    // ConfigIntADC10(ADC_INT_PRI_2 | ADC_INT_SUB_PRI_2 | ADC_INT_ON);

    // clear the interrupt flag
    mAD1ClearIntFlag();

    // Enable the ADC
    EnableADC10();

}

void __ISR(_TIMER_3_VECTOR, ipl2) Timer3Handler(void) {
    mT3ClearIntFlag();


}


// UART 2 interrupt handler set at priority level 2

void __ISR(_UART2_VECTOR, ipl2) IntXBEEHandler(void) {
    UART_LINE_STATUS lineStatus;
    unsigned char BoardID, ch;    
    static unsigned char buffIndex = 0;
    static unsigned char escapeFlag = false;

    lineStatus = UARTGetLineStatus(XBEEuart);
    if (UART_OVERRUN_ERROR & lineStatus) {
        U2STAbits.OERR = 0;
    }

    if (UART_FRAMING_ERROR & lineStatus) {
        ;
    }

    // If preceding character wasn't an escape char,
    // check whether it is STX, ETX or DLE,
    // otherwise store and advance for next char.
    // When ETX is received, check board # for match.
    // If it matches, then copy to buffer to process data.
    if (INTGetFlag(INT_SOURCE_UART_RX(XBEEuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(XBEEuart));

        if (UARTReceivedDataIsAvailable(XBEEuart)) {
            ch = UARTGetDataByte(XBEEuart);
            if (ch == DLE && !escapeFlag)
                escapeFlag = true;
            else if (ch == STX && !escapeFlag)
                buffIndex = 0;
            else if (ch == ETX && !escapeFlag) {
                BoardID = XBEERxBuffer[0];
                if (BoardID == BOARD_ID) {
                    inLength = buffIndex;
                    mode = REMOTE;
                }
                buffIndex = 0;
            } else {
                escapeFlag = false;
                if (buffIndex < MAXBUFFER) XBEERxBuffer[buffIndex++] = ch;
            }
        }


        if (INTGetFlag(INT_SOURCE_UART_TX(XBEEuart))) {
            INTClearFlag(INT_SOURCE_UART_TX(XBEEuart));
        }

    }
}

/*
            // If preceding character wasn't an escape char,
            // check whether it is STX, ETX or DLE,
            // otherwise store and advance for next char.
            // When ETX is received, check board # for match.
            // If it matches, then copy to buffer to process data.
            if (ch == DLE && !escapeFlag)
                escapeFlag = true;
            else if (ch == STX && !escapeFlag)
                j = 0;
            else if (ch == ETX && !escapeFlag) {
                BoardID = XBEERxBuffer[0];
                if (BoardID == DMX_ID) {
                    mode = REMOTE;
                    INlength = j;
                    for (j = 0; j < INlength; j++)
                        INbuffer[j] = XBEERxBuffer[j];
                    j = 0;
                }
            } else {
                escapeFlag = false;
                if (j < MAXBUFFER) XBEERxBuffer[j++] = ch;
            }
 */