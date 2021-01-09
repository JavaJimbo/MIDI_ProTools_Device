/***********************************************************************************************************
 * PROJECT:     MIDI Demo
 * FileName:    main.c for PIC32MX795F512L on Sparkfun UBW32 Board
 *              
 * 11-30-20:    Get both MIDI UART and USB working for recording and playing back using ProTools.
 *              Works well in USB mode with ProTools recording and playing.
 *              Uses MIDI Control Change command "0xBn" to send/receive ten bit pot position
 *              with two MIDI writes. The first send/receive the low five data bits,
 *              the second is for the high five bits. 
 * 
 *              Command format is Bn xx yy where n is channel number, 
 *              xx is servo number, and yy is servo data. If data is low byte, then it is a value from 0-31.
 *              For the high byte, then a value from 1-31 is OR'd with 0b01000000.
 * 
 *              The USER pushbutton on UBW32 board enables AD converter interrupts 
 *              to read ten bit value from pot to use as test data.
 * 
 *              Also a UART #1 can be used as a standard MIDI port at 31250 baud.
 * 12-5-20:     Recompiled and tested with ProTools.
 * 12-6-20:     Modified to send MIDI data for one servo motor on XBEE Uart.
 *              Works with PIC795_MD13S_Controller recording and playing back on XBEE at 57600 baud.
 *              ROM USB_DEVICE_DESCRIPTOR 0x04D8, // Vendor ID   0x0059, // Product ID: Audio MIDI example
 * 12-15-20:    ADDED TEN ANALOG SLIDER POTS: B0,B1,B3,B4,B5,B8,B9,B10,B11,B12
 * 12-16-20:    Debugged & added USBrunning flag.
 * 12-16-20:    Works now with multiple servos recording and playing back on ProTools.
 * 12-20-20:    XBEE baud rate didn't work well at 115200. Set back to 57600.
 * 12-27-20:    Fixed XBEE TxBuffer bug - always transmit character at XBEETxBuffer[XBEETxTail]
 * 12-28-20:    Got MIDI Uart #1 sending and receiving
 * 12-29-20:    
 * 12-30-20:    
 * 12-31-20:    Both MIDI RS232 and USB modes work for recording and playing.
 *              However for RS232 mode, the XBEETxBuffer[] had to be increased in size to over 16000 bytes.
 *              Not sure what the problem is. Maybe the Focusrite USB to MIDI converter
 *              accumulates the MIDI data into big bundles. USB mode might still be the better bet.
 * 1-1-21:      Minor tweaks. Always display overrun errors, even when diagnostics aren't enabled.
 * 1-2-21:      Added MIDI RS232 IO for keyboard. Seems to work better with recording than playing back.
 *              Servo USB control still looks good.
 * 1-3-21:      Swapped in MIDITxPacketType for MIDIDTxBuffer.
 * 1-7-21:      Added DMX512 Rx and Tx.
 * 1-8-21:      Tested DMX512 and servo control with XBEE - everything seems to work.
 * 1-9-21:      Added routine for writing configuration to EEprom.
 *              MIDI channels for DMX and SERVO are stored in EEprom and may be modified.
 ************************************************************************************************************/

#ifndef MAIN_C
#define MAIN_C
//#define DMX_CHANNEL 0xB4
//#define SERVO_CHANNEL 0xB1
// #define KEYBOARD_CHANNEL 0xB0
#define I2C_BUS I2C1

enum {
    MIDI_STANDBY = 0,
    SEND_MIDI_COMMAND,
    SEND_SERVO_ID,
    SEND_SERVO_DATA
};

#define CANCEL_COMMAND 3 // C
#define DIAGNOSTICS 4  // D
#define DMX_ENABLE 24   // X
#define XBEE_ONLY 26  // Z
#define SAVE_CONFIG 19 // S
#define SET_DMX_CHANNEL 22 // V
#define SET_SERVO_CHANNEL 2 // B


#define TEST_OUT LATEbits.LATE4
#define ENTER 13
#define CR ENTER
#define BACKSPACE 8
#define SPACEBAR ' '

#define	STX '>'
#define	DLE '/'
#define	ETX '\r'

#define USER_PUSHBUTTON PORTEbits.RE6
#define PROGRAM_PUSHBUTTON PORTEbits.RE7
#define HIGH_NOTE 0x54
#define LOW_NOTE 0x18

#define true TRUE
#define false FALSE


/** INCLUDES *******************************************************/
#include <plib.h>  // XC.h doesn't seem to work with UART and printf

#include "usb.h"
#include "HardwareProfile.h"
#include "usb_function_midi.h"
#include "Delay.h"

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "I2C_4BUS_EEPROM_PIC32.h"

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

#define RX_BUFFER_ADDRESS_TAG
#define TX_BUFFER_ADDRESS_TAG
#define MIDI_EVENT_ADDRESS_TAG

// BYTE ReceivedDataBuffer[64] RX_BUFFER_ADDRESS_TAG;
BYTE ReceivedDataBuffer[64];
BYTE ToSendDataBuffer[64] TX_BUFFER_ADDRESS_TAG;
USB_AUDIO_MIDI_EVENT_PACKET midiData MIDI_EVENT_ADDRESS_TAG;

USB_HANDLE USBTxHandle = 0;
USB_HANDLE USBRxHandle = 0;

int ADC10_ManualInit(void);
void BlinkUSBStatus(void);
static void InitializeSystem(void);
BYTE ProcessUSB(void);
BYTE ProcessMIDI_RS232(void);
void UserInit(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
BYTE BuildXBEEpacket(BYTE command, BYTE ServoNumber, BYTE numData, short *ptrData);
extern unsigned short CalculateModbusCRC(BYTE *input_str, short num_bytes);

#define MIDIuart UART1
#define MIDIbits U1STAbits
#define MIDI_VECTOR _UART_1_VECTOR

#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR

#define XBEEuart UART4
#define XBEEbits U4STAbits
#define XBEE_VECTOR _UART_4_VECTOR

#define STARTBYTE 0 //0x100
#define UARTMASK STARTBYTE
#define DMX_TX_INT INT_U3TX
#define DMX_RX_INT INT_U3RX

#define	DMX_STANDBY	0
#define	DMX_BREAK 	1
#define DMX_MARK 	2
#define DMX_START 	3
#define DMX_DATA	4
#define DMXLENGTH 	16
#define TICK		10000

#define DMXuart UART3
#define DMXSTAbits U3STAbits
#define DMXTXREG U3TXREG
#define DMX_VECTOR _UART_3_VECTOR

#define RS485uart UART5
#define RS485_RxReg U5RXREG
#define RS485_TxReg U5TXREG
#define RS485bits U5STAbits

#define RS485_VECTOR _UART_5_VECTOR
#define RS485_RX_IRQ _UART5_RX_IRQ
#define RS485_TX_IRQ _UART5_TX_IRQ

#define MAXBUFFER 1024

BYTE DMXRxBuffer[DMXLENGTH];
BYTE DMXTxBuffer[DMXLENGTH];
BYTE PreviousDMXRxBuffer[DMXLENGTH];

BYTE DMXdataReceived = false;
BYTE HOSTRxBuffer[MAXBUFFER];
BYTE HOSTRxBufferFull = false;

#define MAXXBEEBUFFER MAXBUFFER
BYTE XBEETxBuffer[MAXXBEEBUFFER];
unsigned short XBEETxHead = 0, XBEETxTail = 0;
short outData[MAXBUFFER];

BYTE MIDIRxBuffer[MAXBUFFER];
unsigned short MIDIRxHead = 0, MIDIRxTail = 0;

#define MAXPOTS 10
unsigned short ADresult[MAXPOTS];
unsigned short ADpots[MAXPOTS];

void ConfigAd(void);

#define ENABLE_POTS 16
#define SET_DISPLAY 4 // CTL-D
#define SET_SERVO 19 // CTL-S
#define SET_RECORD 18 // CTL-R
#define SET_STANDBY 1 // CTL-A
#define SET_SCALE 3 // CTL-C
#define SET_PLAY 16 // CTL-P
#define SET_PERCUSSION 5 // CTL-E
#define SEND_NOTE_OFF_47 6 // CTL-F
#define SEND_NOTE_OFF_48 7 // CTL-G
#define SET_TEST 20 // CTL-T
#define NUMBER_ERROR 32767

struct servoType {
    BYTE ID;
    unsigned short position;
    BYTE updated;
    BYTE enabled;
};


#define NUM_PERCUSSION_INIT_BYTES 7
const BYTE MIDIpercussionCommand[NUM_PERCUSSION_INIT_BYTES] = {0xB0, 0x00, 0x7F, 0x20, 0x00, 0xC0, 0x00};
unsigned short MIDInoteTimeout = 0;

BYTE ADint = false;
#define FILTERSIZE 16
long arrPotValue[MAXPOTS][FILTERSIZE];
BYTE arrPotEnable[MAXPOTS];
#define MAXSERVOS MAXPOTS
long arrServoValue[MAXSERVOS];
long servoLow = 0, servoHigh = 0;

BYTE setupCommand = 0, controlCommand = 0x00;

void SortPots()
{
    ADpots[0] = ADresult[6];
    ADpots[1] = ADresult[5];
    ADpots[2] = ADresult[4];
    ADpots[3] = ADresult[9];
    ADpots[4] = ADresult[3];
    ADpots[5] = ADresult[2];
    ADpots[6] = ADresult[1];
    ADpots[7] = ADresult[0];
    ADpots[8] = ADresult[7];
    ADpots[9] = ADresult[8];
}

typedef struct MIDIPacketHeader
{
    BYTE MIDICommand;
    BYTE MIDIData1;
    BYTE MIDIData2;    
}MIDITxPacketType;

MIDITxPacketType USBTxBuffer[MAXBUFFER];
short USBTxHead = 0, USBTxTail = 0;

MIDITxPacketType MIDITxBuffer[MAXBUFFER];
unsigned short MIDITxHead = 0, MIDITxTail = 0;


long ActualXBEEBaudrate = 0;
long ActualMIDIBaudrate = 0;
long ActualDMXBaudrate = 0;
short errorCounter = 0;

BYTE MIDIRxOverrunError = false;

unsigned short MIDITxDelay = 0;
static unsigned char DMXstate=DMX_STANDBY;

#define TEST_BYTE_1 0xAB
#define TEST_BYTE_2 0xCD
#define TEST_BYTE_3 0xEF
#define TEST_ADDRESS_1 0
#define TEST_ADDRESS_2 1
#define TEST_ADDRESS_3 2

#define XBEE_ADDRESS 3
#define DMX_ADDRESS 4
#define DIAGNOSTICS_ADDRESS 5
#define DMX_CHANNEL_ADDRESS 6
#define SERVO_CHANNEL_ADDRESS 7

BYTE DMXchannel = 0xB4;
BYTE SERVOchannel = 0xB1;

BYTE XBEEonly = false;
BYTE DMXenable = false;
BYTE DiagnosticsEnable = false;

BYTE CheckConfig();
BYTE WriteConfig();
BYTE ReadConfig();

//BYTE OutString[] = "Let's see if this works...";
//BYTE InString[64];

void DisplayBaudrates()
{
    printf("\rDMX Baud rate: %ld", ActualDMXBaudrate);
    printf("\rXBEE Baud rate: %ld", ActualXBEEBaudrate); 
    printf("\rMIDI Baud rate: %ld", ActualMIDIBaudrate);
}

int main(void) 
{
    BYTE ch = 0;
    short i, j;        
    short filterIndex = 0;
    long averageValue = 0;
    long lampValue = 0;
    long sumValue;    
    BYTE command, ServoNumber;
    BYTE ResetFlag = false;
    BYTE RunFlag = false;        
    BYTE USBactive = false;
    unsigned short MIDIRxOverrunCounter = 0;
    unsigned short DMXTestCounter = 0;
    unsigned short lowDMXLamp = 0, highDMXLamp = 0;
    BYTE enableAD = false;
    BYTE Temp = 0;
            
    for (i = 0; i < DMXLENGTH; i++)
        PreviousDMXRxBuffer[i] = -1;
    
    for (i = 0; i < DMXLENGTH; i++)
        DMXTxBuffer[i] = DMXRxBuffer[i] = 0;
    
    for (i = 0; i < MAXPOTS; i++)
    {        
        for (j = 0; j < FILTERSIZE; j++) 
            arrPotValue[i][j] = 0x0000;
    }
    
    for (i = 0; i < MAXSERVOS; i++)
    {
        arrServoValue[i] = 0x0000;
        arrPotEnable[i] = false;        
    }
        
    InitializeSystem();    
    initI2C(I2C_BUS);
    
    DelayMs(200);
    
    printf("\r\rMIDI CHANNEL TEST #0");
    CheckConfig();
    printf("\r\rDMX Channel: %02X", DMXchannel);
    printf("\rSERVO Channel: %02X", SERVOchannel);      
    
    if (XBEEonly) printf("\rXBEE ONLY enabled");
    else printf("\rXBEE ONLY disabled");

    if (DMXenable) printf("\rDMX enabled");
    else printf("\rDMX disabled");

    if (DiagnosticsEnable) printf("\rDiagnostics enabled");
    else printf("\rDiagnostics disabled");

    printf("\r\rCONTROL COMMANDS:");    
    printf("\rCANCEL: C");
    printf("\rDIAGNOSTICS: D");
    printf("\rDMX: X");
    printf("\rXBEE: Z");       
    
    mLED_1_Off();
    mLED_2_Off();
    mLED_3_Off();
    mLED_4_Off();    
 
    while (1)         
    {         
        if (DMXdataReceived)
        {
            DMXdataReceived = false;
            if (DiagnosticsEnable)
            {
                printf("\r#%d DMX: ", DMXTestCounter++);
                for (i = 0; i < DMXLENGTH; i++)
                    printf("%d, ", DMXRxBuffer[i]);
            }
            for (i = 0; i < DMXLENGTH; i++)
            {
                if (PreviousDMXRxBuffer[i] != DMXRxBuffer[i])
                {
                    lampValue = DMXRxBuffer[i];
                    if (lampValue >= 255) lampValue = 255;
                                        
                    highDMXLamp = (lampValue / 32) | 0b01000000;
                    lowDMXLamp = (lampValue - (highDMXLamp * 32));  
                    
                    if (USBTxHead>=MAXBUFFER) USBTxHead = 0;
                    USBTxBuffer[USBTxHead].MIDICommand = DMXchannel;
                    USBTxBuffer[USBTxHead].MIDIData1 = i+1; // Add one to servo or lamp number
                    USBTxBuffer[USBTxHead].MIDIData2 = (BYTE) lowDMXLamp;
                    USBTxHead++;
                    if (USBTxHead>=MAXBUFFER) USBTxHead = 0;
                    USBTxBuffer[USBTxHead].MIDICommand = DMXchannel;
                    USBTxBuffer[USBTxHead].MIDIData1 = i+1;
                    USBTxBuffer[USBTxHead].MIDIData2 = (BYTE) highDMXLamp;
                    USBTxHead++;
                    if (USBTxHead>=MAXBUFFER) USBTxHead = 0;                                
                    if (DiagnosticsEnable || ResetFlag) printf("\rPOT #%02X: %i, ", i, lampValue);                    
                }
                PreviousDMXRxBuffer[i] = DMXRxBuffer[i];
            }
        }
        if (MIDIRxOverrunError)
        {
            MIDIRxOverrunCounter++;
            printf("\r#%d MIDI RX OVERRUN", MIDIRxOverrunCounter++);
            MIDIRxOverrunError = false;
        }
        if (controlCommand)
        {
            printf("\rCONTROL COMMAND %d: ", controlCommand);
            if (controlCommand == DMX_ENABLE)
            {
                if (DMXenable)
                {
                    DMXenable = false;
                    printf("\rDMX disabled");                    
                }
                else
                {
                    DMXenable = true;
                    printf("\rDMX enabled");
                }
                DelayMs(100);
            }
            else if (controlCommand == SAVE_CONFIG)
            {
                if (WriteConfig()) printf("\rConfig bytes written to EEprom");
                else printf("\rERROR writing config to EEprom");
            }
            
//#define SET_DMX_CHANNEL 22 // V
//#define SET_SERVO_CHANNEL 2 // B            
            
            else if (controlCommand == SET_DMX_CHANNEL)
            {
                Temp = HOSTRxBuffer[0];
                if (Temp >= 'A' && Temp <= 'F') 
                {
                    DMXchannel = (Temp - 'A' + 10) | 0xB0; 
                    printf("\rDMX Channel: %02X", DMXchannel);
                }
                else if (Temp >= '0' && Temp <= '9') 
                {
                    DMXchannel = (Temp - '0') | 0xB0; 
                    printf("\rDMX Channel: %02X", DMXchannel);
                }
                else printf("\rERROR: Bad character: %c", Temp);               
            }            
            else if (controlCommand == SET_SERVO_CHANNEL)
            {
                Temp = HOSTRxBuffer[0];
                if (Temp >= 'A' && Temp <= 'F') 
                {
                    SERVOchannel = (Temp - 'A' + 10) | 0xB0; 
                    printf("\rSERVO Channel: %02X", SERVOchannel);
                }
                else if (Temp >= '0' && Temp <= '9') 
                {
                    SERVOchannel = (Temp - '0') | 0xB0; 
                    printf("\rSERVO Channel: %02X", SERVOchannel);
                }
                else printf("\rERROR: Bad character: %c", Temp);               
            }            
            else if (controlCommand == XBEE_ONLY)
            {
                if (XBEEonly)
                {
                    XBEEonly = false;
                    printf("\rXBEE only OFF");
                }
                else
                {
                    XBEEonly = true;
                    printf("\rXBEE only ON");
                }
            }
            else if (controlCommand == DIAGNOSTICS)
            {
                if (DiagnosticsEnable)
                {
                    DiagnosticsEnable = false;
                    printf(" Diagnostics OFF");
                }
                else
                {
                    DiagnosticsEnable = true;
                    printf(" Diagnostics ON");
                }
            }
            else if (controlCommand == SPACEBAR) 
            {
                if (!RunFlag)
                {
                    printf(" RESET");
                    ResetFlag = true;
                    RunFlag = true;
                }
                else 
                {
                    RunFlag = false;
                    printf("\rTransmit disabled");
                }
            }
            else if (controlCommand == ENABLE_POTS) 
            {
                printf("ENABLE POTS: ");
                setupCommand = ENABLE_POTS;
                RunFlag = false;
            }
            else if (controlCommand == CANCEL_COMMAND)
            {
                printf("CANCEL COMMAND");
                setupCommand = 0;
            }            
            else if (controlCommand == SET_PERCUSSION)
            {
                printf("\rInitializing MIDI percussion: "); 
                if (MIDITxHead < MAXBUFFER)
                {
                    for (i = 0; i < NUM_PERCUSSION_INIT_BYTES; i++)
                    {
                        ch = MIDIpercussionCommand[i];
                        printf("%02X ", ch);
                        while(!UARTTransmitterIsReady(MIDIuart));                                
                        UARTSendDataByte(MIDIuart, ch);
                    }
                }
                else printf("\rMAXBUFFER OVERRUN ERROR"); 
            }
            else if (controlCommand == SEND_NOTE_OFF_47)
            {
                printf("\rNOTE OPF 0x47");                         
                while(!UARTTransmitterIsReady(MIDIuart));                                
                UARTSendDataByte(MIDIuart, 0x80);
                while(!UARTTransmitterIsReady(MIDIuart));                                
                UARTSendDataByte(MIDIuart, 0x47);
                while(!UARTTransmitterIsReady(MIDIuart));                                
                UARTSendDataByte(MIDIuart, 0x40);                                                
            }
            else if (controlCommand == SEND_NOTE_OFF_48)
            {
                printf("\rNOTE OPF 0x48");                         
                while(!UARTTransmitterIsReady(MIDIuart));                                
                UARTSendDataByte(MIDIuart, 0x80);
                while(!UARTTransmitterIsReady(MIDIuart));                                
                UARTSendDataByte(MIDIuart, 0x48);
                while(!UARTTransmitterIsReady(MIDIuart));                                
                UARTSendDataByte(MIDIuart, 0x40);                
            }
            controlCommand = 0;
        }
        if (ADint) 
        {
            ADint = false;
            if (enableAD)
            {
                IFS1bits.AD1IF = 0;
                while(!IFS1bits.AD1IF);        
                AD1CON1bits.ASAM = 0;        // Pause sampling. 
                for (i = 0; i < MAXPOTS; i++)
                    ADresult[i] = (unsigned short) ReadADC10(i); // read the result of channel 0 conversion from the idle buffer
                AD1CON1bits.ASAM = 1;        // Restart sampling.s            
                SortPots();
                for (i = 0; i < MAXPOTS; i++)
                    arrPotValue[i][filterIndex] = ADpots[i];  
                filterIndex++;
                if (filterIndex >= FILTERSIZE) filterIndex = 0;
            }
            
            if (RunFlag)
            {
                if (enableAD)
                {
                    for (i = 0; i < MAXPOTS; i++)
                    {         
                        if (arrPotEnable[i])
                        {                        
                            sumValue = 0;
                            for (j = 0; j < FILTERSIZE; j++) 
                                sumValue = sumValue + (long)arrPotValue[i][j];
                            averageValue = sumValue / FILTERSIZE;                
                            if ( (abs(averageValue - arrServoValue[i]) > 0) || ResetFlag)
                        
                            // averageValue = ADpots[i];;
                            // if ((averageValue != arrServoValue[i]) || ResetFlag)
                            if  ( (abs(averageValue - arrServoValue[i]) > 2) || ResetFlag)
                            {                                    
                                if (averageValue >= 1023) averageValue = 1023;                    
                                arrServoValue[i] = averageValue;
                    
                                servoHigh = (averageValue / 32) | 0b01000000;
                                servoLow = (averageValue - (servoHigh * 32));                            
                        
                                if (XBEEonly)
                                {
                                    outData[0] = averageValue; 
                                    command = 0xBA;
                                    ServoNumber = i;                                                    

                                    if ( !BuildXBEEpacket(command, ServoNumber, 1, outData) )
                                        printf("\rXBEE ONLY OVERRUN ERROR #%d", errorCounter++);
                                    else if (!INTGetEnable(INT_SOURCE_UART_TX(XBEEuart)))
                                    {
                                        ch = XBEETxBuffer[XBEETxTail++];
                                        if (XBEETxTail >= MAXXBEEBUFFER) XBEETxTail = 0;                
                                        while (!UARTTransmitterIsReady(XBEEuart));
                                        UARTSendDataByte(XBEEuart, ch);    
                                        INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_ENABLED);                                    
                                    }                                
                                    if (DiagnosticsEnable || ResetFlag) printf("\rXBEE ONLY #%d: %i, ", i, averageValue);
                                } 
                                else 
                                {
                                    if (USBTxHead>=MAXBUFFER) USBTxHead = 0;
                                    USBTxBuffer[USBTxHead].MIDICommand = SERVOchannel;
                                    USBTxBuffer[USBTxHead].MIDIData1 = i+1;  // Add one to servo or lamp number
                                    USBTxBuffer[USBTxHead].MIDIData2 = (BYTE) servoLow;
                                    USBTxHead++;
                                    if (USBTxHead>=MAXBUFFER) USBTxHead = 0;
                                    USBTxBuffer[USBTxHead].MIDICommand = SERVOchannel;
                                    USBTxBuffer[USBTxHead].MIDIData1 = i+1;
                                    USBTxBuffer[USBTxHead].MIDIData2 = (BYTE) servoHigh;
                                    USBTxHead++;
                                    if (USBTxHead>=MAXBUFFER) USBTxHead = 0;                                
                                    if (DiagnosticsEnable || ResetFlag) printf("\rUSBTxHead SERVO #%d: %i, ", i, averageValue);
                                }
                            } // if  ( (abs(averageValue
                        } // if (arrPotEnable[i])
                    } // for (i = 0;
                } // end enableAD   
            } // end if RunFlag
            if (ResetFlag) ResetFlag = false;
        } // end if (ADint) 
        // MIDIStateMachine(); // Check MIOI UART for incoming data        
        
        /*
#define USER_PUSHBUTTON PORTEbits.RE6
#define PROGRAM_PUSHBUTTON PORTEbits.RE7                  
        // USER pushbutton toggles AD pot reading on and off.
        // DiagnosticsEnable is set to true to enable AD converter.        
        if (ButtonReadCounter) ButtonReadCounter--;
        else
        {
            ButtonReadCounter = 10;
            ButtonRead = USER_PUSHBUTTON;
            if (ButtonRead != UserButtonState)
            {
                UserButtonDebouncer++;                        
                if (UserButtonDebouncer > 10)
                {
                    UserButtonDebouncer = 0;
                    UserButtonState = ButtonRead;
                    if (UserButtonState) printf("\rUSER Button RELEASED");
                    else
                    {
                        if (DiagnosticsEnable)
                        {
                            DiagnosticsEnable = false;
                            printf("\rTEST DISABLED");
                        }
                        else 
                        {
                            DiagnosticsEnable = true;
                            printf("\rTEST ENABLED");
                        }
                    }
                }
                DelayMs(10);
            }
            else UserButtonDebouncer = 0;
        }    
        */
        
        if (HOSTRxBufferFull)
        {
            HOSTRxBufferFull = false;
            if (setupCommand == ENABLE_POTS)
            {
                for (i = 0; i < MAXPOTS; i++) 
                    arrPotEnable[i] = false;
                enableAD = false;
                i = 0;
                do {
                    ch = HOSTRxBuffer[i];
                    if (ch == CR) break;
                    if (ch >= '0' && ch <= '9')
                    {
                        if (ch <= '9') arrPotEnable[ch - '0'] = true;
                        else arrPotEnable[ch - 'A' + 10] = true;
                        enableAD = true;
                    }
                    i++;
                } while(i < MAXPOTS);
                printf("\rENABLED POTS: ");
                for (i = 0; i < MAXPOTS; i++) 
                    if (arrPotEnable[i]) printf("\r%d, ", i);
            }
            setupCommand = 0;
        }        
#if defined(USB_INTERRUPT)
        USBDeviceAttach();
        Blah blah blah
#endif

#if defined(USB_POLLING)
        // Check bus status and service USB interrupts.
        USBDeviceTasks();
#endif
        if (ProcessUSB()) 
        {
            XBEEonly = false;
            USBactive = true;
        }
        ProcessMIDI_RS232();
    }//end while
}//end main        



void BlinkUSBStatus(void) {
    static long led_count = 0;

    if (led_count == 0) {
        led_count = 50000U;
    }
    led_count--;

#define mLED_Both_Off()         {mLED_1_Off();mLED_2_Off();}
#define mLED_Both_On()          {mLED_1_On();mLED_2_On();}
#define mLED_Only_1_On()        {mLED_1_On();mLED_2_Off();}
#define mLED_Only_2_On()        {mLED_1_Off();mLED_2_On();}

    if (USBSuspendControl == 1) {
        if (led_count == 0) {
            mLED_1_Toggle();
        }//end if
    } else {
        if (USBDeviceState == DETACHED_STATE) {
            mLED_Both_Off();
        } else if (USBDeviceState == ATTACHED_STATE) {
            mLED_Both_On();
        } else if (USBDeviceState == POWERED_STATE) {
            mLED_Only_1_On();
        } else if (USBDeviceState == DEFAULT_STATE) {
            mLED_Only_2_On();
        } else if (USBDeviceState == ADDRESS_STATE) {
            if (led_count == 0) {
                mLED_1_Toggle();
                mLED_2_Off();
            }//end if
        } else if (USBDeviceState == CONFIGURED_STATE) {
            if (led_count == 0) {
                mLED_1_Toggle();
            }//end if
        }//end if(...)
    }//end if(UCONbits.SUSPND...)

}//end BlinkUSBStatus

void USBCBSuspend(void) {
    ;
}

void USBCBWakeFromSuspend(void) {
    ;
}

void USBCB_SOF_Handler(void) {
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
    ;
}

// For debugging?

void USBCBErrorHandler(void) {
    ;
}

void USBCBCheckOtherReq(void) {
    ;
}//end

void USBCBStdSetDscHandler(void) {
    ;
}//end

void USBCBInitEP(void) {
    //enable the HID endpoint
    USBEnableEndpoint(MIDI_EP, USB_OUT_ENABLED | USB_IN_ENABLED | USB_HANDSHAKE_ENABLED | USB_DISALLOW_SETUP);

    //Re-arm the OUT endpoint for the next packet
    USBRxHandle = USBRxOnePacket(MIDI_EP, (BYTE*) & ReceivedDataBuffer, 64);
}

void USBCBSendResume(void) {
    static WORD delay_count;

    if (USBGetRemoteWakeupStatus() == true) {
        if (USBIsBusSuspended() == true) {
            USBMaskInterrupts();

            //Clock switch to settings consistent with normal USB operation.
            USBCBWakeFromSuspend();
            USBSuspendControl = 0;
            USBBusIsSuspended = false; //So we don't execute this code again,
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

BOOL USER_USB_CALLBACK_EVENT_HANDLER(int event, void *pdata, WORD size) {
    switch (event) {
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
    return true;
}

/*
void __ISR(_CHANGE_NOTICE_VECTOR, ipl2) ChangeNotice_Handler(void) {
    unsigned short PORTin;

    // Step #1 - always clear the mismatch condition first
    PORTin = PORTC & 0x6000;

    // Step #2 - then clear the interrupt flag
    mCNClearIntFlag();

}
*/


// HOST UART interrupt handler it is set at priority level 2

void __ISR(HOST_VECTOR, ipl2) IntHostUartHandler(void) {
    BYTE ch;
    static unsigned short HOSTRxIndex = 0;

    if (HOSTbits.OERR || HOSTbits.FERR) {
        if (UARTReceivedDataIsAvailable(HOSTuart))
            ch = UARTGetDataByte(HOSTuart);
        HOSTbits.OERR = 0;
        HOSTRxIndex = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));
        if (UARTReceivedDataIsAvailable(HOSTuart)) {
            ch = toupper(UARTGetDataByte(HOSTuart));
            if (ch == '\n' || ch == 0);
            else if (HOSTRxIndex == 0 && ch == ' ')
                controlCommand = SPACEBAR;
            else if (ch == BACKSPACE) {
                while (!UARTTransmitterIsReady(HOSTuart));
                UARTSendDataByte(HOSTuart, ' ');
                while (!UARTTransmitterIsReady(HOSTuart));
                UARTSendDataByte(HOSTuart, BACKSPACE);
                if (HOSTRxIndex > 0) HOSTRxIndex--;
            } else if (ch == ENTER) {
                HOSTRxBuffer[HOSTRxIndex] = '\r'; // $$$$
                HOSTRxBufferFull = true;
                HOSTRxIndex = 0;
            } else if (ch < 27) {
                controlCommand = ch;
                HOSTRxIndex = 0;
            }
            else if (HOSTRxIndex < MAXBUFFER) {
                HOSTRxBuffer[HOSTRxIndex++] = ch;
            }
        }
    }
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) 
    {
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
    }
}





static void InitializeSystem(void) 
{
    AD1PCFG = 0xFFFF;
    SYSTEMConfigPerformance(60000000);

#if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
#endif

#if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN; // See HardwareProfile.h
#endif

    UserInit();
    USBDeviceInit();  // Initializes USB module SFRs and firmware variables to known states.
}

// TEN ANALOG INPUTS: B0,B1,B3,B4,B5,B8,B9,B10,B11,B12
int ADC10_ManualInit(void)
{
    int i, dummy;
    
    AD1CON1bits.ON = 0;
    mAD1IntEnable(INT_DISABLED);   
    mAD1ClearIntFlag();
    
    AD1CON1 = 0;
    AD1CON2 = 0;
    AD1CON3 = 0;
    AD1CHS  = 0;
    AD1CSSL = 0;
    
    // Set each Port B pin for digital or analog
    // Analog = 0, digital = 1
    AD1PCFGbits.PCFG0 = 0; 
    AD1PCFGbits.PCFG1 = 0; 
    AD1PCFGbits.PCFG2 = 1; 
    AD1PCFGbits.PCFG3 = 0; 
    AD1PCFGbits.PCFG4 = 0; 
    AD1PCFGbits.PCFG5 = 1; 
    AD1PCFGbits.PCFG6 = 1; 
    AD1PCFGbits.PCFG7 = 1; 
    AD1PCFGbits.PCFG8 = 0; 
    AD1PCFGbits.PCFG9 = 0; 
    AD1PCFGbits.PCFG10 = 0; 
    AD1PCFGbits.PCFG11 = 0; 
    AD1PCFGbits.PCFG12 = 0; 
    AD1PCFGbits.PCFG13 = 0; 
    AD1PCFGbits.PCFG14 = 1; 
    AD1PCFGbits.PCFG15 = 1;     
    
    AD1CON1bits.FORM = 000;        // 16 bit integer format.
    AD1CON1bits.SSRC = 7;        // Auto Convert
    AD1CON1bits.CLRASAM = 0;    // Normal operation - buffer overwritten by next conversion sequence
    AD1CON1bits.ASAM = 0;        // Not enable Automatic sampling yet.
    
    AD1CON2bits.VCFG = 0;        // Reference AVdd, AVss
    AD1CON2bits.OFFCAL = 0;        // Offset calibration disable.
    AD1CON2bits.CSCNA = 1;        // Scan inputs for CH0+ SHA Input for Mux A input 
    AD1CON2bits.SMPI = 0b1000;        // Interrupt after 9+1 conversion
    AD1CON2bits.BUFM = 0;        // One 16 word buffer
    AD1CON2bits.ALTS = 0;        // Use only Mux A
    AD1CON2bits.SMPI =  MAXPOTS-1;    // Number of channels to sample
    AD1CON2bits.BUFM = 0;                // Single 16-word buffer with CLRASAM.    
    AD1CHSbits.CH0NA = 0; // Mux A Negative input from VR-
    AD1CHSbits.CH0SA = 3; // Mux A Positive input from pin AN3

    // Set conversion clock and set sampling time.
    AD1CON3bits.ADRC = 0;        // Clock derived from peripheral bus clock
    AD1CON3bits.SAMC = 0b11111;        // Sample time max
    AD1CON3bits.ADCS = 0b11111111;   // Conversion time max

    // TEN ANALOG INPUTS: B0,B1,B3,B4,B5,B8,B9,B10,B11,B12
    // Select channels to scan. Scan channels = 1, Skip channels = 0
    AD1CSSLbits.CSSL0 = 1;
    AD1CSSLbits.CSSL1 = 1;
    AD1CSSLbits.CSSL2 = 0;
    AD1CSSLbits.CSSL3 = 1;
    AD1CSSLbits.CSSL4 = 1;
    AD1CSSLbits.CSSL5 = 0;
    AD1CSSLbits.CSSL6 = 0;
    AD1CSSLbits.CSSL7 = 0;
    AD1CSSLbits.CSSL8 = 1;
    AD1CSSLbits.CSSL9 = 1;
    AD1CSSLbits.CSSL10 = 1;
    AD1CSSLbits.CSSL11 = 1;
    AD1CSSLbits.CSSL12 = 1;
    AD1CSSLbits.CSSL13 = 1;
    AD1CSSLbits.CSSL14 = 0;
    AD1CSSLbits.CSSL15 = 0;
    
    // Make sure all buffers have been Emptied. 
    for (i = 0; i < 16; i++) dummy = (ADC1BUF0+i*4);    
    
    AD1CON1bits.ASAM = 1;        // Start Automatic Sampling. 
    AD1CON1bits.ON = 1;            // Turn on ADC.
    return (1);
}


void __ISR(XBEE_VECTOR, ipl2) IntXBEEHandler(void) 
{    
    BYTE ch;

    if (XBEEbits.OERR || XBEEbits.FERR) 
    {
        if (UARTReceivedDataIsAvailable(XBEEuart))
            ch = UARTGetDataByte(XBEEuart);
        XBEEbits.OERR = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(XBEEuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(XBEEuart));
    }

    if (INTGetFlag(INT_SOURCE_UART_TX(XBEEuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(XBEEuart));
        if (XBEETxTail!=XBEETxHead) 
        {
            ch = XBEETxBuffer[XBEETxTail++];
            if (XBEETxTail >= MAXXBEEBUFFER) XBEETxTail = 0;                
            while (!UARTTransmitterIsReady(XBEEuart));
            UARTSendDataByte(XBEEuart, ch);
        } else INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_DISABLED);
    }
}




BYTE BuildXBEEpacket(BYTE command, BYTE ServoNumber, BYTE numData, short *ptrData)
{
	int i, j;    
    BYTE arrOutputBytes[MAXBUFFER];
	short packetIndex = 0, packetLength = 0, numBytes = 0;
    static short totalPacketLength = 0, packetCount = 0;
    BYTE dataByte;  
    BYTE TxPacket[MAXBUFFER];  
    short TempHead;
    
    if (numData > MAXBUFFER) 
    {
        printf("\rERROR #1 DATA > MAX");
        return false;
    }
    
    union
    {
        BYTE b[2];
        unsigned short integer;
    } convert;	

	j = 0;
	// Packet header first:
	arrOutputBytes[j++] = command;
	arrOutputBytes[j++] = ServoNumber;
	arrOutputBytes[j++] = numData;

	// Now add packet data:  
	for (i = 0; i < numData; i++)
	{
		convert.integer = ptrData[i];
		arrOutputBytes[j++] = convert.b[0];
		arrOutputBytes[j++] = convert.b[1];
	}
    // Finally, packet CRC:
	convert.integer = CalculateModbusCRC(arrOutputBytes, j);      
    
	arrOutputBytes[j++] = convert.b[0];
	arrOutputBytes[j++] = convert.b[1];
	numBytes = j;

    // Now encode packet:
    packetIndex = 0;
	TxPacket[packetIndex++] = STX;
	for (i = 0; i < numBytes; i++)
	{
		dataByte = arrOutputBytes[i];        
		if (dataByte == STX || dataByte == DLE || dataByte == ETX)
        {
            if (packetIndex >= MAXBUFFER) 
            {
                printf("\rERROR #2 PACKET > MAX");
                return false;
            }
			TxPacket[packetIndex++] = DLE;
        }
		if (dataByte == ETX) dataByte = ETX - 1;
        if (packetIndex >= MAXBUFFER) 
        {
            printf("\rERROR #3 PACKET > MAX");
            return false;        
        }
		TxPacket[packetIndex++] = dataByte;
	}
    if (packetIndex >= MAXBUFFER) 
    {
        printf("\rERROR #4 PACKET > MAX");
        return false;            
    }

	TxPacket[packetIndex++] = ETX;		
	
    packetLength = packetIndex;
    totalPacketLength = totalPacketLength + packetLength;
    
    // Before inserting new packet into transmit buffer, make sure there is enough room for it:
    TempHead = XBEETxHead;    
    i = 0;
    do {        
        i++;
        if (i >= packetLength) break;
        TempHead++;
        if (TempHead >= MAXXBEEBUFFER) TempHead = 0;
    } while (TempHead!=XBEETxTail);
    
    if (TempHead==XBEETxTail) 
    {
        printf("\rBuildXBEEpacket ERROR #5 LEN: %d: %d, TAIL: %d, COUNT; %d, TOTAL: %d", packetLength, XBEETxTail, packetCount, totalPacketLength);
        packetCount = 0;
        totalPacketLength = 0;
        XBEETxHead = XBEETxTail;
        INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_DISABLED);
        return false; 
    }
    else packetCount++;
    
    // Ready to go. Copy packet to transmit buffer:
    for (i = 0; i < packetLength; i++)
    {        
        XBEETxBuffer[XBEETxHead++] = TxPacket[i];
        if (XBEETxHead >= MAXXBEEBUFFER) XBEETxHead = 0;
    }    
    
    return true;
}



void __ISR(MIDI_VECTOR, ipl2) IntMIDIHandler(void) 
{    
    BYTE ch;

    if (MIDIbits.OERR || MIDIbits.FERR) {
        if (UARTReceivedDataIsAvailable(MIDIuart))
            ch = UARTGetDataByte(MIDIuart);
        MIDIbits.OERR = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(MIDIuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(MIDIuart));
        
        if (UARTReceivedDataIsAvailable(MIDIuart)) {
            ch = UARTGetDataByte(MIDIuart);                    
            if (ch != 0xfe) 
            {
                if (MIDIRxHead < MAXBUFFER) MIDIRxBuffer[MIDIRxHead] = ch;
                MIDIRxHead++;
                if (MIDIRxHead >= MAXBUFFER) MIDIRxHead = 0;
                if (MIDIRxHead == MIDIRxTail) MIDIRxOverrunError = true;
            }
        }
    }

    if (INTGetFlag(INT_SOURCE_UART_TX(MIDIuart))) 
        INTClearFlag(INT_SOURCE_UART_TX(MIDIuart));
}

BYTE ProcessMIDI_RS232(void)
{
    static BYTE ch = 0, MIDIcommand = 0, MIDIdata1 = 0, MIDIdata2 = 0;
    static unsigned short MIDIRxState = 0;        
    
    if (MIDITxTail!=MIDITxHead) 
    {           
        if (MIDITxTail >= MAXBUFFER)
            MIDITxTail=MIDITxHead;
        else if (UARTTransmitterIsReady(MIDIuart) && !MIDITxDelay)
        {
                ch = MIDITxBuffer[MIDITxTail].MIDICommand;                
                UARTSendDataByte(MIDIuart, ch);
                
                while(!UARTTransmitterIsReady(MIDIuart));
                ch = MIDITxBuffer[MIDITxTail].MIDIData1;                
                UARTSendDataByte(MIDIuart, ch);
                
                while(!UARTTransmitterIsReady(MIDIuart));
                ch = MIDITxBuffer[MIDITxTail].MIDIData2;                
                UARTSendDataByte(MIDIuart, ch);                
                
                MIDITxTail++;
                if (MIDITxTail >= MAXBUFFER) MIDITxTail = 0;
                MIDITxDelay = 5;
        }
    }
    
    while (MIDIRxTail!=MIDIRxHead)
    {
        ch = MIDIRxBuffer[MIDIRxTail];        
        MIDIRxTail++;
        if (MIDIRxTail>=MAXBUFFER) MIDIRxTail=0;
        if (ch & 0x80) 
        {                                      
            MIDIcommand = ch;
            MIDIRxState = 1;
        }
        else if (MIDIRxState==1) 
        {
            MIDIdata1 = ch;
            MIDIRxState = 2;
        }
        else if (MIDIRxState==2)
        {
            MIDIdata2 = ch;
            MIDIRxState = 1;
            if (MIDIdata1 != 0x47 && MIDIdata1 != 0x48)
            {
                if (USBTxHead < MAXBUFFER)
                {
                    USBTxBuffer[USBTxHead].MIDICommand = MIDIcommand;
                    USBTxBuffer[USBTxHead].MIDIData1 = MIDIdata1;
                    USBTxBuffer[USBTxHead].MIDIData2 = MIDIdata2;
                    USBTxHead++;
                    if (USBTxHead >= MAXBUFFER) USBTxHead = 0;
                }
                else // This error should never occur
                {
                    USBTxHead = USBTxTail = 0;
                    MIDIRxState = 0;
                    printf("\rMIDI USB TX HEAD OVERRUN ERROR");
                    return false;
                }
            }
        }            
    }    
    return true;
}//end ProcessUSB    



void UserInit(void) 
{
    // Turn off JTAG so we get the pins back
    mJTAGPortEnable(false);

    ADC10_ManualInit();
    
    //PORTSetPinsDigitalIn(IOPORT_C, BIT_14 | BIT_13);
    //mCNOpen(CN_ON, CN0_ENABLE | CN1_ENABLE, CN0_PULLUP_ENABLE | CN1_PULLUP_ENABLE);
    //ConfigIntCN(CHANGE_INT_ON | CHANGE_INT_PRI_2);

    //initialize the variable holding the handle for the last
    // transmission
    USBTxHandle = NULL;
    USBRxHandle = NULL;

    // Set up main UART
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
#define SYS_FREQ 80000000
    UARTSetDataRate(HOSTuart, SYS_FREQ, 921600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART #2 Interrupts
    INTEnable(INT_U2TX, INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(HOSTuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);

    
    // Set up XBEE at 57600 baud
    UARTConfigure(XBEEuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);      
    UARTSetFifoMode(XBEEuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(XBEEuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    ActualXBEEBaudrate = UARTSetDataRate(XBEEuart, SYS_FREQ, 57600);
    UARTEnable(XBEEuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure XBEE Interrupts
    INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(XBEEuart), INT_DISABLED);
    INTSetVectorPriority(INT_VECTOR_UART(XBEEuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(XBEEuart), INT_SUB_PRIORITY_LEVEL_0);
    
        
    // Set up MIDI at 31250 baud
    UARTConfigure(MIDIuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(MIDIuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(MIDIuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    ActualMIDIBaudrate = UARTSetDataRate(MIDIuart, SYS_FREQ, 31250);
    UARTEnable(MIDIuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure MIDI Interrupts
    INTEnable(INT_SOURCE_UART_TX(MIDIuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(MIDIuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(MIDIuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(MIDIuart), INT_SUB_PRIORITY_LEVEL_0);
        
    
// Set up DMX UART for 250000 baud
    UARTConfigure(DMXuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(DMXuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(DMXuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_2);
    ActualDMXBaudrate = UARTSetDataRate(DMXuart, SYS_FREQ, 250000);
    UARTEnable(DMXuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure DMX UART Interrupts
    INTEnable(DMX_TX_INT, INT_DISABLED);
    INTEnable(DMX_RX_INT, INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(DMXuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(DMXuart), INT_SUB_PRIORITY_LEVEL_0);    

    // Set up Port A outputs:
    PORTSetPinsDigitalIn(IOPORT_A, BIT_14 | BIT_15);
    
    // Set up Port A outputs:
    PORTSetPinsDigitalOut(IOPORT_A, BIT_5);
    
    // Set up Port D outputs:
    PORTSetPinsDigitalOut(IOPORT_D, BIT_2);
    
    // Set up Port E outputs:
    PORTSetPinsDigitalOut(IOPORT_E, BIT_0 | BIT_1 | BIT_2 | BIT_3 | BIT_4);
    PORTSetBits(IOPORT_E, BIT_0 | BIT_1 | BIT_2 | BIT_3);

    // Set up Port E outputs:
    PORTSetPinsDigitalIn(IOPORT_E, BIT_6 | BIT_7);

    // Set up Timer 2 for 0.2 millisecond roll-over rate
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_64, 250); // was 1250
    // set up the core timer interrupt with a priority of 5 and zero sub-priority
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_5);
    
    // Set up Timer 4 with interrupts
    ConfigIntTimer4(T4_INT_ON | T4_INT_PRIOR_2);
    OpenTimer4(T4_ON | T4_SOURCE_INT | T4_PS_1_256, 7102);

    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();

}//end UserInit

void __ISR(_TIMER_2_VECTOR, ipl5) Timer2Handler(void) 
{
    static unsigned short hundredthSecondTimer = 0;

    mT2ClearIntFlag(); // clear the interrupt flag    
    
    hundredthSecondTimer++;
    if (hundredthSecondTimer > 50) // Was 10
    {
        hundredthSecondTimer = 0;
        ADint = true; 
    }
    
    if (MIDITxDelay)
        MIDITxDelay--;
}


void __ISR(_TIMER_4_VECTOR, ipl2) Timer4Handler(void) 
{

    mT4ClearIntFlag();

    if (DMXstate == DMX_STANDBY){
        TEST_OUT = 1;
        DMXstate = DMX_BREAK; // This is the beginning of the BREAK
        OpenTimer4(T4_ON | T4_SOURCE_INT | T4_PS_1_1, 9600);
        PORTClearBits(IOPORT_D, BIT_2);
    }else if (DMXstate == DMX_BREAK){ // MARK before transmitting data:        
        DMXstate = DMX_MARK;
        OpenTimer4(T4_ON | T4_SOURCE_INT | T4_PS_1_1, 960);
        PORTSetBits(IOPORT_D, BIT_2); // Release transmit line, let it go high to mark end of break
    }else if (DMXstate == DMX_MARK){
        TEST_OUT = 0;
        DMXstate = DMX_DATA;        
        INTEnable(DMX_TX_INT, INT_ENABLED);
        OpenTimer4(T4_ON | T4_SOURCE_INT | T4_PS_1_256, 7102);
        while (!UARTTransmitterIsReady(DMXuart));
        DMXTXREG = STARTBYTE;
    }else{
        DMXstate = DMX_BREAK; // This is the beginning of the BREAK
        OpenTimer4(T4_ON | T4_SOURCE_INT | T4_PS_1_1, 9600);
        PORTClearBits(IOPORT_D, BIT_2);
        
    }
}


// This routine handles both USB receiving and sending.
// Returns TRUE if USB is active,
// otherwise returns FALSE to indicate USB isn't working.
//
// The Microchip MIDI device is designed to send and receive
// only one MIDI packet at a time. All packets must be four bytes.
// First byte we ignore, next three bytes are standard MIDI command + data.
// All MIDI commands have MSB = 1. The next two bytes must have MSB = 0.
// So data greater than 127 must be broken up into two consecutive packets.
// 
// BYTE 0: Not sure what first byte does, but we ignore it.
// BYTE 1: Standard MIDI command. High nibble is command, low nibble is MIDI channel. We use MIDI control command 0xB for servos. 
// BYTE 2: We use this as servo ID number.
// BYTE 3: Servo data.
BYTE ProcessUSB(void)
{
    static BYTE LEDflag = false;
    static short LEDcounter = 0;
    static unsigned short lowServoByte = 0, highServoByte = 0, lowDMXLamp = 0, highDMXLamp = 0;
    short servoValue = 0, lampValue = 0;
    BYTE ch, MIDIcommand = 0, command = 0, DMXLampNumber = 0, ServoNumber = 0, frame = 0, MIDIvelocity = 0;
    unsigned short TempHead = 0, TxIndex = 0;
    static unsigned MIDIerrorCounter = 0;
    short i;
   
    if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) return false;  // Return FALSE if USB is not in use.
    
    BlinkUSBStatus();

    // INCOMING USB DATA FROM PC IS HANDLED HERE
    if (!USBHandleBusy(USBRxHandle)) 
    {
        USBRxHandle = USBRxOnePacket(MIDI_EP, ReceivedDataBuffer, 4);
        MIDIcommand = ReceivedDataBuffer[1]; // & 0xF0;
        MIDIvelocity = ReceivedDataBuffer[3];
        // If incoming USB MIDI data isn't intended for robots, 
        // assume it is for other MIDI devices such the keyboard.
        // So copy the data and send it out the MIDI Uart: 
        
        /*
        if (MIDIcommand == 0x80 || MIDIcommand == 0x90)
        {            
            if (MIDIcommand == 0x80 || MIDIvelocity == 0) 
                printf("\r>NOTE OFF: %02X, %02X, %02X, %02X", ReceivedDataBuffer[0], ReceivedDataBuffer[1], ReceivedDataBuffer[2], ReceivedDataBuffer[3]);
            else printf("\r>NOTE ON: %02X, %02X, %02X, %02X", ReceivedDataBuffer[0], ReceivedDataBuffer[1], ReceivedDataBuffer[2], ReceivedDataBuffer[3]);
            // Before inserting new packet into MIDI transmit buffer, make sure there is enough room for it:
            TempHead = MIDITxHead;    
            TxIndex = 0;
            do {        
                TempHead++;
                if (TempHead >= MAXBUFFER) TempHead = 0;
                TxIndex++;
            } while (TempHead!=MIDITxTail && TxIndex < 3);    
            
            // If there is an overrun error in MIDI TX buffer, display it here
            if (TempHead==MIDITxTail || MIDITxHead >= MAXBUFFER) 
                printf("\r#%d: MIDI TX OVERRUN in ProcessUSB()", MIDIerrorCounter++);
            // Otherwise insert the three MIDI bytes in the TX buffer and send to MIDI Uart:
            else 
            {
                MIDITxBuffer[MIDITxHead].MIDICommand = ReceivedDataBuffer[1];
                MIDITxBuffer[MIDITxHead].MIDIData1 = ReceivedDataBuffer[2];
                MIDITxBuffer[MIDITxHead].MIDIData2 = ReceivedDataBuffer[3];
                MIDITxHead++;
                if (MIDITxHead >= MAXBUFFER) MIDITxHead = 0;                
            }
            return true; // Return true to indicate USB is active
        }        
        */
        
        // MIDI commands are for DMX lamp control:
        if (MIDIcommand == (DMXchannel-1) && DMXenable)
        {
            if (0 == (ReceivedDataBuffer[3] & 0b01000000)) lowDMXLamp = (unsigned short)ReceivedDataBuffer[3];
            else {
                highDMXLamp = (unsigned short) (ReceivedDataBuffer[3] & 0b10111111);
                highDMXLamp = highDMXLamp << 5;
                lampValue = lowDMXLamp | highDMXLamp;
                
                outData[0] = lampValue; 
                command = ReceivedDataBuffer[1] & 0xFF;
                DMXLampNumber = ReceivedDataBuffer[2]-1;                      
                if (DMXLampNumber >= 1 && DMXLampNumber <= DMXLENGTH)
                {
                    DMXTxBuffer[DMXLampNumber] = lampValue;
                    printf("\rMIDI %02X LAMP #%02X DMX USB TX: ", MIDIcommand, DMXLampNumber);
                    for (i = 0; i < DMXLENGTH; i++)                    
                        printf("%d ", DMXTxBuffer[i]);                    
                }
            }
        }                  
        // Assume 0xB MIDI commands are for servo control:
        else if (MIDIcommand == SERVOchannel-1)
        {
            if (0 == (ReceivedDataBuffer[3] & 0b01000000)) lowServoByte = (unsigned short)ReceivedDataBuffer[3];
            else {
                highServoByte = (unsigned short) (ReceivedDataBuffer[3] & 0b10111111);
                highServoByte = highServoByte << 5;
                servoValue = lowServoByte | highServoByte;
                
                outData[0] = servoValue; 
                command = ReceivedDataBuffer[1] & 0xFF;
                ServoNumber = ReceivedDataBuffer[2]-1;                      
                
                if (ServoNumber >= 0)
                {
                    printf("\r"); 
                    if ( !BuildXBEEpacket(command, ServoNumber, 1, outData) )
                        printf("OVERRUN ERROR %d, ", errorCounter++);                                        
                    else if (!INTGetEnable(INT_SOURCE_UART_TX(XBEEuart)))
                    {
                        ch = XBEETxBuffer[XBEETxTail++];
                        if (XBEETxTail >= MAXXBEEBUFFER) XBEETxTail = 0;                
                        while (!UARTTransmitterIsReady(XBEEuart));
                        UARTSendDataByte(XBEEuart, ch);
                        INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_ENABLED);                        
                    }                                        
                }
                printf("USB RX: %02X Servo #%d: %d", command, ServoNumber, servoValue);            
            }
        }          
        else printf("\r>???: %02X, %02X, %02X, %02X", ReceivedDataBuffer[0], ReceivedDataBuffer[1], ReceivedDataBuffer[2], ReceivedDataBuffer[3]);
        LEDcounter++;
        if (LEDcounter > 10) {
            LEDcounter = 0;
            if (LEDflag) {
                mLED_4_On();
                LEDflag = false;
                LATBbits.LATB2 = 1;
            } else {
                mLED_4_Off();
                LEDflag = true;
                LATBbits.LATB2 = 0;
            }
        }
    }
    
    // OUTGOING USB DATA TO BE SENT TO PC IS HANDLED HERE
    if (USBTxTail!=USBTxHead && !USBHandleBusy(USBTxHandle))
    {           
        midiData.Val = 0;   //must set all unused values to 0 so go ahead
                    
        midiData.CableNumber = 0;
        midiData.CodeIndexNumber = MIDI_CIN_CONTROL_CHANGE;
        midiData.DATA_0 = USBTxBuffer[USBTxTail].MIDICommand;
        midiData.DATA_1 = USBTxBuffer[USBTxTail].MIDIData1;
        midiData.DATA_2 = USBTxBuffer[USBTxTail].MIDIData2;
                
        USBTxHandle = USBTxOnePacket(MIDI_EP,(BYTE*)&midiData,4);
        USBTxTail++;
        if (USBTxTail >= MAXBUFFER) USBTxTail = 0;
    }    
    return true; // Return true to indicate USB is active
}//end ProcessUSB    

//	DMX512 INTERRUPTS
void __ISR(DMX_VECTOR, ipl2) IntDMXHandler(void) 
{
static unsigned short	DMXRxPtr = 0;
static unsigned short	DMXTxPtr = 0;
static BYTE             DMXDataCounter = 0, dummyCounter = 0;
BYTE                    ch, dummy;

	// RX interrupts
	if(INTGetFlag(INT_SOURCE_UART_RX(DMXuart)))
    {
		// Clear the RX interrupt Flag
	   INTClearFlag(INT_SOURCE_UART_RX(DMXuart));
	   
   		if (1==DMXSTAbits.OERR)				// If DMX overrun occurs, clear overrun flag
				DMXSTAbits.OERR=0;										   
	   
		if (DMXSTAbits.FERR==1)
        {				
   			dummy=UARTGetDataByte(DMXuart); 	
 			dummyCounter=0;
			DMXRxPtr=0;																									
		}			
		else if (UARTReceivedDataIsAvailable(DMXuart))
        {
			ch=UARTGetDataByte(DMXuart);	
			if(dummyCounter<1)
				dummyCounter++;			
			else if (DMXRxPtr<DMXLENGTH)
            {	
                DMXRxBuffer[DMXRxPtr]=ch;			 
                DMXRxPtr++;
				if (DMXRxPtr==DMXLENGTH)
                {		
                    DMXenable = true;
                    DMXdataReceived = true;
				}					
			}	
		}			
	}	

    // TX interrupts:
    if (INTGetFlag(INT_SOURCE_UART_TX(DMXuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(DMXuart));
        if (DMXstate == DMX_DATA) 
        {
            if (DMXTxPtr < DMXLENGTH) 
            {
                ch = DMXTxBuffer[DMXTxPtr];
                DMXTxPtr++;
                while (!UARTTransmitterIsReady(DMXuart));
                DMXTXREG = ch; // (ch | UARTMASK);
            } else {
                DMXstate = DMX_STANDBY;
                INTEnable(DMX_TX_INT, INT_DISABLED);
                DMXTxPtr = 0;
            }
        }
    }
}


BYTE WriteConfigByte(unsigned short address, BYTE DataByte)
{
    BYTE result;    
    result = EepromWriteByte(I2C_BUS, EEPROM_ID, address, DataByte);
    DelayMs(10);        
    if (result) return true;
    else return false;
}

BYTE ReadConfigByte(unsigned short address, BYTE *ptrDataByte)
{
BYTE result;
    result = EepromReadByte(I2C_BUS, EEPROM_ID, address, ptrDataByte);    
    if (result) return true;
    return false;
}

BYTE CheckConfig()
{
BYTE TestByte1, TestByte2, TestByte3;
    printf("\rCHECKING config bytes...");
    if (!ReadConfigByte(TEST_ADDRESS_1, &TestByte1)) return false;
    if (!ReadConfigByte(TEST_ADDRESS_2, &TestByte2)) return false;
    if (!ReadConfigByte(TEST_ADDRESS_3, &TestByte3)) return false;
    if (TestByte1 != TEST_BYTE_1 || TestByte2 != TEST_BYTE_2 || TestByte3 != TEST_BYTE_3)
    {    
        if (!WriteConfig()) return false;
        if (!WriteConfigByte (TEST_ADDRESS_1, TEST_BYTE_1)) return false;
        if (!WriteConfigByte (TEST_ADDRESS_2, TEST_BYTE_2)) return false;
        if (!WriteConfigByte (TEST_ADDRESS_3, TEST_BYTE_3)) return false;
        printf("\rCHECK FACTORY CONFIG WRITTEN");        
    }
    else printf("\rConfig bytes OK.");
    if (ReadConfig()) 
    {
        printf("\rConfig bytes read.");
        return true;
    }
    else
    {
        printf("\rCHECK CONFIG BYTE READ ERROR");
        return false;
    }
}

BYTE WriteConfig()
{
    if (!WriteConfigByte(XBEE_ADDRESS, XBEEonly)) return false;
    if (!WriteConfigByte(DMX_ADDRESS, DMXenable)) return false;
    if (!WriteConfigByte(DIAGNOSTICS_ADDRESS, DiagnosticsEnable)) return false;
    if (!WriteConfigByte(DMX_CHANNEL_ADDRESS, DMXchannel)) return false;
    if (!WriteConfigByte(SERVO_CHANNEL_ADDRESS, SERVOchannel)) return false;
    return true;
}

BYTE ReadConfig()
{
    if (!ReadConfigByte(XBEE_ADDRESS, &XBEEonly)) return false;
    if (!ReadConfigByte(DMX_ADDRESS, &DMXenable)) return false;
    if (!ReadConfigByte(DIAGNOSTICS_ADDRESS, &DiagnosticsEnable)) return false;
    if (!ReadConfigByte(DMX_CHANNEL_ADDRESS, &DMXchannel)) return false;
    if (!ReadConfigByte(SERVO_CHANNEL_ADDRESS, &SERVOchannel)) return false;    
    return true;
}

/** EOF main.c *************************************************/
#endif
