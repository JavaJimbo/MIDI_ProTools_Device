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
 ************************************************************************************************************/

#ifndef MAIN_C
#define MAIN_C

enum {
    MIDI_STANDBY = 0,
    SEND_MIDI_COMMAND,
    SEND_SERVO_ID,
    SEND_SERVO_DATA
};

#define DIAGNOSTICS 4
#define XBEE_ONLY 24
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

#define RS485uart UART5
#define RS485_RxReg U5RXREG
#define RS485_TxReg U5TXREG
#define RS485bits U5STAbits

#define RS485_VECTOR _UART_5_VECTOR
#define RS485_RX_IRQ _UART5_RX_IRQ
#define RS485_TX_IRQ _UART5_TX_IRQ


#define MAXBUFFER 1024
BYTE HOSTRxBuffer[MAXBUFFER];
BYTE HOSTRxBufferFull = false;

#define MAXXBEEBUFFER 16384
unsigned short XBEERxIndex = 0;
unsigned short XBEERxLength = 0;
BYTE XBEERxBuffer[16384];
BYTE XBEEBufferFull = false;
BYTE XBEETxBuffer[MAXXBEEBUFFER];
unsigned short XBEETxHead = 0, XBEETxTail = 0;
short outData[MAXBUFFER];

BYTE MIDI_USB_Buffer[MAXBUFFER];
BYTE MIDIRxBuffer[MAXBUFFER];
unsigned short MIDIRxHead = 0, MIDIRxTail = 0;

#define MAXPOTS 10
unsigned short ADresult[MAXPOTS];
unsigned short ADpots[MAXPOTS];

void ConfigAd(void);

#define SET_DISPLAY 4 // CTL-D
#define SET_SERVO 19 // CTL-S
#define SET_RECORD 18 // CTL-R
#define SET_STANDBY 1 // CTL-A
#define SET_SCALE 3 // CTL-C
#define SET_PLAY 16 // CTL-P
#define SET_PERCUSSION 5 // CTL-E
#define SET_TEST 20 // CTL-T
#define NUMBER_ERROR 32767

struct servoType {
    BYTE ID;
    unsigned short position;
    BYTE updated;
    BYTE enabled;
};

unsigned short MIDIStateMachine(void);
// short MIDItimeout = 0;
BYTE MIDIbufferFull = false;

/*
#define MIDI_TIMEOUT 2
#define STANDBY 0
#define PLAY 1
#define RECORD 2
#define MIDI_MODE 3
#define TEST_MODE 4
*/

const BYTE MIDIpercussionCommand[7] = {0xB0, 0x00, 0x7F, 0x20, 0x00, 0xC0, 0x00};
unsigned short MIDInoteTimeout = 0;

BYTE ADint = false;
#define FILTERSIZE 16
long arrPotValue[MAXPOTS][FILTERSIZE];
BYTE arrPotEnable[MAXPOTS];
#define MAXSERVOS MAXPOTS
long arrServoValue[MAXSERVOS];
long servoLow = 0, servoHigh = 0;

BYTE setupCommand = 0, controlCommand = 0x00;

// ADresult
// ADpots

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
    BYTE MIDIServoNumber;
    BYTE servoData;    
}MIDITxPacketType;

MIDITxPacketType MIDITxBuffer[MAXBUFFER];
short MIDITxHead = 0, MIDITxTail = 0, USBTxTail = 0;
BYTE MIDITxState = 0;

#define ENABLE_POTS 16
BYTE XBEEonly = false;
long ActualXBEEBaudrate = 0;
short errorCounter = 0;
BYTE DiagnosticsEnable = false;

int main(void) 
{
    BYTE UserButtonState = 1, ButtonRead, ch = 0;
    short i, j, k, UserButtonDebouncer = 0;        
    short filterIndex = 0;
    long averageValue = 0;
    long sumValue;    
    BYTE command, ServoNumber;
    BYTE ResetFlag = false;
    BYTE RunFlag = false;
    short ButtonReadCounter = 0;    
    BYTE USBactive = false;
    
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
    
    DelayMs(200);

    printf("\r\rOVERRUN ERRORS MAXXBEEBUFFER = 16000, INTERRUPTS NOT INTERRUPTED #1");
    printf("\rXBEE XMIT START #1: XBEE Baudrate: %ld #1", ActualXBEEBaudrate);    
    
    mLED_1_Off();
    mLED_2_Off();
    mLED_3_Off();
    mLED_4_Off();    
    
    while (1)         
    {        
        if (controlCommand)
        {
            printf("\rCONTROL COMMAND %d: ", controlCommand);
            if (controlCommand == XBEE_ONLY)
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
            if (controlCommand == DIAGNOSTICS)
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
            else if (controlCommand == 17 || controlCommand == 3)
            {
                printf("CANCEL COMMAND");
                setupCommand = 0;
            }
            controlCommand = 0;
        }
                
        
        if (ADint) 
        {
            ADint = false;
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
            
            if (RunFlag)
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
                                if (MIDITxHead>=MAXBUFFER) MIDITxHead = 0;
                                MIDITxBuffer[MIDITxHead].MIDICommand = 0xB0;
                                MIDITxBuffer[MIDITxHead].MIDIServoNumber = i+1;
                                MIDITxBuffer[MIDITxHead].servoData = (BYTE) servoLow;
                                MIDITxHead++;
                                if (MIDITxHead>=MAXBUFFER) MIDITxHead = 0;
                                MIDITxBuffer[MIDITxHead].MIDICommand = 0xB0;
                                MIDITxBuffer[MIDITxHead].MIDIServoNumber = i+1;
                                MIDITxBuffer[MIDITxHead].servoData = (BYTE) servoHigh;
                                MIDITxHead++;
                                if (MIDITxHead>=MAXBUFFER) MIDITxHead = 0;    
                            
                                if (!USBactive)
                                {
                                    if (DiagnosticsEnable || ResetFlag) printf("\rMIDI RS232 OUT #%d: %i, ", i, averageValue);
                                    if (!INTGetEnable(INT_SOURCE_UART_TX(MIDIuart)))
                                    {                                                 
                                        ch = MIDITxBuffer[MIDITxTail].MIDICommand;
                                        while (!UARTTransmitterIsReady(MIDIuart));
                                        UARTSendDataByte(MIDIuart, ch);    
                                        INTEnable(INT_SOURCE_UART_TX(MIDIuart), INT_ENABLED);
                                        MIDITxState = SEND_SERVO_ID; // Tx state 0 is done, so go to next.
                                    }                            
                                }
                                else if (DiagnosticsEnable || ResetFlag) printf("\rUSB OUT #%d: %i, ", i, averageValue);
                            }
                        }
                    }                
                } // end if (ADint) 
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
            printf("\rRECEIVED: %s", HOSTRxBuffer);
            if (setupCommand == ENABLE_POTS)
            {
                for (i = 0; i < MAXPOTS; i++) 
                    arrPotEnable[i] = false;
                i = 0;
                do {
                    ch = HOSTRxBuffer[i];
                    if (ch == CR) break;
                    // if ((ch >= '0' && ch <= '9')||(ch >= 'A' && ch <= 'B'))
                    if (ch >= '0' && ch <= '9')
                    {
                        if (ch <= '9') arrPotEnable[ch - '0'] = true;
                        else arrPotEnable[ch - 'A' + 10] = true;
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
        if (!USBactive) 
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
unsigned short MIDIStateMachine(void) {
    BYTE ch;
    static unsigned short index = 0;

    while (index != MIDIRxIndex) {
        if (index >= MAXBUFFER) index = 0;
        ch = MIDIRxBuffer[index++];
        if (ch & 0x80) 
        {
            printf("\r%X ", ch);
        } else printf("%X ", ch);
    }
    return (index);
}
*/

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
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) {
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

    USBDeviceInit(); //usb_device.c.  Initializes USB module SFRs and firmware
    //variables to known states.
}//end InitializeSystem





void __ISR(_TIMER_2_VECTOR, ipl5) Timer2Handler(void) 
{
    static unsigned short hundredthSecondTimer = 0;

    mT2ClearIntFlag(); // clear the interrupt flag    

    // milliSecondCounter++;

    /*
    if (MIDItimeout) 
    {
        MIDItimeout--;
        if (!MIDItimeout) MIDIbufferFull = true;
    }
    if (MIDInoteTimeout)
        MIDInoteTimeout--;
    */
    
    hundredthSecondTimer++;
    if (hundredthSecondTimer > 10)
    {
        hundredthSecondTimer = 0;
        ADint = true;
    }
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
    // UARTConfigure(XBEEuart, UART_ENABLE_PINS_TX_RX_ONLY); 
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
    UARTSetDataRate(MIDIuart, SYS_FREQ, 31250);
    UARTEnable(MIDIuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure MIDI Interrupts
    INTEnable(INT_SOURCE_UART_TX(MIDIuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(MIDIuart), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(MIDIuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(MIDIuart), INT_SUB_PRIORITY_LEVEL_0);
    
    /*
    // Set up DMX512 UART @ 25000 baud   	
    UARTConfigure(DMXuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(DMXuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(DMXuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_2);
    UARTSetDataRate(DMXuart, SYS_FREQ, 250000);
    UARTEnable(DMXuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure DMX interrupts
    INTEnable(INT_SOURCE_UART_RX(DMXuart), INT_ENABLED);
    INTEnable(INT_SOURCE_UART_TX(DMXuart), INT_DISABLED);
    INTSetVectorPriority(INT_VECTOR_UART(DMXuart), INT_PRIORITY_LEVEL_1);
    INTSetVectorSubPriority(INT_VECTOR_UART(DMXuart), INT_SUB_PRIORITY_LEVEL_0);
    */

    // Set up Port E outputs:
    PORTSetPinsDigitalOut(IOPORT_E, BIT_0 | BIT_1 | BIT_2 | BIT_3 | BIT_4);
    PORTSetBits(IOPORT_E, BIT_0 | BIT_1 | BIT_2 | BIT_3);

    // Set up Port E outputs:
    PORTSetPinsDigitalIn(IOPORT_E, BIT_6 | BIT_7);

    // Set up Timer 2 for 100 microsecond roll-over rate
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_64, 1250);
    // set up the core timer interrupt with a priority of 5 and zero sub-priority
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_5);

    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();

}//end UserInit




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

        if (UARTReceivedDataIsAvailable(XBEEuart)) {
            ch = UARTGetDataByte(XBEEuart);
            if (XBEERxIndex < MAXBUFFER) XBEERxBuffer[XBEERxIndex++] = ch;
            if (ch == '\r') 
            {
                XBEEBufferFull = true;
                XBEERxBuffer[XBEERxIndex++] = '\0';
                XBEERxIndex = 0;
            }
        }
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
            XBEEonly = false;
            if (ch != 0xfe) 
            {
                if (MIDIRxHead >= MAXBUFFER) MIDIRxHead = 0;
                MIDIRxBuffer[MIDIRxHead++] = ch;
            }
        }
    }

    if (INTGetFlag(INT_SOURCE_UART_TX(MIDIuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(MIDIuart));
        if (MIDITxTail!=MIDITxHead) 
        {
            if (MIDITxState==SEND_MIDI_COMMAND)            
                ch = MIDITxBuffer[MIDITxTail].MIDICommand;                          
            else if (MIDITxState==SEND_SERVO_ID)
                ch = MIDITxBuffer[MIDITxTail].MIDIServoNumber;
            else ch = MIDITxBuffer[MIDITxTail].servoData;
            
            while (!UARTTransmitterIsReady(MIDIuart));
            UARTSendDataByte(MIDIuart, ch);
            MIDITxState++;
            if (MIDITxState > SEND_SERVO_DATA)
            {                
                MIDITxState = SEND_MIDI_COMMAND;
                MIDITxTail++;
                if (MIDITxTail >= MAXBUFFER) MIDITxTail = 0;
            }
        } else INTEnable(INT_SOURCE_UART_TX(MIDIuart), INT_DISABLED);
    }
}

BYTE ProcessUSB(void)
{
    static BYTE LEDflag = false;
    static short LEDcounter = 0;
    static unsigned short lowServoByte = 0, highServoByte = 0;
    short servoValue = 0;
    BYTE ch, MIDIcommand = 0, command = 0, ServoNumber = 0, frame = 0;
    static unsigned short BeatNumber = 0x0000;
   
    if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) return false;
    
    BlinkUSBStatus();

    if (!USBHandleBusy(USBRxHandle)) 
    {
        USBRxHandle = USBRxOnePacket(MIDI_EP, ReceivedDataBuffer, 4);
        MIDIcommand = ReceivedDataBuffer[1];
        if (MIDIcommand == 0xF1) 
        {
            if (TEST_OUT) TEST_OUT = 0;
            else TEST_OUT = 1;
            frame = ReceivedDataBuffer[2];
            printf("\r>FRM: %02X", frame);
        }        
        else if (MIDIcommand == 0xF8) 
        {            
            printf("\r>BEAT: %d", BeatNumber++);
            if (TEST_OUT) TEST_OUT = 0;
            else TEST_OUT = 1;            
        }
        else if ((MIDIcommand & 0xF0) != 0xB0)
            printf("\r>???: %02X, %02X, %02X, %02X", ReceivedDataBuffer[0], ReceivedDataBuffer[1], ReceivedDataBuffer[2], ReceivedDataBuffer[3]);
        else 
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
                    DelayMs(1);
                }
                printf("USB RX: %02X Servo #%d: %d", command, ServoNumber, servoValue);                
            }
        }
            
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
    
    if (USBTxTail!=MIDITxHead && !USBHandleBusy(USBTxHandle))
    {           
        midiData.Val = 0;   //must set all unused values to 0 so go ahead
                    
        midiData.CableNumber = 0;
        midiData.CodeIndexNumber = MIDI_CIN_CONTROL_CHANGE;
        midiData.DATA_0 = MIDITxBuffer[USBTxTail].MIDICommand;
        midiData.DATA_1 = MIDITxBuffer[USBTxTail].MIDIServoNumber;
        midiData.DATA_2 = MIDITxBuffer[USBTxTail].servoData;
                
        USBTxHandle = USBTxOnePacket(MIDI_EP,(BYTE*)&midiData,4);
        USBTxTail++;
        if (USBTxTail >= MAXBUFFER) USBTxTail = 0;
    }    
    return true;
}//end ProcessUSB    

BYTE ProcessMIDI_RS232(void)
{
    static unsigned short lowServoByte = 0, highServoByte = 0;
    short servoValue = 0;
    static BYTE ch, command = 0, ServoNumber = 0;
    static unsigned short MIDIRxState = 0;    
    static unsigned short RxCounter = 0;
    
    while(MIDIRxTail!=MIDIRxHead)
    {
        ch = MIDIRxBuffer[MIDIRxTail];        
        MIDIRxTail++;
        if (MIDIRxTail>=MAXBUFFER) MIDIRxTail=0;
        if (ch & 0x80)
        {
            if ((ch & 0xF0)==0xB0) MIDIRxState = SEND_MIDI_COMMAND;
            else MIDIRxState = 0;
        }        
        
        if (!MIDIRxState) 
            return false;        
        else if (MIDIRxState==SEND_MIDI_COMMAND) 
        {
            command = ch;
            MIDIRxState++;
        }
        else if (MIDIRxState==SEND_SERVO_ID) 
        {
            ServoNumber = ch-1;
            MIDIRxState++;
        }
        else // if (MIDIRxState==SEND_SERVO_DATA) 
        {
            MIDIRxState = 0;
            if (0 == (ch & 0b01000000)) lowServoByte = (unsigned short)ch;
            else {
                highServoByte = (unsigned short) (ch & 0b10111111);
                highServoByte = highServoByte << 5;
                servoValue = lowServoByte | highServoByte;                
                outData[0] = servoValue; 
                if (DiagnosticsEnable) printf ("\rRx %d Servo %d: %d", RxCounter++, ServoNumber, servoValue);
                
                if (ServoNumber >= 0)
                {
                    if ( !BuildXBEEpacket(command, ServoNumber, 1, outData) )
                    {
                       if (DiagnosticsEnable) printf("\rMIDI RX #%d: XBEE OVERRUN ERROR!", errorCounter++);
                    }
                    else if (!INTGetEnable(INT_SOURCE_UART_TX(XBEEuart)))
                    {
                        ch = XBEETxBuffer[XBEETxTail++];
                        if (XBEETxTail >= MAXXBEEBUFFER) XBEETxTail = 0;                
                        while (!UARTTransmitterIsReady(XBEEuart));
                        UARTSendDataByte(XBEEuart, ch);
                        INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_ENABLED);
                    }                    
                }
            }
        }            
    }    
    return true;
}//end ProcessUSB    

BYTE BuildXBEEpacket(BYTE command, BYTE ServoNumber, BYTE numData, short *ptrData)
{
	int i, j;    
    BYTE arrOutputBytes[MAXBUFFER];
	short packetIndex = 0, packetLength = 0, numBytes = 0;
    static short totalPacketLength = 0, packetCount = 0;
    BYTE dataByte;  
    BYTE TxPacket[MAXBUFFER];  
    short TempHead;
    // BYTE InterruptFlag = false;      
    
    /*
    if (INTGetEnable(INT_SOURCE_UART_TX(XBEEuart)))
    {
        INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_DISABLED);
        InterruptFlag = true;
    }      
    */
    if (numData > MAXBUFFER) 
    {
        printf("\r#1 DATA > MAX");
        //if (InterruptFlag) INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_ENABLED);
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
                printf("\r#2 PACKET > MAX");
                //if (InterruptFlag) INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_ENABLED);
                return false;
            }
			TxPacket[packetIndex++] = DLE;
        }
		if (dataByte == ETX) dataByte = ETX - 1;
        if (packetIndex >= MAXBUFFER) 
        {
            printf("\r#3 PACKET > MAX");
            //if (InterruptFlag) INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_ENABLED);
            return false;        
        }
		TxPacket[packetIndex++] = dataByte;
	}
    if (packetIndex >= MAXBUFFER) 
    {
        // if (InterruptFlag) INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_ENABLED);        
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
        printf("\r#4 HEAD == TAIL, LENGTH: %d, TAIL: %d, COUNT; %d, TOTAL: %d", packetLength, XBEETxTail, packetCount, totalPacketLength);
        packetCount = 0;
        totalPacketLength = 0;
        XBEETxHead = XBEETxTail;
        INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_DISABLED);
        return false; // Not enough room! Return false to indicate overrun error.
    }
    else packetCount++;
    
    // Ready to go. Copy packet to transmit buffer:
    for (i = 0; i < packetLength; i++)
    {        
        XBEETxBuffer[XBEETxHead++] = TxPacket[i];
        if (XBEETxHead >= MAXXBEEBUFFER) XBEETxHead = 0;
    }    
    
    //if (InterruptFlag) INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_ENABLED);
    return true;
}

/** EOF main.c *************************************************/
#endif
