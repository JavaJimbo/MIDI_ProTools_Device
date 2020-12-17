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
 * 12-16-20:    Works now with multiple sevos recording and playing back on ProTools.
 ************************************************************************************************************/

#ifndef MAIN_C
#define MAIN_C
#define TEST_OUT LATEbits.LATE4
#define ENTER 13
#define CR ENTER
#define BACKSPACE 8


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

// unsigned char ReceivedDataBuffer[64] RX_BUFFER_ADDRESS_TAG;
BYTE ReceivedDataBuffer[64];
unsigned char ToSendDataBuffer[64] TX_BUFFER_ADDRESS_TAG;
USB_AUDIO_MIDI_EVENT_PACKET midiData MIDI_EVENT_ADDRESS_TAG;



USB_HANDLE USBTxHandle = 0;
USB_HANDLE USBRxHandle = 0;


int ADC10_ManualInit(void);
void BlinkUSBStatus(void);
static void InitializeSystem(void);
unsigned char ProcessUSB(void);
void UserInit(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
short BuildPacket(unsigned char command, unsigned char subcommand, unsigned char numData, short *ptrData, unsigned char *ptrPacket);
extern unsigned short CalculateModbusCRC(unsigned char *input_str, short num_bytes);

#define MIDIuart UART1
#define MIDIbits U1STAbits
#define MIDI_VECTOR _UART_1_VECTOR

#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR

#define XBEEuart UART4
#define XBEEbits U4STAbits
#define XBEE_VECTOR _UART_4_VECTOR

//#define DMXuart UART5
//#define DMXbits U5STAbits
//#define DMX_VECTOR _UART_5_VECTOR

#define MAXBUFFER 256
unsigned char HOSTRxBuffer[MAXBUFFER];
unsigned char HOSTRxBufferFull = false;

unsigned short XBEERxIndex = 0;
unsigned short XBEERxLength = 0;
unsigned char XBEERxBuffer[MAXBUFFER];
unsigned short XBEETxLength = 0;
unsigned short XBEETxIndex = 0;
unsigned char XBEETxBuffer[MAXBUFFER];
short outData[MAXBUFFER];

unsigned short MIDITxLength = 0;
unsigned short MIDITxIndex = 0;
unsigned char MIDITxBuffer[MAXBUFFER];
unsigned char MIDIRxBuffer[MAXBUFFER];
unsigned short MIDIRxIndex = 0;

/*
unsigned char DMXRxBuffer[MAXBUFFER + 1];
unsigned char DMXTxBuffer[MAXBUFFER + 1];
unsigned char DMXflag = false;
static unsigned char DMXstate = DMX_STANDBY;
*/

#define MAXPOTS 10
unsigned short ADresult[MAXPOTS];
unsigned short ADpots[MAXPOTS];

void ConfigAd(void);

// unsigned short milliSecondCounter = 0;


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
    unsigned char ID;
    unsigned short position;
    unsigned char updated;
    unsigned char enabled;
};

unsigned short MIDIStateMachine(void);
// short MIDItimeout = 0;
unsigned char MIDIbufferFull = false;

#define MIDI_TIMEOUT 2

#define STANDBY 0
#define PLAY 1
#define RECORD 2
#define MIDI_MODE 3
#define TEST_MODE 4


const unsigned char MIDIpercussionCommand[7] = {0xB0, 0x00, 0x7F, 0x20, 0x00, 0xC0, 0x00};
unsigned short MIDInoteTimeout = 0;

unsigned char ADint = false;
#define FILTERSIZE 16
long arrPotValue[MAXPOTS][FILTERSIZE];
unsigned char arrPotEnable[MAXPOTS];
#define MAXSERVOS MAXPOTS
long arrServoValue[MAXSERVOS];
unsigned char arrServoDataReady[MAXSERVOS];
long servoLow = 0, servoHigh = 0;

enum {
    IDLE = 0,    
    SEND_LOW,
    SEND_HI
};

unsigned char MIDIState[MAXSERVOS];
unsigned char setupCommand = 0, controlCommand = 0x00;


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

#define ENABLE_POTS 16
unsigned char USBrunning = false;
int main(void) 
{
    unsigned char UserButtonState = 1, ButtonRead, ch = 0;
    short i, j, k, UserButtonDebouncer = 0;    
    unsigned char TestEnable = false;
    short filterIndex = 0;
    long averageValue = 0;
    long sumValue;    
    unsigned char command, subCommand;
    
    for (i = 0; i < MAXPOTS; i++)
    {        
        for (j = 0; j < FILTERSIZE; j++) 
            arrPotValue[i][j] = 0x0000;
    }
    
    for (i = 0; i < MAXSERVOS; i++)
    {
        arrServoValue[i] = 0x0000;
        arrPotEnable[i] = false;
        MIDIState[i] = IDLE;
    }
    
    
    InitializeSystem();
    
    DelayMs(200);

    printf("\r\rTESTING NO USB MODE #1\r\r");

    mLED_1_Off();
    mLED_2_Off();
    mLED_3_Off();
    mLED_4_Off();
    
    while (1) 
    {
        if (controlCommand)
        {
            printf("\rCONTROL COMMAND %d: ", controlCommand);
            if (controlCommand == ENABLE_POTS) 
            {
                printf("ENABLE POTS: ");
                setupCommand = ENABLE_POTS;
            }
            else if (controlCommand == 17 || controlCommand == 24 || controlCommand == 3)
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
            //printf("\r#%d:", trialNum++);
            //for (i = 0; i < MAXPOTS; i++) printf(" %d,", ADpots[i]);                        
            
            // printf("\r#%d: ", trialNum++);
            for (i = 0; i < MAXPOTS; i++)
            {
                arrPotValue[i][filterIndex] = ADpots[i];
                //sumValue = 0;
                //for (j = 0; j < FILTERSIZE; j++) 
                //    sumValue = sumValue + (long)arrPotValue[i][j];
                //averageValue = sumValue / FILTERSIZE;
                //printf("%i, ", averageValue);
            }
            filterIndex++;
            if (filterIndex >= FILTERSIZE) filterIndex = 0;            
                        
            for (i = 0; i < MAXPOTS; i++)
            {         
                if (arrPotEnable[i])
                {
                    sumValue = 0;
                    for (j = 0; j < FILTERSIZE; j++) 
                        sumValue = sumValue + (long)arrPotValue[i][j];
                    averageValue = sumValue / FILTERSIZE;                
                    if (abs(averageValue - arrServoValue[i]) > 1)
                    {                               
                        arrServoDataReady[i] = true;
                        if (averageValue >= 1023) averageValue = 1023;                    
                        arrServoValue[i] = averageValue;
                    
                        servoHigh = (averageValue / 32) | 0b01000000;
                        servoLow = (averageValue - (servoHigh * 32));

                        if (TestEnable) printf("\r>>#%d: %i, ", i, averageValue);
                        
                        if (!USBrunning)
                        {
                            outData[0] = averageValue; 
                            command = 0xB0;
                            subCommand = i;                                    
                            XBEETxLength = BuildPacket(command, subCommand, 1, outData, XBEETxBuffer);
                            while(!UARTTransmitterIsReady(XBEEuart));
                            UARTSendDataByte (XBEEuart, XBEETxBuffer[0]);
                            XBEETxIndex = 1;
                            INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_ENABLED);                                                        
                        }                        
                        else if (!MIDIState[i])
                        {                            
                            k = i * 2;
                            MIDITxBuffer[k+0] = (unsigned char) servoLow;
                            MIDITxBuffer[k+1] = (unsigned char) servoHigh;                                        
                            MIDIState[i] = SEND_LOW;                               
                        }                        
                    }
                }
            }            
        }
        
        // MIDIStateMachine(); // Check MIOI UART for incoming data
        
#define USER_PUSHBUTTON PORTEbits.RE6
#define PROGRAM_PUSHBUTTON PORTEbits.RE7        
        
        // USER pushbutton toggles AD pot reading on and off.
        // TestEnable is set to true to enable AD converter.
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
                    if (TestEnable)
                    {
                        TestEnable = false;
                        printf("\rTEST DISABLED");
                    }
                    else 
                    {
                        TestEnable = true;
                        printf("\rTEST ENABLED");
                    }
                }
            }
            DelayMs(10);
        }
        else UserButtonDebouncer = 0;
        
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
        // Application-specific tasks.
        // Application related code may be added here, or in the ProcessUSB() function.
        USBrunning = ProcessUSB();
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


unsigned short MIDIStateMachine(void) {
    unsigned char ch;
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
    unsigned char ch;
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


/*
void __ISR(MIDI_VECTOR, ipl2) IntMIDIHandler(void) 
{    
    unsigned char ch;

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
                MIDItimeout = MIDI_TIMEOUT;
                if (MIDIRxIndex >= MAXBUFFER) MIDIRxIndex = 0;
                MIDIRxBuffer[MIDIRxIndex++] = ch;
            }
        }
    }

    if (INTGetFlag(INT_SOURCE_UART_TX(MIDIuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(MIDIuart));
        if (MIDITxLength) 
        {
            if (MIDITxIndex < MAXBUFFER) ch = MIDITxBuffer[MIDITxIndex++];
            while (!UARTTransmitterIsReady(MIDIuart));
            UARTSendDataByte(MIDIuart, ch);
            if (MIDITxIndex >= MIDITxLength) {
                INTEnable(INT_SOURCE_UART_TX(MIDIuart), INT_DISABLED);
                MIDITxLength = 0;
                MIDITxIndex = 0;
            }
        } else INTEnable(INT_SOURCE_UART_TX(MIDIuart), INT_DISABLED);
    }
}
*/

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


short BuildPacket(unsigned char command, unsigned char subcommand, unsigned char numData, short *ptrData, unsigned char *ptrPacket)
{
	int i, j;
    unsigned char arrOutputBytes[64];
	short packetIndex = 0, numBytes = 0;
    unsigned char dataByte;    
    
    union
    {
        unsigned char b[2];
        unsigned short integer;
    } convert;	

	j = 0;
	// Header first
	arrOutputBytes[j++] = command;
	arrOutputBytes[j++] = subcommand;
	arrOutputBytes[j++] = numData;

	// Convert integer data to unsigned chars    
	for (i = 0; i < numData; i++)
	{
		convert.integer = ptrData[i];
		arrOutputBytes[j++] = convert.b[0];
		arrOutputBytes[j++] = convert.b[1];
	}

	convert.integer = CalculateModbusCRC(arrOutputBytes, j);      
    
	arrOutputBytes[j++] = convert.b[0];
	arrOutputBytes[j++] = convert.b[1];
	numBytes = j;

	if (numBytes <= (MAXBUFFER + 16))
	{
        packetIndex = 0;
		ptrPacket[packetIndex++] = STX;
		for (i = 0; i < numBytes; i++)
		{
			dataByte = arrOutputBytes[i];
			if (dataByte == STX || dataByte == DLE || dataByte == ETX)
				ptrPacket[packetIndex++] = DLE;
			if (packetIndex >= MAXBUFFER) return 0;
			if (dataByte == ETX) dataByte = ETX - 1;
			ptrPacket[packetIndex++] = dataByte;
		}
		ptrPacket[packetIndex++] = ETX;
		return (packetIndex);
	}
	else return 0;
}


void __ISR(XBEE_VECTOR, ipl2) IntXBEEHandler(void) 
{    
    unsigned char ch;

    if (XBEEbits.OERR || XBEEbits.FERR) 
    {
        if (UARTReceivedDataIsAvailable(XBEEuart))
            ch = UARTGetDataByte(XBEEuart);
        XBEEbits.OERR = 0;
    } else if (INTGetFlag(INT_SOURCE_UART_RX(XBEEuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(XBEEuart));

        if (UARTReceivedDataIsAvailable(XBEEuart)) {
            ch = UARTGetDataByte(XBEEuart);
            if (ch != 0xfe) 
            {
                // XBEEtimeout = XBEE_TIMEOUT;
                if (XBEERxIndex >= MAXBUFFER) XBEERxIndex = 0;
                XBEERxBuffer[XBEERxIndex++] = ch;
            }
        }
    }

    if (INTGetFlag(INT_SOURCE_UART_TX(XBEEuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(XBEEuart));
        if (XBEETxLength) 
        {
            if (XBEETxIndex < MAXBUFFER) ch = XBEETxBuffer[XBEETxIndex++];
            while (!UARTTransmitterIsReady(XBEEuart));
            UARTSendDataByte(XBEEuart, ch);
            if (XBEETxIndex >= XBEETxLength) 
            {
                INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_DISABLED);
                XBEETxLength = 0;
                XBEETxIndex = 0;
            }
        } else INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_DISABLED);
    }
}

void __ISR(_TIMER_2_VECTOR, ipl5) Timer2Handler(void) 
{
    static unsigned short hundredthSecondTimer = 0;

    mT2ClearIntFlag(); // clear the interrupt flag
    
    if (TEST_OUT) TEST_OUT = 0;
    else TEST_OUT = 1;

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
    UARTSetFifoMode(XBEEuart, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(XBEEuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(XBEEuart, SYS_FREQ, 57600);
    UARTEnable(XBEEuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure XBEE Interrupts
    INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_DISABLED);
    INTEnable(INT_SOURCE_UART_RX(XBEEuart), INT_DISABLED);
    INTSetVectorPriority(INT_VECTOR_UART(XBEEuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(XBEEuart), INT_SUB_PRIORITY_LEVEL_0);
    
    /*    
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
    */
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

unsigned char ProcessUSB(void)
{
    static short servoNumber = 0;
    static unsigned char LEDflag = false;
    static short LEDcounter = 0;
    static unsigned short lowServoByte = 0, highServoByte = 0;
    short servoValue = 0;
    char command = 0, subcommand = 0, lastServo = false;
    short k = 0;
    
    if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) return false;
    
    BlinkUSBStatus();

    if (!USBHandleBusy(USBRxHandle)) 
    {
        USBRxHandle = USBRxOnePacket(MIDI_EP, ReceivedDataBuffer, 4);
        if ((ReceivedDataBuffer[1] & 0xF0) != 0xB0)
            printf("\r>%02X, %02X, %02X, %02X", ReceivedDataBuffer[0], ReceivedDataBuffer[1], ReceivedDataBuffer[2], ReceivedDataBuffer[3]);
        else {
            if (0 == (ReceivedDataBuffer[3] & 0b01000000)) lowServoByte = (unsigned short)ReceivedDataBuffer[3];
            else {
                highServoByte = (unsigned short) (ReceivedDataBuffer[3] & 0b10111111);
                highServoByte = highServoByte << 5;
                servoValue = lowServoByte | highServoByte;
                
                outData[0] = servoValue; 
                command = ReceivedDataBuffer[1];
                subcommand = ReceivedDataBuffer[2]-1;       
                
                printf("\rUSB RX Servo #%d: %d", subcommand, servoValue);                
                
                if (subcommand >= 0)
                {
                    XBEETxLength = BuildPacket(command, subcommand, 1, outData, XBEETxBuffer);
                    while(!UARTTransmitterIsReady(XBEEuart));
                    UARTSendDataByte (XBEEuart, XBEETxBuffer[0]);
                    XBEETxIndex = 1;
                    INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_ENABLED);
                }
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
    
    lastServo = false;
    while (!MIDIState[servoNumber])
    {
        servoNumber++;
        if (servoNumber > MAXSERVOS) 
        {            
            servoNumber = 0;
            if (lastServo) break;
            else lastServo = true;
        }
    }
    
    if (MIDIState[servoNumber] && !USBHandleBusy(USBTxHandle))
    {           
        k = servoNumber * 2;
        midiData.Val = 0;   //must set all unused values to 0 so go ahead
                    
        midiData.CableNumber = 0;
        midiData.CodeIndexNumber = MIDI_CIN_CONTROL_CHANGE;
        midiData.DATA_0 = 0xB0; 
                
        if (MIDIState[servoNumber] == SEND_LOW) 
        {
            midiData.DATA_1 = servoNumber+1;
            midiData.DATA_2 = MIDITxBuffer[k+0]; // (BYTE)(servoLow);
            MIDIState[servoNumber] = SEND_HI;
        }
        else
        {
            midiData.DATA_1 = servoNumber+1;
            midiData.DATA_2 = MIDITxBuffer[k+1];// (BYTE)(servoHigh);
            MIDIState[servoNumber] = IDLE;
        }
        USBTxHandle = USBTxOnePacket(MIDI_EP,(BYTE*)&midiData,4);
    }    
    return true;
}//end ProcessUSB    

/** EOF main.c *************************************************/
#endif
