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
 ************************************************************************************************************/

#ifndef MAIN_C
#define MAIN_C

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

/** PRIVATE PROTOTYPES *********************************************/
void BlinkUSBStatus(void);
static void InitializeSystem(void);
void ProcessUSB(void);
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

#define MAXBUFFER 128
unsigned char HOSTRxBuffer[MAXBUFFER];
unsigned char HOSTRxBufferFull = false;

unsigned short XBEERxIndex = 0;
unsigned short XBEERxLength = 0;
unsigned char XBEERxBuffer[MAXBUFFER];
unsigned short XBEETxLength = 0;
unsigned short XBEETxIndex = 0;
unsigned char XBEETxBuffer[MAXBUFFER];


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

#define MAXPOTS 1
unsigned short ADresult[MAXPOTS]; // read the result of channel 0 conversion from the idle buffer

void ConfigAd(void);

unsigned short milliSecondCounter = 0;


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
short MIDItimeout = 0;
unsigned char MIDIbufferFull = false;

#define MIDI_TIMEOUT 2

#define STANDBY 0
#define PLAY 1
#define RECORD 2
#define MIDI_MODE 3
#define TEST_MODE 4

unsigned char servoNumber = 3;
const unsigned char MIDIpercussionCommand[7] = {0xB0, 0x00, 0x7F, 0x20, 0x00, 0xC0, 0x00};
unsigned short MIDInoteTimeout = 0;
unsigned short tenthSecondTimeout = 0;

unsigned char ADint = false;
#define FILTERSIZE 16
long arrServoValue[FILTERSIZE];
long servoValue = 0, servoLow = 0, servoHigh = 0;

enum {
    IDLE = 0,
    SEND_LOW,
    SEND_HI
};
unsigned char MIDIState = IDLE;
unsigned char controlCommand = 0x0;

unsigned short outData[MAXBUFFER];

int main(void) 
{
    unsigned char UserButtonState = 1, ButtonRead;
    short i, UserButtonDebouncer = 0;    
    unsigned char TestEnable = false;
    short trialNum = 0;
    short length;
    short filterIndex = 0;
    long averageValue = 0;
    long sumValue;    
    unsigned char ch, textXBEE[MAXBUFFER];    
    
    for (i = 0; i < FILTERSIZE; i++) arrServoValue[i] = 0x0000;
    
    InitializeSystem();
    DelayMs(200);
    printf("\r\rTesting USB SEND and RECEIVE with XBEE transmit #1\r\r");

    mLED_1_Off();
    mLED_2_Off();
    mLED_3_Off();
    mLED_4_Off();
    
    while (1) 
    {
        if (TestEnable && !MIDIState && ADint && !MIDITxLength) 
        {
            ADint = false;
            
            if (filterIndex >= FILTERSIZE) filterIndex = 0;
            arrServoValue[filterIndex++] = ADresult[0];
            
            sumValue = 0;
            for (i = 0; i < FILTERSIZE; i++) sumValue = sumValue + (long)arrServoValue[i];
            averageValue = sumValue / FILTERSIZE;
            
            if (abs(averageValue - servoValue) > 1)
            {       
                servoValue = averageValue;
                if (servoValue >= 1023) servoValue = 1023;
                servoHigh = (servoValue / 32) | 0b01000000;
                servoLow = (servoValue - (servoHigh * 32));
                
                MIDITxBuffer[0] = 0xC0;            
                MIDITxBuffer[1] = servoNumber;
                MIDITxBuffer[2] = (unsigned char) servoLow;
                
                MIDITxBuffer[3] = 0xC0;            
                MIDITxBuffer[4] = servoNumber;
                MIDITxBuffer[5] = (unsigned char) servoHigh;                
                                        
                MIDIState++;          
            
                while (!UARTTransmitterIsReady(MIDIuart));                
                UARTSendDataByte(MIDIuart, MIDITxBuffer[0]);
                printf("\r#%d %02X %02X %02X %02X %02X %02X", trialNum++, MIDITxBuffer[0], MIDITxBuffer[1], MIDITxBuffer[2], MIDITxBuffer[3], MIDITxBuffer[4], MIDITxBuffer[5]);
                MIDITxLength = 6;
                MIDITxIndex = 1;
                INTEnable(INT_SOURCE_UART_TX(MIDIuart), INT_ENABLED);                   
            }
            mAD1IntEnable(INT_ENABLED);
        }
        
        MIDIStateMachine(); // Check MIOI UART for incoming data
        
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
        ProcessUSB();
    }//end while
}//end main        


static void InitializeSystem(void) {
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

void ConfigAd(void) {

    mPORTBSetPinsAnalogIn(BIT_0); // $$$$

    // ---- configure and enable the ADC ----

    // ensure the ADC is off before setting the configuration
    CloseADC10();

    // define setup parameters for OpenADC10
    //                 Turn module on | ouput in integer | trigger mode auto | enable autosample
#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

    // ADC ref external    | disable offset test    | enable scan mode | perform  samples | use dual buffers | use only mux A
#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF

    //                   use ADC internal clock | set sample time
    // #define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_31
#define PARAM3  ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_31 | ADC_CONV_CLK_32Tcy

    //  set AN2 (A2 on Olimex 220 board) input to analog
    // #define PARAM4    ENABLE_AN0_ANA | ENABLE_AN1_ANA| ENABLE_AN2_ANA | ENABLE_AN3_ANA
#define PARAM4    ENABLE_AN0_ANA

    // USE AN0
#define PARAM5 SKIP_SCAN_AN1 | SKIP_SCAN_AN2 | SKIP_SCAN_AN3 |\
SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 |\
SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11 |\
SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15

    // set negative reference to Vref for Mux A
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF);

    // open the ADC
    OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5);

    ConfigIntADC10(ADC_INT_PRI_2 | ADC_INT_SUB_PRI_2 | ADC_INT_ON);

    // clear the interrupt flag
    mAD1ClearIntFlag();

    // Enable the ADC
    EnableADC10();
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




void __ISR(_ADC_VECTOR, ipl6) ADHandler(void) {
    unsigned short offSet;
    unsigned char i;

    mAD1IntEnable(INT_DISABLED);
    mAD1ClearIntFlag();

    // Determine which buffer is idle and create an offset
    offSet = 8 * ((~ReadActiveBufferADC10() & 0x01));

    for (i = 0; i < MAXPOTS; i++)
        ADresult[i] = (unsigned short) ReadADC10(offSet + i); // read the result of channel 0 conversion from the idle buffer
}



#define ENTER 13
#define BACKSPACE 8
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
                HOSTRxBuffer[HOSTRxIndex] = '\0'; // $$$$
                HOSTRxBufferFull = true;
                HOSTRxIndex = 0;
            } else if (ch < 27) controlCommand = ch;
            else if (HOSTRxIndex < MAXBUFFER) {
                HOSTRxBuffer[HOSTRxIndex++] = ch;
                    HOSTRxBufferFull = true;
                    HOSTRxIndex = 0;
            }
        }
    }
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
    }
}



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


void __ISR(_TIMER_2_VECTOR, ipl5) Timer2Handler(void) 
{
    static unsigned short tenthSecondTimer = 0;

    mT2ClearIntFlag(); // clear the interrupt flag

    milliSecondCounter++;

    if (MIDItimeout) 
    {
        MIDItimeout--;
        if (!MIDItimeout) MIDIbufferFull = true;
    }

    if (MIDInoteTimeout)
        MIDInoteTimeout--;

    if (tenthSecondTimeout)
        tenthSecondTimeout--;

    tenthSecondTimer++;
    if (tenthSecondTimer > 10)
    {
        tenthSecondTimer = 0;
        ADint = true;
    }
}

void UserInit(void) 
{
    // Turn off JTAG so we get the pins back
    mJTAGPortEnable(false);

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
    PORTSetPinsDigitalOut(IOPORT_E, BIT_0 | BIT_1 | BIT_2 | BIT_3);
    PORTSetBits(IOPORT_E, BIT_0 | BIT_1 | BIT_2 | BIT_3);

    // Set up Port E outputs:
    PORTSetPinsDigitalIn(IOPORT_E, BIT_6 | BIT_7);
    

    ConfigAd();

    // Set up Timer 2 for 100 microsecond roll-over rate
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_64, 1250);
    // set up the core timer interrupt with a priority of 5 and zero sub-priority
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_5);

    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();

}//end UserInit


void ProcessUSB(void) 
{
    static unsigned char LEDflag = false;
    static short LEDcounter = 0;
    static unsigned short lowServoByte = 0, highServoByte = 0;
    short servoValue = 0;
    unsigned char command = 0, subcommand = 0;
    short outData[MAXBUFFER];
    
    if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) return;
    
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
                printf("\rServo: %d", servoValue);
                
                outData[0] = 0; 
                outData[1] = servoValue;
                outData[2] = 0; 
                outData[3] = 0;             
                command = 0x56;
                subcommand = 0x78;                                    
                XBEETxLength = BuildPacket(command, subcommand, 4, outData, XBEETxBuffer);
                while(!UARTTransmitterIsReady(XBEEuart));
                UARTSendDataByte (XBEEuart, XBEETxBuffer[0]);
                XBEETxIndex = 1;
                INTEnable(INT_SOURCE_UART_TX(XBEEuart), INT_ENABLED);
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
    
    if (MIDIState && !USBHandleBusy(USBTxHandle))
    {           
                    midiData.Val = 0;   //must set all unused values to 0 so go ahead
                    
                    midiData.CableNumber = 0;
                    midiData.CodeIndexNumber = MIDI_CIN_CONTROL_CHANGE;
                    midiData.DATA_0 = 0xB0; 
                    
                    if (MIDIState == SEND_LOW) 
                    {
                        midiData.DATA_1 = servoNumber;
                        midiData.DATA_2 = (BYTE)(servoLow);
                    }
                    else
                    {
                        midiData.DATA_1 = servoNumber;
                        midiData.DATA_2 = (BYTE)(servoHigh);
                    }
                    USBTxHandle = USBTxOnePacket(MIDI_EP,(BYTE*)&midiData,4);
                    MIDIState++;
                    if (MIDIState > SEND_HI) MIDIState = IDLE;
        
    }
}//end ProcessUSB    



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


/** EOF main.c *************************************************/
#endif


