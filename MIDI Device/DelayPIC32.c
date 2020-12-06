/*********************************************************************
 *
 *                  General Delay routines
 *
 *********************************************************************
 * FileName:        DelayPIC32.c
 * Dependencies:    Compiler.h
 * Processor:       PIC32
 * Compiler:        Microchip C32 v1.05 or higher
 *
 * JBS:				9-16-20: tweaked for UBW32 board @80 Mhz I think
 ********************************************************************/
#include "Delay.h"

void DelayMs(long ms){
unsigned char i;

    while(ms--)
    {
        i=4;
        while(i--) Delay10us(25);
    }
}

void Delay10us(long dwCount){
unsigned short i;

	while(dwCount--)	
		for (i=0; i<87; i++);  
}

void DelayUs(unsigned short Count)
{
unsigned short i;

	while(Count--)	
		for (i=0; i<7; i++);
}
