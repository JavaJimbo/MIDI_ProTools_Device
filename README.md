# MIDI_ProTools_Device 

This firmware was designed for a PIC 32MX795F512L microcontroller on a Sparkfun UBW32 board.
When the UBW32 is connected to a USB port, the PC recognizes it as a MIDI USB device.
This code was designed to record and play back MIDI data stored on the PC using ProTools.
The MIDI data is received via USB and is transmitted at 57600 baud using an XBEE wireless transmitter 
connected to one of the PIC UARTs. It can then be received by a slave PIC which uses the
ProTools MIDI data to control servo motors.
