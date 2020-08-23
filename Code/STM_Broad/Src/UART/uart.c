/*
 * uart.c
 *
 *  Created on: Aug 21, 2020
 *      Author: ASUS
 */

#include "main.h"

enum
{
	STOP = 0,
	HANDLE
};

static uint16_t UARTlengthcount=0;
static uint8_t copystate=STOP;

void getdata()
{
	switch (copystate) {
		case STOP:
			if(UARTgetchar=='[')
			{
				copystate=HANDLE;
				UARTlengthcount=0;
			}
			break;
		case HANDLE:
			if(UARTgetchar==']')
			{
				copystate=STOP;
				newblockdata==1;
			}
			else if(UARTgetchar=='[')
			{
				UARTlengthcount=0;
			}
			else
				UARTbuffer[UARTlengthcount++]==UARTgetchar[0];
			break;
		default:
			break;
	}
}


