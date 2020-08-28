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
			if(UARTgetchar[0]=='[')
			{
				copystate=HANDLE;
				UARTlengthcount=0;
				memset(UARTbuffer,0,strlen(UARTbuffer));
			}
			break;
		case HANDLE:
			if(UARTgetchar[0]==']')
			{
				copystate=STOP;
				newblockdata=1;
			}
			else if(UARTgetchar[0]=='[')
			{
				UARTlengthcount=0;
				memset(UARTbuffer,0,strlen(UARTbuffer));
			}
			else
				UARTbuffer[UARTlengthcount++]=UARTgetchar[0];
			break;
		default:
			break;
	}
}


