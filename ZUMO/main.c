#include "MKL05Z4.h"
#include "motors_zumo.h"
#include "uart0.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define LF	0xa		// Enter
#define MOTOR_FOR 180
#define MOTOR_TURN 200

char rx_buf[16];

char Forward[] = "F";
char Left[] = "L";
char Right[] = "R";
char Stop[] = "S";
char Back[] = "B";

char Error[] = "Wrong command";
char TooLong[] = "Too long command";

uint8_t rx_buf_pos = 0;
uint8_t rx_FULL = 0;
uint8_t too_long = 0;
char temp, buf;


void UART0_IRQHandler()
{
	if(UART0->S1 & UART0_S1_RDRF_MASK)
	{
		temp=UART0->D;	// Odczyt wartosci z bufora odbiornika i skasowanie flagi RDRF
		if(!rx_FULL)
		{
			if(temp!=LF)
			{
				if(!too_long)	// Jesli za dlugi ciag, ignoruj reszte znaków
				{
					rx_buf[rx_buf_pos] = temp;	// Kompletuj komende
					rx_buf_pos++;
					if(rx_buf_pos==16)
						too_long=1;		// Za dlugi ciag
				}
			}
			else
			{
				if(!too_long)	// Jesli za dlugi ciag, porzuc tablice
					rx_buf[rx_buf_pos] = 0;
				rx_FULL=1;
			}
		}
	NVIC_EnableIRQ(UART0_IRQn);
	}
}

int main(void)
{
	uint8_t i;
	UART0_Init();
	ZUMO_init();
	ZUMO_setLeftSpeed(0);
	ZUMO_setRightSpeed(0);
	while(1)
	{
		if(rx_FULL)		// Czy dana gotowa?
		{
			
			//pytanie czy jest sens to zostawiac//
			if(too_long)
			{
				for(i=0;TooLong[i]!=0;i++)	// Zbyt dlugi ciag
					{
						while(!(UART0->S1 & UART0_S1_TDRE_MASK));	// Czy nadajnik gotowy?
						UART0->D = TooLong[i];
					}
					while(!(UART0->S1 & UART0_S1_TDRE_MASK));	// Czy nadajnik gotowy?
					UART0->D = 0xa;		// Nastepna linia
					too_long=0;
			}
			////////////////////////
			else
			{				
				if(strcmp (rx_buf,Forward)==0)	// Forward
				{
					ZUMO_setLeftSpeed(MOTOR_FOR);
					ZUMO_setRightSpeed(MOTOR_FOR);
				}
				
				if(strcmp (rx_buf,Back)==0)	// Backward
				{
					ZUMO_setLeftSpeed(-MOTOR_FOR);
					ZUMO_setRightSpeed(-MOTOR_FOR);
				}
				
				else if(strcmp (rx_buf,Right)==0) // Right
				{
					ZUMO_setLeftSpeed(MOTOR_TURN);
					ZUMO_setRightSpeed(-MOTOR_TURN);
				}
				
        else if(strcmp (rx_buf,Left)==0) // Left
				{
					ZUMO_setLeftSpeed(-MOTOR_TURN);
					ZUMO_setRightSpeed(MOTOR_TURN);
				}
				
				else if(strcmp (rx_buf,Stop)==0) // Stop
				{
					ZUMO_setLeftSpeed(0);
					ZUMO_setRightSpeed(0);
				}
				
        else
					{
						for(i=0;Error[i]!=0;i++)	// Zla komenda
						{
							while(!(UART0->S1 & UART0_S1_TDRE_MASK));	// Czy nadajnik gotowy?
							UART0->D = Error[i];
						}
						while(!(UART0->S1 & UART0_S1_TDRE_MASK));	// Czy nadajnik gotowy?
						UART0->D = 0xa;		// Nastepna linia
					}
				}
			rx_buf_pos=0;
			rx_FULL=0;	// Dana skonsumowana
		}	
	}
}

