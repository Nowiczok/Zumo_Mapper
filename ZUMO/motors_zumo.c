#include "motors_zumo.h"

void ZUMO_init()
{
	//enable clock for TPM0
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	//set clock source
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	//GPIO init
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	PORTB->PCR[11] = PORT_PCR_MUX(2);
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
	PORTA->PCR[5] = PORT_PCR_MUX(2);
	
	//TPM0 init
	TPM0->SC |= TPM_SC_PS(7);  //prescaler					
	TPM0->SC |= TPM_SC_CMOD(1);		// clock mode			
	TPM0->SC &= ~TPM_SC_CPWMS_MASK; //up-counting
	TPM0->MOD = 400;

	//channel 0 init
	TPM0->CONTROLS[0].CnSC |= (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK);  //Edge-aligned PWM, high-true

	//channel 5 init
	TPM0->CONTROLS[5].CnSC |= (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK);
	
	PTB->PDDR |= (1<<10 | 1<<7); //set ptb10 and ptb7 as output
	PORTB->PCR[10] = PORT_PCR_MUX(1);
	PORTB->PCR[7] = PORT_PCR_MUX(1);
	
}
void ZUMO_setLeftSpeed(int16_t speed)
{
	uint8_t rev = 0;
	if(speed < 0)
	{
		speed = -speed;
		rev = 1;
	}
	TPM0->CONTROLS[5].CnV = speed;
	if(!rev)
	{
		PTB->PDOR |= 1<<10;
	}else
	{
		PTB->PDOR &= ~(1<<10);
	}
}

void ZUMO_setRightSpeed(int16_t speed)
{
	uint8_t rev = 0;
	if(speed < 0)
	{
		speed = -speed;
		rev = 1;
	}
	TPM0->CONTROLS[0].CnV = speed;
	if(!rev)
	{
		PTB->PDOR |= 1<<7;
	}else
	{
		PTB->PDOR &= ~(1<<7);
	}
}
void ZUMO_setSpeeds(int16_t leftSpeed, int16_t rightSpeed);