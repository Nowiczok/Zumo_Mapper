#include "MKL05Z4.h"

#define PWM_L 10
#define PWM_R 9
#define DIR_L 8 //ptb10
#define DIR_R 7 //ptb7
//tpm0_ch5 => pta5 => arduino pin 10
//tpm_ch0 => ptb11 => arduino pin 9

void ZUMO_init();
void ZUMO_setLeftSpeed(int16_t speed);
void ZUMO_setRightSpeed(int16_t speed);
void ZUMO_setSpeeds(int16_t leftSpeed, int16_t rightSpeed);