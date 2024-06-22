
/************************************************************************/
/* 
 ATMEGA32				  L298 (motor drive)
|-------------|			  |-----------|
|		 PC2  |___________| IN1		  |
|		 PC3  |___________| IN2	      |
|			  |			  |			  |
|	     PC4  |___________| IN1	      |
|		 PC5  |___________| IN2	      |
|			  |			  |			  |
|  (PWM2)PD7  |___________| ENA	      |
|  (PWM0)PB3  |___________| ENB	      |
|	  	      |			  |	  		  |
|	  		  |			  |	  		  |
|			  |			  |			  |
|-------------|			  |-----------|
                                                     */
/************************************************************************/


#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include "lowPassFilter.h"
#include "encoder.h"
#include "checkBetween.h"
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define Motor_controll_DDR	DDRA
#define Motor_controll_PORT	PORTA
#define Motor_controll_PIN	PINA

//#define STBY				PC4		//Standby
#define AIN1				PA0		//logic input FORWARD Motor A
#define AIN2				PA1		//logic input REVERSE Motor A
#define BIN1				PA2		//logic input FORWARD Motor B
#define BIN2				PA3		//logic input REVERSE Motor B
#define PWMA				PD7		//connected to ENA in L298
#define PWMB				PB3		//connected to ENB in L298

#define PWMA_DDR			DDRD
#define PWMB_DDR			DDRB
#define THRESHOLD			80	//minimum PWM for the motors to start

//#define PWM_LPF_GAIN		MOTOR_PWM_LPF_GAIN


void driveMotor(int motorA,int motorB);
void INIT_MotorController(void);

void driveMotor(int motorA,int motorB)
{
	if (motorA>0)
	{
		//positive direction
		Motor_controll_PORT |= 1<<AIN1;
		Motor_controll_PORT &= ~(1<<AIN2);
		OCR2 = constrain(motorA,0,255);
	}
	//If the PWM value is negative then turn the rotate to the opposite direction and use abs(PWM value)
	else if (motorA<=0)
	{
		//negative direction
		Motor_controll_PORT |= 1<<AIN2;
		Motor_controll_PORT &= ~(1<<AIN1);
		OCR2 = constrain(abs(motorA),0,255);
 	}
	if (motorB>0)
	{	
		//positive direction	
		Motor_controll_PORT |= 1<<BIN1;
		Motor_controll_PORT &= ~(1<<BIN2);
		OCR0 = constrain(motorB,0,255);
	}
// 	//If the PWM value is negative then turn the rotate to the opposite direction and use abs(PWM value) 
 	else if (motorB<=0)
	{
		//negative direction
		Motor_controll_PORT |= 1<<BIN2;
		Motor_controll_PORT &= ~(1<<BIN1);
		OCR0 = constrain(abs(motorB),0,255);
	}	
}

void INIT_MotorController(void)
{
	Motor_controll_DDR |= 1<<AIN1 | 1<<AIN2 | 1<<BIN1 | 1<<BIN2;
	//Motor_controll_PORT|= 1<<STBY;
	PWMA_DDR |= 1<<PWMA;
	PWMB_DDR |= 1<<PWMB;
}
#endif /* INCFILE1_H_ */