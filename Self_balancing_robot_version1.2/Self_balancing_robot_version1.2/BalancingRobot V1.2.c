

#ifndef F_CPU
#define F_CPU 8000000
#endif


#define RAD_to_DEG		57.295779513082320876798154814105
#define DEG_to_RAD		0.0174532925

#include <avr/io.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <util/atomic.h> 
#include <util/delay.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "checkBetween.h"
#include "balance.h"
#include "i2cmaster.h"
#include "MPU6050.h"
#include "encoder.h"
#include "pid.h"
#include "motorController.h"
#include "USART_serial.h"


//----------USART------------------------------------
#define MYUBRR 12	//8MHz 38400baud 0.2% error UBRR=12
uint8_t UarTemp;


//delta time for PID calculations
float dt = 0;
//PID PARAMETERS
float KP =5.50  ;
float KI = 0.000 ;
float KD = 0.023  ;
int loopCount=0 ;
float angle = 0;
float requestedAngle = -105.00;//-100.199;//-80.00;//-97.356;//-92.366;//-85.236 ;//-100.199 ;-88.403  ;b-85.44; -83.40;
float MOTOR_PWM_LPF_GAIN = 0.4;
PID_t			tiltPID    /* =   {0,0,0,0,0,0,0,0,0,0}*/ ;

int main(void)
{
	
	//float PWM_A_filt[2] = {0,0};
	//float PWM_B_filt[2] = {0,0};
	float PWMduty =			0;	//PWM duty cycle
	//float PWMoffset =		0;	//difference between the two motors PWM for turning
	//float PWMgain =			MOTOR_PWM_LPF_GAIN;	
		
				
	accRawSTRUCT	accRaw		=	{0,0,0};
	accAngleSTRUCT	accAngle	=	{0,0};
	gyroRawSTRUCT	gyroRaw		=	{0,0,0};
	eulerSTRUCT		eulerAngVel	=	{0,0,0};
	compSTRUCT		compAngle	=	{0,0,0,COMP_ANGLE_GAIN};

	//If the robot tilt PID's derivative tag (robotTiltPID.D) calculated from the input(the measured angle) then we get rid of the derivative spike
	//We can also use the FROM_EXTERNAL_SOURCE macro, if the input is too noisy.
	//In that case derivative will calculated from the gyroscope's angular velocity which is the derivative angle	

	DDRB |= 1<<PB5;
	PORTB &= ~(1<<PB5);

	//for tuning pid parameters
	DDRD &= ~(1<<PD2) |~(1<<PD3) |~(1<<PD4) |~(1<<PD5);
		
	
	//------TIMER for main loop timing--------------
	//with prescale 8
	//At 8MHz 0.01s = 10 000 unit
	TCCR1A=0X00;
	TCCR1B |=1<<CS11;
	
	//--------------TIMER for PWM-------------------
	//PIN 0C0-OC2 inverted FAST PWM with prescale 8. This means a ~3.9kHz signal to the motor controller.
	TCCR2 |=1<<WGM21 | 1<<WGM20 | 1<<COM21 | 1<<CS21;
	TCCR0 |=1<<WGM01 | 1<<WGM00 | 1<<COM01 | 1<<CS01;
	OCR2 = 0; OCR0 = 0;
	
	pid_init(&tiltPID,KP,KI,KD);
	pid_set(&tiltPID,requestedAngle);
	
	USART_init();
	USART_Transmit_string("USART_init\r\n");

	sei();							//Enable global interrupts
		
	//----------------Init MPU6050------------------
 	i2c_init();
	USART_Transmit_string("i2c_init\r\n");
	 
	MPU6050_DDR |= 1<<MPU6050_AD0;
	MPU6050_PORT &= ~(1<<MPU6050_AD0);
	MPU6050_write(MPU6050_PWR_MGMT_1,0);
	MPU6050_write(MPU6050_GYRO_CONFIG,0b00001000);
	
	INIT_MotorController();
	
	_delay_ms(2000); //wait for the MPU6050 to turn on.
	accRaw.x = MPU6050_read_ACC(MPU6050_ACC_X)-MPU6050_ACC_X_AVG;
	accRaw.y = MPU6050_read_ACC(MPU6050_ACC_Y)-MPU6050_ACC_Y_AVG;
	accRaw.z = -MPU6050_read_ACC(MPU6050_ACC_Z)-MPU6050_ACC_Z_AVG;
	compAngle.roll	= atan2(accRaw.z,accRaw.y)*RAD_to_DEG;
	compAngle.pitch	= atan2(accRaw.x,hypotf(accRaw.y,accRaw.z))*RAD_to_DEG;
	
	//----------------Main loop---------------------//

    while(1)
    {
		if (TCNT1>=loopTIME)
		{			
			dt = (float)TCNT1/1000000;
			TCNT1 = 0;

			//Note that the accRaw.z's sign is changed
			accRaw.x = MPU6050_read_ACC(MPU6050_ACC_X)-MPU6050_ACC_X_AVG;
			accRaw.y = MPU6050_read_ACC(MPU6050_ACC_Y)-MPU6050_ACC_Y_AVG;
			accRaw.z = -MPU6050_read_ACC(MPU6050_ACC_Z)-MPU6050_ACC_Z_AVG;

			//I needed to swap gyroRaw.y and gyroRaw.z because of the gyro placement
			//with this modification I was able to use the usual Euler angle frame setup in the calcEulerAngles() like in this document:
			//http://www.chrobotics.com/library/understanding-euler-angles
			//I also change the sign of the gyroRaw.z and the accRaw.z to match with the example in the document
			gyroRaw.x = (MPU6050_read_GYRO(MPU6050_GYRO_ROLL)-MPU6050_GYRO_ROLL_AVG)/MPU6050_ScaleFactor*dt;
			gyroRaw.y = (MPU6050_read_GYRO(MPU6050_GYRO_PITCH)-MPU6050_GYRO_PITCH_AVG)/MPU6050_ScaleFactor*dt;
			gyroRaw.z = -(MPU6050_read_GYRO(MPU6050_GYRO_YAW)-MPU6050_GYRO_YAW_AVG)/MPU6050_ScaleFactor*dt;
			
			
			
			accAngle.roll	= atan2(accRaw.z,accRaw.y)*RAD_to_DEG;
			accAngle.pitch	= atan2(accRaw.x,hypotf(accRaw.y,accRaw.z))*RAD_to_DEG;	//-ACC_PITCH_OFFSET
			
			eulerAngVel		= calcEulerAngles(&gyroRaw,&compAngle);
			compAngle.pitch = compFilter(eulerAngVel.derivatedPitch,accAngle.pitch,compAngle.pitch,compAngle.gain);
			compAngle.roll	= compFilter(eulerAngVel.derivatedRoll,accAngle.roll,compAngle.roll,compAngle.gain);
			angle=compAngle.roll ;
		
			//--------------Start balancing--------------
			/*if(loopCount==10)
			{
				USART_Transmit_string("compAngle.roll:"); USART_Transmit_float(compAngle.roll);
				USART_Transmit_string("  |  PWMduty:");		USART_Transmit_float(PWMduty);
				USART_Transmit_string("  |  KP:");			USART_Transmit_float(tiltPID.kp);
				USART_Transmit_string("  |  KD:");			USART_Transmit_float(tiltPID.kd);
				USART_Transmit_string("  |  KI:");			USART_Transmit_float(tiltPID.ki);	
				//USART_Transmit_string("  |  LPF_GAIN:");		USART_Transmit_float(MOTOR_PWM_LPF_GAIN);
				USART_Transmit_string("  |  Error");		USART_Transmit_float(tiltPID.sp - compAngle.roll);
				USART_Transmit_string("\r\n");
				loopCount =0;
			}*/
			//PID functions and motor controller. The actual balancing.
			PWMduty	= pid_calculate(&tiltPID,compAngle.roll,dt);
			
			//driveMotor(PWMduty,PWMduty);
			/*LPfilter((PWMduty+PWMoffset),PWM_A_filt,MOTOR_PWM_LPF_GAIN);
			LPfilter((PWMduty-PWMoffset),PWM_B_filt,MOTOR_PWM_LPF_GAIN);
			driveMotor(PWM_A_filt[0],PWM_B_filt[0]);*/
			driveMotor(PWMduty,PWMduty);
	
			/*********************************tuning**********************************/
			/*     if (PIND & (1<<PD2)){ while(PIND & (1<<PD2)); KP +=0.1 ; }
			else if (PIND & (1<<PD3)){ while(PIND & (1<<PD3)); KP -=0.1 ; }
			else if (PIND & (1<<PD4)){ while(PIND & (1<<PD4)); KD +=0.01 ; }
			else if (PIND & (1<<PD5)){ while(PIND & (1<<PD5)); KD -=0.01 ; }
			else if (PIND & (1<<PD6)){ while(PIND & (1<<PD6)); KI +=0.001 ; }
			//else if (PIND & (1<<PD6)){ while(PIND & (1<<PD6)); MOTOR_PWM_LPF_GAIN +=0.1 ;}
			//else if (PINB & (1<<PB0)){ while(PINB & (1<<PB0)); MOTOR_PWM_LPF_GAIN -=0.1 ;}
			//else if (PINB & (1<<PB1)){ while(PINB & (1<<PB1)); KI +=0.0001 ; }
			//else if (PINB & (1<<PB2)){ while(PINB & (1<<PB2)); KI -=0.01 ; }*/
			
			pid_update(&tiltPID, KP , KI,  KD,requestedAngle) ;
			                                                
			/*********************************End of tuning********************************/
			
 		loopCount++ ;
		}
	}
}

ISR(USART_RXC_vect)
{
	UarTemp=UDR ;
	//USART_Transmit(UarTemp);
	//USART_Transmit_string("\r\n");
	if (UarTemp=='0')
	{
		KP +=0.1 ;
		print_PID_values(&tiltPID);

	}
	else if (UarTemp=='1')
	{
		KP -=0.1 ;
		print_PID_values(&tiltPID);
		
	}
	else if (UarTemp=='2')
	{
		KD +=0.001 ;
		print_PID_values(&tiltPID);
		
	}
	else if (UarTemp=='3')
	{
		KD -=0.001 ;
		print_PID_values(&tiltPID);
		
	}
	else if (UarTemp=='4')
	{
		KI +=0.001 ;
		print_PID_values(&tiltPID);
	}
	else if (UarTemp=='5')
	{
		KI -=0.001 ;
		print_PID_values(&tiltPID);
	}
	else if (UarTemp=='6')
	{
		USART_Transmit_string("compAngle.roll:"); USART_Transmit_float(angle);
		USART_Transmit_string("\r\n");
	}
	else if(UarTemp=='7')
	{
		MOTOR_PWM_LPF_GAIN +=0.01 ;
		USART_Transmit_string("PWM gain"); USART_Transmit_float(MOTOR_PWM_LPF_GAIN);
		USART_Transmit_string("\r\n");
	}
	else if(UarTemp=='8')
	{
		MOTOR_PWM_LPF_GAIN -=0.01 ;
		USART_Transmit_string("PWM gain"); USART_Transmit_float(MOTOR_PWM_LPF_GAIN);
		USART_Transmit_string("\r\n");
	}
	else if (UarTemp=='a')
	{
		requestedAngle +=1 ;
		USART_Transmit_string("angle"); USART_Transmit_float(requestedAngle);
		USART_Transmit_string("\r\n");
	}
	else if (UarTemp=='b')
	{
		requestedAngle -=1 ;
		USART_Transmit_string("angle"); USART_Transmit_float(requestedAngle);
		USART_Transmit_string("\r\n");
	}
	
	else if (UarTemp=='f')
	{
		requestedAngle =-110.00;
	}
	else if(UarTemp=='s')
	{
		requestedAngle =-89.00;
	}
	pid_update(&tiltPID,KP ,KI, KD,requestedAngle);
}
