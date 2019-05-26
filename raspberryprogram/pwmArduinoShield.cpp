#include <stdio.h>
#include <wiringPi.h>

#define STOP 1
#define GO 0
#define FORWARD 1
#define BACKWARD 0

const int pwmpinA = 1;
const int dirA = 26;
const int breakA =27; 
const int pwmpinB = 23;
const int dirB = 24;
const int breakB = 25


void breaker(int breakingA, int breakingB)
{
	//------BREAK--A---------
	digitalWrite(breakA, breakingA);
	//-----------------------

	//------BREAK--B---------
	digitalWrite(breakB, breakingB);
	//-----------------------
}

void movingMotor(int directionA, int directionB, int speedA, int speedB)
{
	//------Forward--A-------
	digitalWrite(breakA, 0);
	digitalWrite(dirA, directionA);		
	pwmWrite(pwmpinA,speedA);
	//-----------------------

	//------Forward--B-------
	digitalWrite(breakB, 0);
	digitalWrite(dirB, directionB);		
	pwmWrite(pwmpinB,speedB);
	//-----------------------
}

int main(void){
	wiringPiSetup();
	pinMode(pwmpinA,PWM_OUTPUT);
	pinMode(pwmpinB,PWM_OUTPUT);
	pinMode(dirA, OUTPUT);
	pinMode(dirB, OUTPUT);
	pinMode(breakA, OUTPUT);
	pinMode(breakB, OUTPUT);
	//pwmSetMode(PWM_MODE_MS); - normaliai naudojamas Balanced rezimas, kuris isskirsto mazesniais impulsais, bet 2x dazniau.

	pwmSetClock(1920); // Ciklo daznis = (19.2/Divisor)/Range
	pwmSetRange(100);
	
	while(true)
	{
		movingMotor(FORWARD, FORWARD, 50, 50); // movinMotor(dirA, dirB, speedA, speedB)
		
		delayMicroseconds(1000000);

		breaker(STOP, STOP); // breaker(motorA, motorB);

		delayMicroseconds(1000000);	

		movingMotor(BACKWARD, BACKWARD, 50, 50);

		delayMicroseconds(1000000);

		breaker(STOP, STOP);

		delayMicroseconds(1000000);	
	}
}