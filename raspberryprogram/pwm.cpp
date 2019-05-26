#include <stdio.h>
#include <wiringPi.h>

#include <stdio.h>
#include <wiringPi.h>

#define PWMtresholding
#define PWMtresholdingValue	3 // +- x (%)

const int pwmpinA = 1; 	// A
const int pwmpinB = 23;	// B

const int pwmB = 24;// Signalo B generatorius
const int pwmA = 26;// Signalo A generatorius

int lastModeA=20;
int lastModeB=20;

int lastSpeedA = 0;
int lastSpeedB = 0; 
void movingMotor(int SpeedA, int SpeedB, int modeA, int modeB) // judinam motor paduodami srove skirtingais pinais.
{
	if(lastModeA!=modeA)
		{
		switch(modeA)
			{
			  case 0:
			
				pinMode(pwmpinA,INPUT );
				pinMode(pwmA,PWM_OUTPUT );
				pwmSetMode(PWM_MODE_MS); 
				pwmSetClock(960); // Ciklo daznis = (19.2/Divisor)/Range
				pwmSetRange(1024);
				lastModeA=0;
				break;	
			
			  case 1:
				pinMode(pwmpinA,PWM_OUTPUT );
				pinMode(pwmA, INPUT);
				pwmSetMode(PWM_MODE_MS); 
				pwmSetClock(960); // Ciklo daznis = (19.2/Divisor)/Range
				pwmSetRange(1024);
				
				lastModeA=1;
				break;

			  case 2:
				pinMode(pwmpinA, INPUT);
				pinMode(pwmA, INPUT);
				pwmSetMode(PWM_MODE_MS); 
				pwmSetClock(960); // Ciklo daznis = (19.2/Divisor)/Range
				pwmSetRange(1024);
				
				lastModeA=2;
				break;
			}
		}

	
	if(lastModeB!=modeB)
	{
		switch(modeB)
		{
		  case 0:

			pinMode(pwmB,PWM_OUTPUT);
			pinMode(pwmpinB,INPUT );
			pwmSetMode(PWM_MODE_MS); 
			pwmSetClock(960); // Ciklo daznis = (19.2/Divisor)/Range
			pwmSetRange(1024);
			
			lastModeB=0;
			break;	
		
		  case 1:
			pinMode(pwmB,INPUT );
			pinMode(pwmpinB,PWM_OUTPUT );
			pwmSetMode(PWM_MODE_MS); 
			pwmSetClock(960); // Ciklo daznis = (19.2/Divisor)/Range
			pwmSetRange(1024);
			
			lastModeB=1;
			break;

		  case 2:
			pinMode(pwmpinB, INPUT);
			pinMode(pwmB, INPUT);
			pwmSetMode(PWM_MODE_MS); 
			pwmSetClock(960); // Ciklo daznis = (19.2/Divisor)/Range
			pwmSetRange(1024);
			
			lastModeB=2;
			break;
		}
	}
	
	
	#ifdef PWMtresholding
		if(SpeedA>lastSpeedA+PWMtresholdingValue | SpeedA<lastSpeedA-PWMtresholdingValue)
			{
				pwmWrite(pwmA,SpeedA);
				lastSpeedA=SpeedA;
			}
		if(SpeedB>lastSpeedB+PWMtresholdingValue | SpeedB<lastSpeedB-PWMtresholdingValue)
			{
				pwmWrite(pwmB,SpeedB);
				lastSpeedB=SpeedB;
			}
	#else
		pwmWrite(pwmA,SpeedA);
		pwmWrite(pwmB,SpeedB);
	#endif
}


