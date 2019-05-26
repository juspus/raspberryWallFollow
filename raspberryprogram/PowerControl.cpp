#include <stdio.h>   
#include <wiringPi.h> 
#include <stdlib.h>

//WIRING PI PINS
const int ledPin = 23; // need to be changed !
const int butPin = 24; // need to be changed !
const int longPressMs = 900;
void blinkLed(int onTime, int offTime);

int main(void){
	wiringPiSetupGpio(); 
    pinMode(ledPin, OUTPUT);     
    pinMode(butPin, INPUT);     
    pullUpDnControl(butPin, PUD_UP); 
	printf("Power button/led is working.\n");
	
	bool shortPress =false; 
	bool longPress= false;
	bool online = true;

	while(1)
		{
			if(online)
			{
				blinkLed(100,400);
				
				if(!(digitalRead(butPin)))
					{
						shortPress=true;
						delay(longPressMs);
						if(!(digitalRead(butPin)))
							{
								longPress=true;
							}
					}
				
				if(shortPress)
				{
					if(!(longPress))
					{
						//Short press, shutdown
						printf("Bye bye\n");
						online = false;
						system("shutdown now");
					}
					else
					{
						//Long press, reset
						printf("See you soon\n");
						online = false;
						system("reboot");
					}
				}
			}
			else
			{
				// go to offline;
				delay(100);
			}		
	}
    return 0;
}

void blinkLed(int onTime, int offTime)
{
	digitalWrite(ledPin, HIGH);
	delay(onTime);
	digitalWrite(ledPin, LOW);
	delay(offTime);
}


// !!!! compilint naudojant    "  -l wiringPi  "

