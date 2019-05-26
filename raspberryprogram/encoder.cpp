#include <stdio.h>
#include <wiringPi.h>
#include <iostream>
#include <unistd.h>

//Pinout
//Encoderiai
uint8_t AInput = 2; //Deseines pinais 1
uint8_t BInput = 0; //Desines 2
uint8_t CInput = 7;//kaires 1
uint8_t DInput = 6;//kaires 2 
//

uint  RlastState = 0;    
uint  LlastState = 0;
uint8_t AState = 0;
uint8_t BState = 0;
uint8_t CState = 0;
uint8_t DState = 0;
uint8_t RState = 0;
uint8_t LState = 0;

uint 	Rsteps = 0;
uint 	Lsteps = 0;

uint GetStepsRight (){ //EncoderReading::
	  pullUpDnControl (AInput,PUD_UP);
		 pullUpDnControl (BInput,PUD_UP);
		  
		AState = digitalRead(AInput);
		BState = digitalRead(BInput) <<1;
		RState = AState | BState;

		

		if (RlastState != RState){
		  switch (RState) {
		    case 0:
		      if (RlastState == 2){
		        Rsteps++;
		      }
		      else if(RlastState == 1){
		        Rsteps--;
		      }
		      break;
		    case 1:
		      if (RlastState == 0){
		        Rsteps++;
		      }
		      else if(RlastState == 3){
		        Rsteps--;
		      }
		      break;
		    case 2:
		      if (RlastState == 3){
		        Rsteps++;
		      }
		      else if(RlastState == 0){
		        Rsteps--;
		      }
		      break;
		    case 3:
		      if (RlastState == 1){
		        Rsteps++;
		      }
		      else if(RlastState == 2){
		        Rsteps--;
		      }
		      break;
		  }
		}

		
	    usleep (1000);

	    RlastState = RState;
	    

	  
  	return Rsteps;
}

uint GetStepsLeft(){
	pullUpDnControl (CInput,PUD_UP);
		 pullUpDnControl (DInput,PUD_UP);
		CState = digitalRead(CInput);
		DState = digitalRead(DInput) << 1;
		LState = CState | DState;
  
  if (LlastState != LState){      
		  switch (LState) {
		    case 0:
		      if (LlastState == 2){
		        Lsteps++;
		      }
		      else if(LlastState == 1){
		        Lsteps--;
		      }
		      break;
		    case 1:
		      if (LlastState == 0){
		        Lsteps++;
		      }
		      else if(LlastState == 3){
		        Lsteps--;
		      }
		      break;
		    case 2:
		      if (LlastState == 3){
		        Lsteps++;
		      }
		      else if(LlastState == 0){
		        Lsteps--;
		      }
		      break;
		    case 3:
		      if (LlastState == 1){
		        Lsteps++;
		      }
		      else if(LlastState == 2){
		        Lsteps--;
		      }
		      break;
		  }
		}
			usleep (1000);
		LlastState = LState;
	
		return Lsteps;
}
