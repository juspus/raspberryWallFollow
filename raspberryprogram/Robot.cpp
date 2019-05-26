#include "VL53L0X.hpp"
#include "Global.h"
#include <linux/i2c-dev.h>
#include <chrono>
#include <csignal>
#include <exception>
#include <iomanip>
#include <iostream>
#include <unistd.h> //sleeping
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <sys/types.h>
#include <unistd.h>
#include <wiringPi.h>
#include <cstdlib>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <chrono>
#include <inttypes.h>
#include <math.h>


#define greicioUzduotis 350 // saugus greitis 300, pilnai pakrovus baterijas ant mano robotu, visu robotai skirsis. 

#define minGreitis 0.1

#define stepLengthmm 7.8






//VL53L0X
pthread_t laserThread;
//MPU6050
pthread_t mpuThread;
//Encoder
pthread_t encoderThread;
//PWM
pthread_t pwmThread;
//BlueTooth
pthread_t bluetoothThread;
//SienosSekimas
pthread_t sienosSekimas;
//Greicio skaiciavimas
pthread_t velocityCalculation;
//Kelio radimas
pthread_t kelioRadimas;
//Spausdinimas
pthread_t spausdinimas;


char mode;

int driveMode;
// SIGINT (CTRL-C) exit flag and signal handler
volatile sig_atomic_t exitFlag = 0;
void sigintHandler(int)
{
	exitFlag = 1;
}

void *EncoderMeasurement(void *arg)
{ //Encoder
	std::cerr << " Starting Encoder thread  " << std::endl;
	signal(SIGINT, sigintHandler);

	for (; !exitFlag;)
	{
		rightSteps = GetStepsRight();
		leftSteps = GetStepsLeft();
	}
}

void *LaserMeasurement(void *arg)
{
	std::cerr << " Starting VL530X Thread " << std::endl;
	//pthread_mutex_lock(&mutex1);

	// Configuration constants
	// Number of sensors. If changed, make sure to adjust pins and addresses accordingly (ie to match size).
	const int SENSOR_COUNT = 6;
	// GPIO pins to use for sensors' XSHUT. As exported by WiringPi
	const uint8_t pins[SENSOR_COUNT] = {5, 22, 20, 6, 21, 26}; //BCM Pin's! //Tito Raspberry: 5,22,20,6,21,26// Legit raspberry: 22,5,6,21,20,26
	// Sensors' addresses that will be set and used. These have to be unique.
	const uint8_t addresses[SENSOR_COUNT] = {

		0x15,
		0x20,
		0x25,
		0x30,
		0x35,
		0x40

	};

	// Register SIGINT handler
	signal(SIGINT, sigintHandler);

	// Create sensor objects' array
	VL53L0X *sensors[SENSOR_COUNT];

	// Create sensors (and ensure GPIO pin mode)
	for (int i = 0; !exitFlag && i < SENSOR_COUNT; ++i)
	{
		sensors[i] = new VL53L0X(pins[i]);
		sensors[i]->powerOff();
	}
	// Just a check for an early CTRL-C
	if (exitFlag)
	{
		//return 0;
	}

	// For each sensor: create object, init the sensor (ensures power on), set timeout and address
	// Note: don't power off - it will reset the address to default!
	for (int i = 0; !exitFlag && i < SENSOR_COUNT; ++i)
	{
		try
		{
			// Initialize...
			sensors[i]->initialize();
			printf("try initialize\n");
			// ...set measurement timeout...
			sensors[i]->setTimeout(200);
			// ...set the lowest possible timing budget (high speed mode)...
			
			sensors[i]->setMeasurementTimingBudget(20000);
			// ...and set I2C address...
			;
			sensors[i]->setAddress(addresses[i]);
			// ...also, notify user.
			std::cout << "Sensor " << i << " initialized, real time budget: " << sensors[i]->getMeasurementTimingBudget() << std::endl;
		}
		catch (const std::exception &error)
		{
			std::cerr << "Error initializing sensor " << i << " with reason:" << std::endl
					  << error.what() << std::endl;
			//return 1;
			exitFlag=true;
		}
	}

	// Start continuous back-to-back measurement
	for (int i = 0; !exitFlag && i < SENSOR_COUNT; ++i)
	{
		try
		{
			sensors[i]->startContinuous();
		}
		catch (const std::exception &error)
		{
			std::cerr << "Error starting continuous read mode for sensor " << i << " with reason:" << std::endl
					  << error.what() << std::endl;
			//return 2;
			exitFlag	=true;
		}
	}

	// Durations in nanoseconds
	uint64_t totalDuration = 0;
	uint64_t maxDuration = 0;
	uint64_t minDuration = 1000 * 1000 * 1000;
	// Initialize reference time measurement
	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

	// We need that variable after the for loop
	int j = 0;
	// Also, set fill and width options for cout so that measurements are aligned
	std::cout << std::setw(4) << std::setfill('0');

	// Take the measurements!
	for (; !exitFlag; ++j)
	{
		//	std::cout << ">"; //no of times read
		for (int i = 0; !exitFlag && i < SENSOR_COUNT; ++i)
		{
			uint16_t distance;
			try
			{
				// Read the range. Note that it's a blocking call
				distance = sensors[i]->readRangeContinuousMillimeters();
				lasersData[i] = distance;
				usleep(2000);
			}
			catch (const std::exception &error)
			{
				std::cerr << std::endl
						  << "Error geating measurement from sensor " << i << " with reason:" << std::endl
						  << error.what() << std::endl;
				// You may want to bail out here, depending on your application - error means issues on I2C bus read/write.
				//return 3;
				distance = 8096;
			}

			// Check for timeout
			if (sensors[i]->timeoutOccurred())
			{
				//	std::cout << "tout | ";
			}
			else
			{

				// Display the reading
			//		std::cout << distance << ",";
				usleep(10000);
			}
		}
		//std::cout << std::endl << std::flush;

		// Calculate duration of current iteration
		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
		uint64_t duration = (std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1)).count();
		// Save current time as reference for next iteration
		t1 = t2;
		// Add total measurements duration
		totalDuration += duration;
		// Skip comparing first measurement against max and min as it's not a full iteration
		if (j == 0)
		{
			continue;
		}
		// Check and save max and min iteration duration
		if (duration > maxDuration)
		{
			maxDuration = duration;
		}
		if (duration < minDuration)
		{
			minDuration = duration;
		}
		usleep(500);
	}

	// Print measurement duration statistics
	std::cout << "\nMax duration: " << maxDuration << "ns" << std::endl;
	std::cout << "Min duration: " << minDuration << "ns" << std::endl;
	std::cout << "Avg duration: " << totalDuration / (j + 1) << "ns" << std::endl;
	std::cout << "Avg frequency: " << 1000000000 / (totalDuration / (j + 1)) << "Hz" << std::endl;

	// Clean-up: delete objects, set GPIO/XSHUT pins to low.
	for (int i = 0; i < SENSOR_COUNT; ++i)
	{
		sensors[i]->stopContinuous();
		delete sensors[i];
	}
}
void *MPU_Measurement(void *arg)
{
	std::cerr << " Starting MPU thread  " << std::endl;
	I2C_Init();
	MPU6050_Init();
	signal(SIGINT, sigintHandler);
	//printf("testing MPU connection: \n");

	//printf(TestConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
	uint64_t totalMpuDuration = 0;
	std::chrono::steady_clock::time_point mput1 = std::chrono::steady_clock::now();
	int m = 0;

	for (; !exitFlag; ++m)
	{
		//no of times read

		aX = read_raw_data(ACCEL_XOUT_H);
		aY = read_raw_data(ACCEL_YOUT_H);
		aZ = read_raw_data(ACCEL_ZOUT_H);
		gX = read_raw_data(GYRO_XOUT_H);
		gY = read_raw_data(GYRO_YOUT_H);
		gZ = read_raw_data(GYRO_ZOUT_H);
		usleep(10);

		// printf("\nGx=%d Gy=%d Gz=%d Ax=%dg g Ay=%dg Az=%dg\n",gX,gY,gZ,aX,aY,aZ);
		accelX = (float)aX / 16384.0;
		accelY = (float)aY / 16384.0;
		accelZ = (float)aZ / 16384.0;
		gyroX = (float)gX / 131;
		gyroY = (float)gY / 131;
		gyroZ = (float)gZ / 131;

		// printf("\nGx=%.3f Gy=%.3f Gz=%.3f Ax=%.3fg Ay=%.3fg Az=%.3fg\n",gyroX,gyroY,gyroZ,accelX,accelY,accelZ);
		//printf("\nAx=%.3fg Ay=%.3fg Az=%.3fg Gx=%.3f Gy=%.3f Gz=%.3f \n",accelX,accelY,accelZ,gyroX,gyroY,gyroZ);
		usleep(500000);
		std::chrono::steady_clock::time_point mput2 = std::chrono::steady_clock::now();
		uint64_t mpuDuration = (std::chrono::duration_cast<std::chrono::nanoseconds>(mput2 - mput1)).count();
		// Save current time as reference for next iteration
		mput1 = mput2;
		// Add total measurements duration
		totalMpuDuration += mpuDuration;
	}
	std::cout << "Avg MPU frequency: " << 1000000000 / (totalMpuDuration / (m + 1)) << "Hz" << std::endl;
}

void *PwmControl(void *arg)
{
		usleep(2000020);
	//usleep(3000000);
	std::cerr << " Starting pwmControl thread  " << std::endl;

	// std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

	pwmLeft=22;
	pwmRight=9;
	//movingMotor(pwmLeft, pwmRight, FORW, BACKW);
	//usleep(1000000);
	signal(SIGINT, sigintHandler);

	//while (rightSteps <= 118 && leftSteps <= 118 && rightSteps >= -118 && leftSteps >= -118)
	//{
		





		

	//	usleep(100000);
	//}

	//movingMotor(50, 50, STOP, STOP);
	//exitFlag = true;

	for (; !exitFlag;)
	{

		double skirtumasL, skirtumasR, kof,kof2;
		kof=0.05;
		kof2=0.05;
		skirtumasR=0;
		skirtumasL=0;
		if(VelocityRight>1000)
		{
			VelocityRight=0;
		}
		if(VelocityLeft>1000)
		{
			VelocityLeft=0;
		}
		if(VelocityRight<rightSpeed){
			skirtumasR=(rightSpeed-VelocityRight)*kof;
		}	
		else{
			skirtumasR=0;
		}
		if(VelocityLeft<leftSpeed){
			skirtumasL=(leftSpeed-VelocityLeft)*kof;
		}
		else {
			skirtumasL=0;
		}
		if(VelocityRight>rightSpeed){
			skirtumasR=(rightSpeed-VelocityRight)*kof2;
		}	
		else{
			skirtumasR=0;
		}
		if(VelocityLeft>leftSpeed){
			skirtumasL=(leftSpeed-VelocityLeft)*kof2;
		}
		else {
			skirtumasL=0;
		}
		galutinisGreitisL=leftSpeed+skirtumasL;
		//galutinisGreitisL=0;
		galutinisGreitisR=rightSpeed+skirtumasR;
	//galutinisGreitisR=0;
		if(leftSpeed<minGreitis&&rightSpeed>=minGreitis)
		{	
			
			printf("pwm sukame kaire greiciu: %f\n",galutinisGreitisL);
			/*
			pwmRight=63.956*exp(0.0027*galutinisGreitisR);
			pwmLeft=pwmRight;
			printf("pwmRight: %d pwmLeft: %d\n",pwmRight,pwmLeft);
			movingMotor(pwmLeft, pwmRight, BACKW, FORW);
			*/
			pwmLeft=rightSpeed*0.5;
			pwmRight=rightSpeed*0.5;
			printf("pwmRight: %d pwmLeft: %d\n",pwmRight,pwmLeft);
			movingMotor(pwmLeft,pwmRight, BACKW, FORW);
			//pwmRight = 3.5048 * exp(0.0372 * galutinisGreitisR / 10);
			//pmRight = 3.5048 * exp(0.08 * galutinisGreitisR / 10);
			//pwmRight+=3;
		}
		else if(rightSpeed<minGreitis&& leftSpeed>=minGreitis)
		{
			printf("pwm sukame i desine greiciu: %f\n",galutinisGreitisR);
			
			//pwmLeft = 18.472 * exp(0.0122 * galutinisGreitisL / 10);
			//pwmLeft = 18.472 * exp(0.03 * galutinisGreitisL / 10);
			/*pwmLeft=63.956*exp(0.0027*galutinisGreitisL);
			pwmRight=pwmLeft;
			*/
			pwmLeft=leftSpeed*0.5;
			pwmRight=leftSpeed*0.5;
			printf("pwmRight: %d pwmLeft: %d\n",pwmRight,pwmLeft);
			movingMotor(pwmLeft, pwmRight, FORW, BACKW);
		}
		else if(rightSpeed<minGreitis&&leftSpeed<minGreitis)
		{
			pwmRight=0;
			pwmLeft=0;
			movingMotor(pwmLeft,pwmRight, FORW, FORW);
		}
		
		
		
		else{
			//pwmLeft = 18.472 * exp(0.0122 * galutinisGreitisL / 10);
			//p//wmLeft = 18.472 * exp(0.03 * galutinisGreitisL / 10);
		//	pwmRight = 3.5048 * exp(0.0372 * galutinisGreitisR / 10);
			//pwmRight = 3.5048 * exp(0.08* galutinisGreitisR / 10);
			//pwmRight=pwmRight+3;
			//pwmLeft=63.956*exp(0.0027*galutinisGreitisL);
			//pwmRight=63.956*exp(0.0027*galutinisGreitisR);
			
			pwmLeft=0.5*galutinisGreitisL;
			pwmRight=0.5*galutinisGreitisR;
			movingMotor(pwmLeft,pwmRight, FORW, FORW);
			//pwmLeft-=6;
		}
		
			
		
			//movingMotor(300, 300, FORW, FORW); // Ratu test
		//movingMotor(pwmLeft, pwmRight, FORW, FORW);

		usleep(50000);
	}

	// std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
	// uint64_t duration = (std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1)).count();
	// printf("Time:  %lf  \n", (float)duration/1000000000);//PRId64

	movingMotor(0, 0, STOP, STOP);
}

void *BluetoothConnection(void *arg)
{
	std::cerr << " Starting bluetooth thread  " << std::endl;
	signal(SIGINT, sigintHandler);

	// allocate socket
	s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

	// bind socket to port 1 of the first available
	// local bluetooth adapter
	loc_addr.rc_family = AF_BLUETOOTH;

	bdaddr_t my_bdaddr_any = {0, 0, 0, 0, 0, 0};
	// bacmp(bdaddr, my_bdaddr_any)

	loc_addr.rc_bdaddr = my_bdaddr_any;
	loc_addr.rc_channel = (uint8_t)1;
	bind(s, (struct sockaddr *)&loc_addr, sizeof(loc_addr));

	// put socket into listening mode
	listen(s, 1);
	printf("Put into listening mode\n");

	// accept one connection
	client = accept(s, (struct sockaddr *)&rem_addr, &opt);
	ba2str(&rem_addr.rc_bdaddr, buf);
	fprintf(stderr, "Accepted connection from %s\n", buf);
	memset(buf, 0, sizeof(buf));

	// send a message
	long long milliseconds;
	long long previous_500ms = 0;
	int checkStatus = 0;

	while (!exitFlag)
	{
		usleep(10000);

		milliseconds = current_timestamp();
		if (milliseconds - previous_500ms > 100L)
		{

			previous_500ms = milliseconds;

			// checkStatus= SendData(lasersData[0],lasersData[1],lasersData[2],lasersData[3],lasersData[4],lasersData[5],gX,gY,gZ,aX,aY,aZ,leftSteps,rightSteps,0,0, client);
			checkStatus = SendData(lasersData[0], lasersData[1], lasersData[2], lasersData[3], lasersData[4], lasersData[5], gX, gY, gZ, aX, aY, aZ, leftSteps, rightSteps, xPrad, yPrad, client);
			if (checkStatus < 0)
				break;
		}
	}

	// close connection
	close(client);
	close(s);
}

void *SienosSekimas(void *arg)
{
	usleep(1500000);
	std::cerr << " Starting SienosSekimas thread  " << std::endl;
	float *koord;
	signal(SIGINT, sigintHandler);

	while (!exitFlag)
	{
		l1 = lasersData[0];
		l2 = lasersData[1];
		l3 = lasersData[2];
		l4 = lasersData[3];
		l5 = lasersData[4];
		l6 = lasersData[5];

		usleep(100000);
		koord = gautiKoordinates(l1, l2, l3, l4, l5, l6); //l1,l2,l3,l4,l5,l6
		xPrad = koord[0];
		yPrad = koord[1];
		vKampas = koord[2];
		slopeG=koord[3];
		interceptG=koord[4];
		
		// printf("x: %f y: %f\n",x,y);
	
	}
}

void *VelocityCalculation(void *arg)
{
	usleep(500000);
	std::cerr << " Starting Velocity thread  " << std::endl;
	//double velocityLeft;
	//double velocityRight;
	double avarageLeft = 0;
	double avarageRight = 0;
	double rightStepsNow=0;
	double leftStepsNow=0;
	
	int counter = 0;
	int leftStepsBeginCount, rightStepsBeginCount, rightStepsEndCount, leftStepsEndCount;
	signal(SIGINT, sigintHandler);
	auto startLeft = std::chrono::high_resolution_clock::now();
	auto startRight = std::chrono::high_resolution_clock::now();
	auto stopLeft=std::chrono::high_resolution_clock::now();;
	auto stopRight=std::chrono::high_resolution_clock::now();;

	while (!exitFlag)
	{
			
		    if(leftSteps>=leftStepsNow+2)
			{	
				//printf("leftStepChange IF\n");
				stopLeft = std::chrono::high_resolution_clock::now(); 

				auto durationLeft = std::chrono::duration_cast<std::chrono::milliseconds>(stopLeft - startLeft); 
				//durationLeft.count();
				VelocityLeft=(leftSteps-leftStepsNow)*stepLengthmm; //mm/s
				VelocityLeft=VelocityLeft/(durationLeft.count())*1000;
				std::cout<<durationLeft.count()<<"  "<<"Kairys"<<leftSteps-leftStepsNow<<std::endl;

				leftStepsNow=leftSteps;
				startLeft = std::chrono::high_resolution_clock::now();

			}
			
		     if(rightSteps>=rightStepsNow+2)
			{
				//printf("RightStepChange IF\n");
				stopRight = std::chrono::high_resolution_clock::now(); 

				auto durationRight = std::chrono::duration_cast<std::chrono::milliseconds>(stopRight - startRight); 
				VelocityRight=((rightSteps-rightStepsNow)*stepLengthmm)/(durationRight.count())*1000; //mm/s
				std::cout<<durationRight.count()<<"  "<< "desinys"<<rightSteps-rightStepsNow<<std::endl;
				rightStepsNow=rightSteps;
				startRight = std::chrono::high_resolution_clock::now();

			}	

			auto startEncoder = std::chrono::high_resolution_clock::now();
			auto resetDurationL = std::chrono::duration_cast<std::chrono::milliseconds>(startEncoder - stopLeft);
			auto resetDurationR = std::chrono::duration_cast<std::chrono::milliseconds>(startEncoder - stopRight);
			if(leftSteps==leftStepsNow&& resetDurationL.count()>1000)
			{
				VelocityLeft=0;
			}

			if(rightSteps==rightStepsNow&& resetDurationR.count()>1000)
			{

				VelocityRight=0;
			}




			// leftStepsBeginCount = leftSteps;
			// rightStepsBeginCount = rightSteps;
			// usleep(100000); //100ms
			// leftStepsEndCount = leftSteps;
			// rightStepsEndCount = rightSteps;


				//sumaisyti encoderiai!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

			//VelocityRight = (((double)leftStepsEndCount - (double)leftStepsBeginCount) / 118) / 0.1 * 1000; //Daliname is pulsu per metra(118) ir praejusio laiko sekundemis (0.01) ir verciame  i mm/s
			//VelocityLeft = (((double)rightStepsEndCount - (double)rightStepsBeginCount) / 118) / 0.1 * 1000;
			//avarageLeft += velocityLeft;
			//avarageRight += velocityRight;
			
			//commonVelocityApprox=((velocityLeft)+(velocityRight))/2;
			
		

		//if (counter == 10)
		//{
			//commonVelocityApprox=((avarageLeft/counter)+(avarageRight/counter))/2;
			//printf("\n10 measurments in 100ms:  avarageVelocityLeft: %f avarageVelocityRight: %f\n Left Steps: %d Right Steps: %d", (float)avarageLeft / counter, (float)avarageRight / counter,leftSteps,rightSteps);
			//counter = 0;
			//avarageLeft = 0;
			//avarageRight = 0;
		//}

		//	printf("100ms velocityleft: %0.2lf  velocityright: %0.2lf encoderLeft: %d encoderRight: %d\n\n", velocityLeft, velocityRight, leftSteps, rightSteps);
	}
}
void *Spausdinimas(void *arg)
{
	float elapsedTime=0;
	while (!exitFlag)
	{
		printf("L1: %d L2: %d L3: %d L4: %d L5: %d L6: %d\n", lasersData[0],
		 lasersData[1],
		lasersData[2],
		 lasersData[3],
		 lasersData[4],
		lasersData[5]);
		
		printf("lSteps: %d rSteps: %d\n", rightSteps, leftSteps);
		printf("TG %f, \n",tgGlobal);
		//printf("CommonVelocity %f\n", commonVelocityApprox);
		printf("kampas: %f\n",kampasGlobal);
		printf("galutiniai greiciai: Left: %f Right: %f\n",galutinisGreitisL,galutinisGreitisR);
		printf("pmwLeft: %d pwmRight: %d\n",pwmLeft,pwmRight);
		printf("Vaziuoti i [%0.1f, %0.1f]\n",xPrad,yPrad);
		printf("slope: %f, intercept: %f\n",slopeG,interceptG);
		printf("Elapsed Time: %0.1f\n\n", elapsedTime);
		usleep(100000);
		elapsedTime+=0.1;

	}
}

void *KelioRadimas(void *arg)
{
		usleep(2000000);
		//galutiniaiGreiciai = skaiciuoti((double)xPrad, (double)yPrad, 150, 0, vxg,vyg); //kairys[0] desinys[1] mm/s //commonVelocityApprox
		

	float *galutiniaiGreiciai;
	std::cerr << " Starting kelio radimas thread  " << std::endl;
	double norimasGreitis;
								 // roboto greitis mm/s
								 // double xg = tiksloKoor[0]; //atsinest is sienos aproksimavimo
								 // double yg = tiksloKoor[1]; //atsinest is sienos aproksimavimo
								 // double vKampas = tiksloKoor[2]; //kampas is sienos aproksimavimo
								 // double vx0 = 150; //atsinesti is nuskaitytu enkoderiu
								 // double vy0 = 0;

								//skaiciuoti(tiksloKoor[0], yg, vx0, vy0, vxg, vyg);
	while (!exitFlag)
	{
		//double xg=0; //Norima koord kur atsidurt priekio atzvilgio 
		//	double yg=-500;
		double xg = xPrad;
		double yg=yPrad;
		if(driveMode==0)
		{
			norimasGreitis=greicioUzduotis;
		}
		else{
			norimasGreitis=greicioUzduotis-100;
		}
		double vx0=norimasGreitis;//dabartinis greitis pagal koord nekeiciam
		double vy0=0;
		//vxg=0; //galutine greicio uzduotis
		//vyg=400;
		double a = 70; //ilgis mm tarp rato ir centro roboto
		double x0 = 0; //pradine Koord
		double y0 = 0; //pradine Koord

		double vxg; //greicio vektorius galinis
		double vyg; //greicio vektoris galinis
		double kampas,cosKampas;
		int kampas_rib = 25;
		cosKampas=(xg)/(sqrt(xg*xg+yg*yg));
		kampas = acos(cosKampas)*180/3.14;
		if (yg < 0) { kampas = -1 * kampas; }
		kampasGlobal=kampas;
		
		vyg = norimasGreitis * cos(vKampas);
		vxg = norimasGreitis * sin(vKampas);
		if(vxg<0){
			vKampas=vKampas+180;
			vyg = norimasGreitis * cos(vKampas);
			vxg = norimasGreitis * sin(vKampas);
		}
		if (kampas > 0 && vKampas > (kampas + kampas_rib)) { vKampas = kampas + kampas_rib; }
		if (kampas > 0 && vKampas < (kampas - kampas_rib)) { vKampas = kampas - kampas_rib; }
		if (kampas < 0 && vKampas > (kampas + kampas_rib)) { vKampas = kampas + kampas_rib; }
		if (kampas < 0 && vKampas < (kampas + kampas_rib)) { vKampas = kampas - kampas_rib; }
		vyg = norimasGreitis * cos(vKampas)/2;
		vxg = norimasGreitis * sin(vKampas)/2;
		
	
	// ofstream outfiletpos;
	int counter=0;
	double b = 0.1;
	double tg;
	double  t;
	char key;
	
	printf("kampas: %f norimas[x:%0.1f,y:%0.1f] vxg: %0.1f vyg: %0.1f\n",kampas,xg,yg,vxg,vyg);
	
	
//	printf("kampas: %f\n",kampas);
	
	// darasyti wallAprox kode:" if(diskriminantas<0 buvo){ vKampas=0; tiksloKoor[0] = tiksloKoor[0]/2 ;tiksloKoor[1] = tiksloKoor[1]/2;}

	if(kampas>kampas_rib)
		{	
			if(driveMode==0)
			{
				leftSpeed=0;
				rightSpeed=norimasGreitis*1.5;
				usleep(300000);
				printf("sukame i kaire\n");
				leftSpeed=0;
				rightSpeed=0;
				//exitFlag=true;
			}
			else{
				leftSpeed=0;
				rightSpeed=norimasGreitis;
				usleep(400000);
				printf("sukame i kaire\n");
				
				leftSpeed=0;
				rightSpeed=0;
				printf("stop\n");
				key=getchar();
				
			}
			usleep(300000);
		}
	
		
	else if(kampas< kampas_rib*-1)
		{
			if(driveMode==0)
			{
				leftSpeed=norimasGreitis*1.5;
				rightSpeed=0;
				usleep(300000);
				printf("sukame i desine\n");
				leftSpeed=0;
				rightSpeed=0;
				//exitFlag=true;
			}
			else{
				leftSpeed=norimasGreitis;
				rightSpeed=0;
				usleep(400000);
				printf("sukame i desine\n");
				//galutinisGreitisL=0;
				//galutinisGreitisR=0;
				leftSpeed=0;
				rightSpeed=0;
				printf("stop\n");
				key=getchar();
				
			}
			usleep(300000);
		}
	
	
	
	else
	{ 
		printf("vaziavimas spline\n");
		if(driveMode==0)
		{
		tg = (3 * (-vx0*xg - vxg*xg - vy0*yg - vyg*yg + sqrt(pow(((vx0 + vxg)*(-xg) + (vy0 + vyg)*(-yg)), 2) + 4 * (11 * pow(vx0, 2) + 2 * vx0*vxg + 11 * pow(vxg, 2) + 11 * pow(vy0, 2) + 2 * vy0*vyg + 11 * pow(vyg, 2))*(pow(xg, 2) + pow(yg, 2))))) / (11 * vx0*vx0 + 2 * vx0*vxg + 11 * vxg*vxg + 11 * vy0*vy0 + 2 * vy0*vyg + 11 * vyg*vyg);
		tgGlobal=tg;
		//printf("tg: %f tg/2: %f dabartinisGreitis: %f norimas[x:%0.1f,y:%0.1f]\n",tg,tg/2,vx0,xg,yg);
		//counter=0
		int FirstRun=0;
		
			
		
		for (t = 0; t<=tg*3/5; t += b)
		{
		//outfiletpos.open("variables.txt",ios::app);
	
		double x, y;
		//randame x pozicijos funkcija nuo laiko
		x = (vx0 + (t*(t*(tg*(vx0 + vxg) + 2 * (-xg)) - tg*(2 * tg*vx0 + tg*vxg - 3 * xg))) / pow(tg, 3))*t;
		//randame y pozicijos funkcija nuo laiko
		y = (vy0 + (t*(t*(tg*(vy0 + vyg) + 2 * (-yg)) - tg*(2 * tg*vy0 + tg*vyg - 3 * yg))) / pow(tg, 3))*t;

		double vx = (pow(tg, 3)*vx0 + 3 * pow(t, 2)*(tg*(vx0 + vxg) + 2 * (x0 - xg)) - 2 * t*tg*(2 * tg*vx0 + tg*vxg + 3 * x0 - 3 * xg)) / (pow(tg, 3));
		//printf("vx: %0.2f\n", vx);
		double vy = (pow(tg, 3)*vy0 + 3 * pow(t, 2)*(tg*(vy0 + vyg) + 2 * (y0 - yg)) - 2 * t*tg*(2 * tg*vy0 + tg*vyg + 3 * x0 - 3 * yg)) / (pow(tg, 3));
		//printf("vy: %0.2f\n", vy);
		double ax = (6 * t*(tg*(vx0 + vxg) + 2 * (x0 - xg)) - 2 * tg*(2 * tg*vx0 + tg*vxg + 3 * x0 - 3 * xg)) / (pow(tg, 3));
       // printf("ax: %0.2f\n", ax);
		double ay = (6 * t*(tg*(vy0 + vyg) + 2 * (y0 - yg)) - 2 * tg*(2 * tg*vy0 + tg*vyg + 3 * y0 - 3 * yg)) / (pow(tg, 3));
       // printf("ay: %0.2f\n", ay);


		double xLeftd = (vx*(sqrt(pow((pow(vx, 2) + pow(vy, 2)), 3)) + (a*vy*ax) - (a*vx*ay))) / (sqrt(pow((pow(vx, 2) + pow(vy, 2)), 3)));

		double yLeftd = (vy*(sqrt(pow((pow(vx, 2) + pow(vy, 2)), 3)) + (a*vy*ax) - (a*vx*ay))) / (sqrt(pow((pow(vx, 2) + pow(vy, 2)), 3)));

		double xRightd = (vx*(sqrt(pow((pow(vx, 2) + pow(vy, 2)), 3)) - (a*vy*ax) + (a*vx*ay))) / (sqrt(pow((pow(vx, 2) + pow(vy, 2)), 3)));

		double yRightd = (vy*(sqrt(pow((pow(vx, 2) + pow(vy, 2)), 3)) - (a*vy*ax) + (a*vx*ay))) / (sqrt(pow((pow(vx, 2) + pow(vy, 2)), 3)));

		leftSpeed = sqrt(pow(xLeftd, 2) + pow(yLeftd, 2));
		
		//printf("Left speed: %0.2f\n", leftSpeed);
		rightSpeed = sqrt(pow(xRightd, 2) + pow(yRightd, 2));
	//	printf("Right speed: %0.2f\n", rightSpeed);
      
		printf("Kairys:  %0.1f   Desinys:  %0.1f  t: %0.1f pwmLeft: %d pwmRight: %d velocityLeft: %0.1f velocityRight: %0.1f	PID Left: %0.1f	PID Right: %0.1f\n", leftSpeed, rightSpeed, t,pwmLeft,pwmRight,VelocityLeft,VelocityRight,galutinisGreitisL,galutinisGreitisR);
		printf("ENCODERIAI K: %d	D: %d\n",leftSteps,rightSteps);
		double leftimpulsai = leftSpeed*b * 118;
		double rightimpulsai = rightSpeed*b * 118;
		
		/*
		if(FirstRun==0){
			usleep(200000);
			FirstRun=1;
		}
		*/

		usleep(100000);
		
				//printf("t: %f\n",t);
		//counter++;
		//printf("counter %d\n",counter);
		
			
		}
		
		
		}
		
		else{
			tg = (3 * (-vx0*xg - vxg*xg - vy0*yg - vyg*yg + sqrt(pow(((vx0 + vxg)*(-xg) + (vy0 + vyg)*(-yg)), 2) + 4 * (11 * pow(vx0, 2) + 2 * vx0*vxg + 11 * pow(vxg, 2) + 11 * pow(vy0, 2) + 2 * vy0*vyg + 11 * pow(vyg, 2))*(pow(xg, 2) + pow(yg, 2))))) / (11 * vx0*vx0 + 2 * vx0*vxg + 11 * vxg*vxg + 11 * vy0*vy0 + 2 * vy0*vyg + 11 * vyg*vyg);
		tgGlobal=tg;
			for (t = 0; t<=tg*3/5; t += b)
		{
		//outfiletpos.open("variables.txt",ios::app);
	
		double x, y;
		//randame x pozicijos funkcija nuo laiko
		x = (vx0 + (t*(t*(tg*(vx0 + vxg) + 2 * (-xg)) - tg*(2 * tg*vx0 + tg*vxg - 3 * xg))) / pow(tg, 3))*t;
		//randame y pozicijos funkcija nuo laiko
		y = (vy0 + (t*(t*(tg*(vy0 + vyg) + 2 * (-yg)) - tg*(2 * tg*vy0 + tg*vyg - 3 * yg))) / pow(tg, 3))*t;

		double vx = (pow(tg, 3)*vx0 + 3 * pow(t, 2)*(tg*(vx0 + vxg) + 2 * (x0 - xg)) - 2 * t*tg*(2 * tg*vx0 + tg*vxg + 3 * x0 - 3 * xg)) / (pow(tg, 3));
		//printf("vx: %0.2f\n", vx);
		double vy = (pow(tg, 3)*vy0 + 3 * pow(t, 2)*(tg*(vy0 + vyg) + 2 * (y0 - yg)) - 2 * t*tg*(2 * tg*vy0 + tg*vyg + 3 * x0 - 3 * yg)) / (pow(tg, 3));
		//printf("vy: %0.2f\n", vy);
		double ax = (6 * t*(tg*(vx0 + vxg) + 2 * (x0 - xg)) - 2 * tg*(2 * tg*vx0 + tg*vxg + 3 * x0 - 3 * xg)) / (pow(tg, 3));
       // printf("ax: %0.2f\n", ax);
		double ay = (6 * t*(tg*(vy0 + vyg) + 2 * (y0 - yg)) - 2 * tg*(2 * tg*vy0 + tg*vyg + 3 * y0 - 3 * yg)) / (pow(tg, 3));
       // printf("ay: %0.2f\n", ay);


		double xLeftd = (vx*(sqrt(pow((pow(vx, 2) + pow(vy, 2)), 3)) + (a*vy*ax) - (a*vx*ay))) / (sqrt(pow((pow(vx, 2) + pow(vy, 2)), 3)));

		double yLeftd = (vy*(sqrt(pow((pow(vx, 2) + pow(vy, 2)), 3)) + (a*vy*ax) - (a*vx*ay))) / (sqrt(pow((pow(vx, 2) + pow(vy, 2)), 3)));

		double xRightd = (vx*(sqrt(pow((pow(vx, 2) + pow(vy, 2)), 3)) - (a*vy*ax) + (a*vx*ay))) / (sqrt(pow((pow(vx, 2) + pow(vy, 2)), 3)));

		double yRightd = (vy*(sqrt(pow((pow(vx, 2) + pow(vy, 2)), 3)) - (a*vy*ax) + (a*vx*ay))) / (sqrt(pow((pow(vx, 2) + pow(vy, 2)), 3)));

		leftSpeed = sqrt(pow(xLeftd, 2) + pow(yLeftd, 2));
		
		//printf("Left speed: %0.2f\n", leftSpeed);
		rightSpeed = sqrt(pow(xRightd, 2) + pow(yRightd, 2));
	//	printf("Right speed: %0.2f\n", rightSpeed);
      
		printf("Kairys:  %0.1f   Desinys:  %0.1f  t: %0.1f pwmLeft: %d pwmRight: %d velocityLeft: %0.1f velocityRight: %0.1f	PID Left: %0.1f	PID Right: %0.1f\n", leftSpeed, rightSpeed, t,pwmLeft,pwmRight,VelocityLeft,VelocityRight,galutinisGreitisL,galutinisGreitisR);
		printf("ENCODERIAI K: %d	D: %d\n",leftSteps,rightSteps);
		double leftimpulsai = leftSpeed*b * 118;
		double rightimpulsai = rightSpeed*b * 118;
		
		/*
		if(FirstRun==0){
			usleep(200000);
			FirstRun=1;
		}
		*/

		usleep(100000);
		/*
		double skirtumasL, skirtumasR, kof;
		kof=0.1;
		skirtumasR=0;
		skirtumasL=0;
		if(VelocityRight<galutinisGreitisR){
			skirtumasR=(galutinisGreitisR-VelocityRight)*kof;
		}	
		else{
			skirtumasR=0;
		}
		if(VelocityLeft<galutinisGreitisL){
			skirtumasL=(galutinisGreitisL-VelocityLeft)*kof;
		}
		else {
			skirtumasL=0;
		}
		galutinisGreitisL=leftSpeed+skirtumasL;
		galutinisGreitisR=rightSpeed+skirtumasR;	
		*/
				//printf("t: %f\n",t);
		//counter++;
		//printf("counter %d\n",counter);
			//return bufferSpeed;
		
			
		}
			galutinisGreitisL=0;
			galutinisGreitisR=0;
			leftSpeed=0;
			rightSpeed=0;
			std::cout<<"TG, press any key  tg:" <<tg<<std::endl;
			key=getchar();
		}
			usleep(100000);
		//galutiniaiGreiciai = skaiciuoti((double)xPrad, (double)yPrad, 150, 0, vxg,vyg); //kairys[0] desinys[1] mm/s //commonVelocityApprox
		
		
		}
	}
}
int main()
{

	//Uncomment threads to check out each function
signal(SIGINT, sigintHandler);



	std::cerr << " --  MAIN START --  " << std::endl;
	wiringPiSetup();

 	 pthread_create(&laserThread, NULL, LaserMeasurement, NULL);
	std::cout<<"choose continious mode [0] or test [1]:  "<< std::endl;
	mode=getchar();

	driveMode=mode-'0';

	printf("mode: %c %d\n", mode,driveMode);
	usleep(1000000);
	pthread_create(&mpuThread, NULL, MPU_Measurement,NULL);

	pthread_create(&encoderThread, NULL, EncoderMeasurement, NULL);

	pthread_create(&pwmThread, NULL, PwmControl, NULL);

	//pthread_create(&bluetoothThread, NULL, BluetoothConnection, NULL);

	pthread_create(&sienosSekimas, NULL, SienosSekimas, NULL);
	
	pthread_create(&velocityCalculation, NULL, VelocityCalculation, NULL);

	pthread_create(&kelioRadimas, NULL, KelioRadimas, NULL);

 //pthread_create(&spausdinimas, NULL, Spausdinimas, NULL);


	while (!exitFlag)
	{
		sleep(10000);
	}
	movingMotor(0,0,STOP,STOP);
	std::cerr << " Turning main thread off " << std::endl;

	//getchar();

	return 0;
}
