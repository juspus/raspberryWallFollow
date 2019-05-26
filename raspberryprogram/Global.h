#include <stdint.h>
#include <stdio.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>



//PWM=========================
#define FORW 0
#define BACKW 1
#define STOP 2
int pwmLeft;
	int pwmRight;
void movingMotor(int SpeedA, int SpeedB, int modeA, int modeB);

//BlueTooth===================


#define POLY 0x8408 /* 1021H bit reversed*/
#define PACKET_BYTES_COUNT 72 // Paketo ilgis

 struct sockaddr_rc loc_addr = { 0 }, rem_addr = { 0 };
    char buf[1024] = { 0 };
   // char send[1024];
    int s, client, bytes_read;
    socklen_t opt = sizeof(rem_addr);
    
long long current_timestamp();

int SendData ( int OpticalSensor1,  int OpticalSensor2, int OpticalSensor3, int OpticalSensor4,
int OpticalSensor5, int OpticalSensor6, int GyroscopeX, int GyroscopeY, int GyroscopeZ, int AccelerometerX,
int AccelerometerY, int AccelerometerZ, int Temperature1, int Temperature2,int EncoderLeft, int EncoderRight,int client);



//MPU6050====================
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47
#define MPU_POWER1   0x6b
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C


float accelX,accelY,accelZ,gyroX,gyroY,gyroZ;
int aX, aY, aZ, gX,gY,gZ;

int read_raw_data(int addr);
void MPU6050_Init();
bool TestConnection();
void I2C_Init();

//ENCODER===================
uint rightSteps;
uint leftSteps;


uint GetStepsLeft();
uint GetStepsRight();

//LASERS====================
uint16_t lasersData[6]={0,0,0,0,0,0};

//Sienos aproksimavimas

uint16_t l1, l2, l3, l4, l5, l6;


float koord[5];
float xPrad,yPrad,vKampas;
float interceptG,slopeG;

float *gautiKoordinates(uint16_t l1,uint16_t l2,uint16_t l3,uint16_t l4,uint16_t l5,uint16_t l6);
void ilgisIkoordinate(uint16_t ilgis1, uint16_t ilgis2, uint16_t ilgis3, uint16_t ilgis4, uint16_t ilgis5, uint16_t ilgis6);
void sienosLygtis1_2();
void sienosLygtis2_3();
void sienosLygtis3_4();
void sienosLygtis4_5();
void sienosLygtis5_6();
void menamaSienosLygtis();
void rastiTaska(double a_tieses, double b_tieses);


//Kelio radimas
double motorSpeed[2]={0,0};
float *skaiciuoti(double xg, double yg, double vx0, double vy0, double vxg, double vyg);
float galutiniaiGreiciai[2];
double kampasGlobal;
float galutinisGreitisL=1;
float galutinisGreitisR=0;
double leftSpeed;
double rightSpeed;
//double commonVelocityApprox;
double VelocityRight=0;
double VelocityLeft=0;
double tgGlobal;
