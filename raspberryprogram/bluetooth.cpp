#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
//#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <time.h>
#include <sys/time.h>
#include <csignal>

//paketo ilgis -
//du baitus 0xF1 ir 0xF2 tegul jie buna markeris srauto pradzios
//du baitai headeriui
//24 baitai etanolio matavimam
//crc - 16 bitu (2 baitai) (susirast internete)
/*
* 16 12 5
* this is the CCITT CRC 16 polynomial X + X + X + 1.
* This is 0x1021 when x is 2, but the way the algorithm works
* we use 0x8408 (the reverse of the bit pattern). The high
* bit is always assumed to be set, thus we only use 16 bits to
* represent the 17 bit value.
//*/

#define POLY 0x8408 /* 1021H bit reversed*/
#define PACKET_BYTES_COUNT 72 // Paketo ilgis
unsigned short crcu16(unsigned char *data_p, unsigned short length) {
unsigned char i;
unsigned int data;
unsigned int crc = 0xffff;

if (length == 0)
return (~crc);

do {
for (i=0, data=(unsigned int)0xff & *data_p++; i < 8; i++, data >>= 1) {
if ((crc & 0x0001) ^ (data & 0x0001))
crc = (crc >> 1) ^ POLY;
else
crc >>= 1;
}
}
while (--length);
return (crc);
}
//Funkcija laikui skaiciuoti

long long current_timestamp() {
    struct timeval te;
    gettimeofday(&te, NULL); // get current time
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // calculate milliseconds
    //printf("milliseconds: %lld \n", milliseconds);
    return milliseconds;
}

// Funkcija duomenims issiusti
int SendData ( int OpticalSensor1,  int OpticalSensor2, int OpticalSensor3, int OpticalSensor4,
int OpticalSensor5, int OpticalSensor6, int GyroscopeX, int GyroscopeY, int GyroscopeZ, int AccelerometerX,
int AccelerometerY, int AccelerometerZ, int Temperature1, int Temperature2,int EncoderLeft, int EncoderRight,int client)
{

unsigned char buffer [PACKET_BYTES_COUNT];
int TCP_CLIENT_BUFFER_LENGTH=72;
int status = 0;


    buffer[0] = 0xF1; buffer[1] = 0xF2; // begin stream
buffer[2] = 0; buffer[3] = 12; // header (?)
// ETH values

buffer [7] = (OpticalSensor1>>24) & 0xFF;
buffer [6] = (OpticalSensor1>>16) & 0xFF;
buffer [5] = (OpticalSensor1>>8) & 0xFF;
buffer [4] = OpticalSensor1 & 0xFF;

buffer[11] = (OpticalSensor2>>24) & 0xFF;
buffer[10] = (OpticalSensor2>>16) & 0xFF;
buffer [9] = (OpticalSensor2>>8) & 0xFF;
buffer [8] = OpticalSensor2 & 0xFF;

buffer [15] = (OpticalSensor3>>24) & 0xFF;
buffer [14] = (OpticalSensor3>>16) & 0xFF;
buffer [13] = (OpticalSensor3>>8) & 0xFF;
buffer [12] = OpticalSensor3 & 0xFF;

buffer [19] = (OpticalSensor4>>24) & 0xFF;
buffer [18] = (OpticalSensor4>>16) & 0xFF;
buffer [17] = (OpticalSensor4>>8) & 0xFF;
buffer [16] = OpticalSensor4 & 0xFF;

buffer [23] = (OpticalSensor5>>24) & 0xFF;
buffer [22] = (OpticalSensor5>>16) & 0xFF;
buffer [21] = (OpticalSensor5>>8) & 0xFF;
buffer [20] = OpticalSensor5 & 0xFF;

buffer [27] = (OpticalSensor6>>24) & 0xFF;
buffer [26] = (OpticalSensor6>>16) & 0xFF;
buffer [25] = (OpticalSensor6>>8) & 0xFF;
buffer [24] = OpticalSensor6 & 0xFF;

buffer [31] = (GyroscopeX>>24) & 0xFF;
buffer [30] = (GyroscopeX>>16) & 0xFF;
buffer [29] = (GyroscopeX>>8) & 0xFF;
buffer [28] = GyroscopeX & 0xFF;

buffer [35] = (GyroscopeY>>24) & 0xFF;
buffer [34] = (GyroscopeY>>16) & 0xFF;
buffer [33]= (GyroscopeY>>8) & 0xFF;
buffer [32] = GyroscopeY & 0xFF;

buffer [39] = (GyroscopeZ>>24) & 0xFF;
buffer [38] = (GyroscopeZ>>16) & 0xFF;
buffer [37] = (GyroscopeZ>>8) & 0xFF;
buffer [36] = GyroscopeZ & 0xFF;

buffer [43] = (AccelerometerX>>24) & 0xFF;
buffer [42] = (AccelerometerX>>16) & 0xFF;
buffer [41] = (AccelerometerX>>8) & 0xFF;
buffer [40] = AccelerometerX & 0xFF;

buffer [47] = (AccelerometerY>>24) & 0xFF;
buffer [46] = (AccelerometerY>>16) & 0xFF;
buffer [45] = (AccelerometerY>>8) & 0xFF;
buffer [44] = AccelerometerY & 0xFF;

buffer [51] = (AccelerometerZ>>24) & 0xFF;
buffer [50] = (AccelerometerZ>>16) & 0xFF;
buffer [49] = (AccelerometerZ>>8) & 0xFF;
buffer [48] = AccelerometerZ & 0xFF;

buffer [55] = (Temperature1>>24) & 0xFF;
buffer [54] = (Temperature1>>16) & 0xFF;
buffer [53] = (Temperature1>>8) & 0xFF;
buffer [52] = Temperature1 & 0xFF;

buffer [59] = (Temperature2>>24) & 0xFF;
buffer [58] = (Temperature2>>16) & 0xFF;
buffer [57] = (Temperature2>>8) & 0xFF;
buffer [56] = Temperature2 & 0xFF;

buffer [63] = (EncoderLeft>>24) & 0xFF;
buffer [62] = (EncoderLeft>>16) & 0xFF;
buffer [61] = (EncoderLeft>>8) & 0xFF;
buffer [60] = EncoderLeft& 0xFF;

buffer [67] = (EncoderRight>>24) & 0xFF;
buffer [66] = (EncoderRight>>16) & 0xFF;
buffer [65] = (EncoderRight>>8) & 0xFF;
buffer [64] = EncoderRight& 0xFF;

unsigned short crc16code = crcu16(&buffer[0], 68);
buffer[68] = (unsigned char)(crc16code & 0xFF);
buffer[69] = (unsigned char)((crc16code >> 8) & 0xFF);
buffer[70] = 0xE3; buffer[71] = 0xE4;





        if (status==0)
        {
            status = write(client, buffer, TCP_CLIENT_BUFFER_LENGTH);
            //printf("Buffer sent \n");
            return status;
        }


        if( status < 0 )
        {
             perror("Error 404 ('_')");
           //  close(client);
             return status;

        }

}


