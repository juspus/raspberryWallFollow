#include <stdio.h>
#include <linux/i2c-dev.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>
#include "time.h"
#include <sys/time.h>
#include <stdbool.h>


#define ACCEL_XOUT 0x3B
#define ACCEL_YOUT 0x3D
#define ACCEL_ZOUT 0x3F
#define GYRO_XOUT  0x43
#define GYRO_YOUT  0x45
#define GYRO_ZOUT  0x47
#define MPU_POWER1   0x6b
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define ACCEL_CONFIG 0x1C



int fd1;
const char *file_name = "/dev/i2c-1";
int  mpuAddress = 0x68;

int read_raw_data(int addr){ //MPU uses two 8Bit registers to store data 16bit data value.
                               //Reads two registers and returns 16bit value.
    short high_byte,low_byte,value;
    
    high_byte = i2c_smbus_read_byte_data(fd1, addr);
    usleep(50);
    low_byte = i2c_smbus_read_byte_data(fd1, addr+1);
    
    
    value = (high_byte << 8) | low_byte;
    return value;
}
void MPU6050_Init(){
    
        
        i2c_smbus_write_byte_data (fd1, MPU_POWER1, 0x01);   //Disable sleep mode
        i2c_smbus_write_byte_data (fd1, SMPLRT_DIV, 0x07);
        i2c_smbus_write_byte_data (fd1, ACCEL_CONFIG, 0x00);
        i2c_smbus_write_byte_data (fd1, GYRO_CONFIG, 0x00);
       
        usleep(2);
     
        // +- 200°/s, and acc by default +-2g's
    
    
    }



void I2C_Init()
{
    if ((fd1 = open(file_name, O_RDWR)) < 0) {
        printf("Failed to open i2c port\n");
        exit(1);
    }
    
    if (ioctl(fd1, I2C_SLAVE, mpuAddress) < 0) {
        printf("Unable to get bus access to talk to slave\n");
        exit(1);
    }
}
