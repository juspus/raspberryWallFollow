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

int fd1;
const char *file_Name = "/dev/i2c-1";
int  mpuAddress = 0x68;
float xaccel;
float yaccel;
float zaccel;

float xgyro;
float ygyro;
float zgyro;
