#ifndef MAIN_H // include guard
#define MAIN_H

#define MPU_ADDR 0x68  // I2C address of the MPU-6050 with AD0 set low

struct imu_data{
	double AcX,AcY,AcZ,GyX,GyY,GyZ;
};

#endif