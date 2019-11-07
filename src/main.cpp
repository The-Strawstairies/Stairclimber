// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain

#include "Arduino.h"
#include "Wire.h"
#include "pid.h"
#include "main.h"
#include "Servo.h"

//Servo Setup
Servo balancer;

struct imu_data imu_vals;

double current_angle = 0;

// Define a pid controller -> pid(setpoint (angle), p, i, d)
pid pid_control = pid(0, 1.0, 0.0, 0.0);

// delay variables for handling delayed print statements
unsigned long lastMillis;
unsigned long wait = 1000; // ms to delay code block

void read_accel(){
	// Reads the accelerometer values and updates the imu_vals struct
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);
	Wire.requestFrom(MPU_ADDR,3*2,true);  // request a total of 6 registers

	// /16384.0 converts to g force
	imu_vals.AcX = double(Wire.read()<<8|Wire.read()) / 16384.0;  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)   
	imu_vals.AcY = double(Wire.read()<<8|Wire.read()) / 16384.0;  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
	imu_vals.AcZ = double(Wire.read()<<8|Wire.read()) / 16384.0;  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
}

void read_gyro(){
	// Reads the gyro values and updates the imu_vals struct
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x43);  // starting with register 0x43 (GYRO_XOUT_H)
	Wire.endTransmission(false);
	Wire.requestFrom(MPU_ADDR,3*2,true);  // request a total of 6 registers

	// /131.0 converts to rotation in degrees (depends on gyro mode set - refer to datasheet)
	imu_vals.GyX = double(Wire.read()<<8|Wire.read()) / 131.0;  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	imu_vals.GyY = double(Wire.read()<<8|Wire.read()) / 131.0;  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
	imu_vals.GyZ = double(Wire.read()<<8|Wire.read()) / 131.0;  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void update_angle(double gyro_reading) {
	// update the current angle given change from gyro & time step
	if (abs(gyro_reading) > 10){
		current_angle += (gyro_reading);
	}
}

void print_imu_values(){
	Serial.print("AcX = "); Serial.print(imu_vals.AcX);
	Serial.print(" | AcY = "); Serial.print(imu_vals.AcY);
	Serial.print(" | AcZ = "); Serial.print(imu_vals.AcZ);
	
	Serial.print(" | GyX = "); Serial.print(imu_vals.GyX);
	Serial.print(" | GyY = "); Serial.print(imu_vals.GyY);
	Serial.print(" | GyZ = "); Serial.println(imu_vals.GyZ);
}


void setup(){
	// Servo setup
	balancer.attach(10);
	
	// IMU config
	Wire.begin();
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x6B);  // PWR_MGMT_1 register
	Wire.write(0);     // set to zero (wakes up the MPU-6050)
	Wire.endTransmission(true);
	
	// Setting gyro config
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x1B); 		// gyro config register
	Wire.write(0); // sets to +/- 250 deg/sec (lower range, higher sensitivity) 
	Wire.endTransmission(true);
	// Setting accel config
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x1C); 		// gyro config register
	Wire.write(0); // sets to +/- 2g (lower range, higher sensitivity) 
	Wire.endTransmission(true);

	Serial.begin(115200);
}

void loop(){
	read_accel();
	read_gyro();

	// adjust the current angle
	update_angle(imu_vals.GyX);

	// given angle, update current angle
	pid_control.loopStep(current_angle);
	balancer.write(12);

	// if (millis() % 1000 == 0){
	// 	Serial.print("pid output");
	// 	Serial.print(pid_control.output);
	// 	Serial.print(" raw reading ");
	// 	Serial.println(imu_vals.GyX);
	// }

	// wait between prints
	if (millis() - lastMillis >= wait) {
		Serial.print("pid output");
		Serial.print(pid_control.output);
		Serial.print(" raw reading ");
		Serial.println(imu_vals.GyX);

		lastMillis = millis();
	}

	//print_imu_values();
	//delay(200);
}