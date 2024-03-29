
/*
 * Stair climber
 * By Caroline Rausch, Sam Daitzman, David Tarazi, and Dieter Brehm
 */
#include "Arduino.h"
#include "Wire.h"
#include "pid.h"
#include "drivetrain.h"
#include "CytronMotorDriver.h"
#include "Adafruit_MS_PWMServoDriver.h"
#include "Adafruit_MotorShield.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Servo.h"


//<<< GYRO SETUP >>>

// object representing the gyro
MPU6050 mpu;

//Servo Setup
Servo balancer;

#define INTERRUPT_PIN 2  // use pin 2 for interupts

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// output angle of sensor for pid use
double currentAngle;


// Interrupt detection
// may be a point of contention(!)
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpuInterrupt = true;
}

// Define a pid controller -> pid(setpoint (angle), p, i, d)
// pid pid_control = pid(0, 1.0, 0.0, 0.0);

// delay variables for handling delayed print statements
unsigned long lastMillis;
unsigned long wait = 1000; // ms to delay code block

//<<< MOTOR SETUP >>>

CytronMD l_motor(PWM_DIR, 3, 4); // PWM = Pin 3, DIR = Pin 4
CytronMD r_motor(PWM_DIR, 5, 6); // PWM = Pin 3, DIR = Pin 4

// start/stop variable
int run = 0;

// Use mode to shift between different drive styles
int mode = 0;

// struct for handling a pair of motor speeds
struct Speeds motor_speed;

void send_motor_cmd(int l, int r) {
		// drives motors at desired speeds
		// (int left[-255 to 255], int right[-255 to 255]) -> void
		// handles negative values as reverse direction
		l_motor.setSpeed(l);
		r_motor.setSpeed(-r);
		delay(10);
}

void drive_all(int speed) {
		// drive all wheels at the same speed
		send_motor_cmd(speed, speed);
}

void turn_counterclockwise(int speed){
		// turn counterclockwise in place
		send_motor_cmd(speed/2, -speed/2);
}

void turn_clockwise(int speed){
		// turn clockwise in place
		send_motor_cmd(-speed/2, speed/2);
}

void serialReader() {
	// read the serial buffer for new operations which are:
	// 	   START code -> S
  //     STOP code  -> E
	// 	   VEL code   -> V50.0
	// NOTE: parse float is blocking, we'll have to see how messy that is.
	// AND: assumes that the end of a line has a line ending character
	if (Serial.available() > 0) {
		if (Serial.peek() == 's') {
				// start the program
				Serial.read();
				Serial.println("starting the drive loop");
				run = 1;
		} else if (Serial.peek() == 'e') {
				// end the program
				Serial.read();
				Serial.println("ending the drive loop");
				mode = 0;
				run = 0;
		} else if (Serial.peek() == ',') {
				// read new derivative value
				Serial.read();
				float vel = Serial.parseFloat();
				Serial.println("lower speed");
				motor_speed.linvel = abs(motor_speed.linvel - 30);
		} else if (Serial.peek() == '.') {
				// read new derivative value
				Serial.read();
				float vel = Serial.parseFloat();
				Serial.println("increase speed");
				motor_speed.linvel = abs(motor_speed.linvel + 30);
		// Use modes for keyboard drive
		} else if (Serial.peek() == 'r') {
				// Stop
				Serial.read();
				mode = 0;
		} else if (Serial.peek() == 't') {
				// Forwards
				Serial.read();
				mode = 1;
		} else if (Serial.peek() == 'F') {
				// Counter-clockwise turn in place
				Serial.read();
				mode = 2;
		} else if (Serial.peek() == 'g') {
				// Backwards
				Serial.read();
				mode = 3;
		} else if (Serial.peek() == 'H') {
				// Clockwise turn in place
				Serial.read();
				mode = 4;
		} else {
			while(Serial.available()) {
				Serial.read();
				delay(10);
			}
		}
	}
}

void setup() {
	// Servo setup
	balancer.attach(10);
	balancer.write(12); //initial angle

	// IMU config
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

	// setup the motor shield controller
	send_motor_cmd(0, 0);
	Serial.begin(115200);
	delay(10);

	// Initialize the imu
	mpu.initialize();
	pinMode(INTERRUPT_PIN, INPUT);
	devStatus = mpu.dmpInitialize();

	// set offsets for factory error
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788);

	// start point of the gyroscope init, might need error handling from 
	// prior code base (see integration branch)

	// Calibration Time: generate offsets and calibrate our MPU6050
	mpu.CalibrateAccel(6);
	mpu.CalibrateGyro(6);
	mpu.PrintActiveOffsets();
	// turn on the DMP, now that it's ready
	Serial.println(F("Enabling DMP..."));
	mpu.setDMPEnabled(true);

	// enable Arduino interrupt detection
	Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
	Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
	Serial.println(F(")..."));
	attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
	mpuIntStatus = mpu.getIntStatus();

	// set our DMP Ready flag so the main loop() function knows it's okay to use it
	Serial.println(F("DMP ready! Waiting for first interrupt..."));
	dmpReady = true;

	// get expected DMP packet size for later comparison
	packetSize = mpu.dmpGetFIFOPacketSize();
}

void loop() {
	//print out the sensor reading to the serial

	// respond to serial operations
	serialReader();
	if (run == 1) {
		// For drive commands
		// T = forward
		// F = counterclockwise turn in place
		// G = backward
		// H = clockwise turn in place
		// R = stop

		// S = run
		// E = stop running

    	// log values

		// LOG,time,left,right,sensor_left, sensor_right
		Serial.print("LOG,Motors,");
		Serial.print(String(motor_speed.linvel));
		Serial.print(",Time,");
		Serial.println(millis());
		switch(mode){
			case 0:
				drive_all(0);
				break;
			case 1:
				drive_all(motor_speed.linvel);
				break;
			case 2:
				turn_counterclockwise(motor_speed.linvel);
				break;
			case 3:
				drive_all(-motor_speed.linvel);
				break;
			case 4:
				turn_clockwise(motor_speed.linvel);
				break;
			default:
				drive_all(0);
				break;
		// l_motor.setSpeed(200);
		// r_motor.setSpeed(200);
		}
	} else {
		drive_all(0);
	}

	// balance
	// wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
						// try to get out of the infinite loop
						fifoCount = mpu.getFIFOCount();
        }
        // other program behavior stuff here
    }

	// INT error handling
	// reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
    fifoCount = mpu.getFIFOCount();
		if(fifoCount < packetSize){
				//Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
				// This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
		}
	// check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
				//  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
        Serial.println(F("FIFO overflow!"));

	// otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {

				// read a packet from FIFO
				while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
						mpu.getFIFOBytes(fifoBuffer, packetSize);
						// track FIFO count here in case there is > 1 packet available
						// (this lets us immediately read more without waiting for an interrupt)
						fifoCount -= packetSize;
				}
				// get current angle and map it to servo angle
				currentAngle = ypr[2] * 180/M_PI;

				double servoOutput = map(currentAngle, 0, 30, 12,42);

				mpu.dmpGetQuaternion(&q, fifoBuffer);
				mpu.dmpGetGravity(&gravity, &q);
				mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
				Serial.print("Yaw Pitch Roll PID\t");
				Serial.print(ypr[0] * 180/M_PI);
				Serial.print("\t");
				Serial.print(ypr[1] * 180/M_PI);
				Serial.print("\t");
				Serial.print(ypr[2] * 180/M_PI);
				Serial.print("\t");
				Serial.println(servoOutput);

				// output of gyro: -30-> 150

				// apply a safety range to the output
				if(servoOutput > 0 && servoOutput < 150)
				balancer.write(servoOutput);
		}
}
