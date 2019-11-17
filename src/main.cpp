
/*
 * Line Following Robot
 * By Sam Daitzman, David Tarazi, and Dieter Brehm
 */
#include "Arduino.h"
#include "Wire.h"
#include "pid.h"
#include "drivetrain.h"
#include "CytronMotorDriver.h"

CytronMD l_motor(PWM_DIR, 3, 4); // PWM = Pin 3, DIR = Pin 4
CytronMD r_motor(PWM_DIR, 3, 4); // PWM = Pin 3, DIR = Pin 4

// start/stop variable
int run = 0;

// Use mode to shift between different drive styles
int mode = 0;

// struct for handling a pair of motor speeds
struct Speeds motor_speed;

// PID_Controller pid = PID_Controller(0, 1.0, 0.0, 0.0);;

void send_motor_cmd(int l, int r) {
		// drives motors at desired speeds
		// (int left[-255 to 255], int right[-255 to 255]) -> void
		// handles negative values as reverse direction
		l_motor.setSpeed(l);
		r_motor.setSpeed(r);
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
		if (Serial.peek() == 'S') {
				// start the program
				Serial.read();
				Serial.println("starting the drive loop");
				run = 1;
		} else if (Serial.peek() == 'E') {
				// end the program
				Serial.read();
				Serial.println("ending the drive loop");
				mode = 0;
				run = 0;
		} else if (Serial.peek() == 'V') {
				// read new derivative value
				Serial.read();
				float vel = Serial.parseFloat();
				Serial.println("setting speed constant");
				motor_speed.linvel = vel;
		// Use modes for keyboard drive
		} else if (Serial.peek() == 'R') {
				// Stop
				Serial.read();
				mode = 0;
		} else if (Serial.peek() == 'T') {
				// Forwards
				Serial.read();
				mode = 1;
		} else if (Serial.peek() == 'F') {
				// Counter-clockwise turn in place
				Serial.read();
				mode = 2;
		} else if (Serial.peek() == 'G') {
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
		// setup the motor shield controller
		send_motor_cmd(0, 0);
		Serial.begin(115200);
		delay(10);
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
		}

		// log values
		// LOG,time,left,right,sensor_left, sensor_right
		//Serial.print("LOG,Motors,");
		//Serial.println(String(motor_speed.linvel));

	} else {
		drive_all(0);
	}
}
