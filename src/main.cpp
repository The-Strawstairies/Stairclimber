
/*
 * Line Following Robot
 * By Sam Daitzman, David Tarazi, and Dieter Brehm
 */
#include "Arduino.h"
#include "Adafruit_MotorShield.h"
#include "Wire.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"


struct Speeds {
	// struct
	// default speeds are 0
	float left;
	float right;
};

class PID_Controller {
public:
		// error tracking
		float previousError = 0;
		float error = 0;

		// track prior deriv and int
		float integral = 0;
		float derivative = 0;

		// prior output
		float output = 0;

		// tuning params
		float setPoint;
		float Kp;
		float Ki;
		float Kd;

		// distance between wheels
		float wheelBase = .1016; // m

		// last micros reading
		unsigned long lastRun;

		// linear velocity
		float linVel = 25.0; // hey how fast is this?

		// initialization function for tuning PID
		PID_Controller(float setPoint, float Kp, float Ki, float Kd);

		// empty constructor for initialization
		void setSetpoint(float setPoint);
		void setKp(float Kp);
		void setKi(float Ki);
		void setKd(float Kd);
		void setSpeed(float sd_new);

		// Defines a function that steps the motor speeds based on PID
		void loopStep(float leftSensor, float rightSensor, Speeds *motor_speed);
};

// indicate which motor pins are being used on
// the motor shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);

// setup pins for the two ir sensors
int IRVal1 = 0;
int IRVal2 = 0;
const int IRPin1 = 0;
const int IRPin2 = 1;

// start/stop variable
int run = 0;

// struct for handling a pair of motor speeds
struct Speeds motor_speed;

// create a new PID controller
// for initial tuning, set the I and D values to 0 so we just have
// to tune the proportional part of the system.
PID_Controller pid = PID_Controller(0, 1.0, 0.0, 0.0);;

void send_motor_cmd(int left, int right) {
		// drives motors at desired speeds
		// (int left[-255 to 255], int right[-255 to 255]) -> void
		// handles negative values as reverse direction
		myMotor1->setSpeed(abs(left));
		myMotor2->setSpeed(abs(right));

		if(left>0) myMotor1->run(FORWARD);
		else myMotor1->run(BACKWARD);

		if(right>0) myMotor2->run(FORWARD);
		else myMotor2->run(BACKWARD);
}


float sensor_read(int sensor_num) {
		// sensor testing function
		// just prints sensor vals

		int reading;

		if (sensor_num == 1) {
				reading = analogRead(IRPin1);
		} else {
				reading = analogRead(IRPin2);
		}
		return reading;
}


PID_Controller::PID_Controller(float setPoint_new, float Kp_new, float Ki_new, float Kd_new) {
		// construct!
		// PID_Controller PID = PID_Controller(5.0, 6.0, 6.0, 6.0)

		setPoint = setPoint_new;
		Kp = Kp_new;
		Ki = Ki_new;
		Kd = Kd_new;
		lastRun = micros();
};


void PID_Controller::setSetpoint(float setPoint_new) {
		// change the set point
		setPoint = setPoint_new;
}


void PID_Controller::setKp(float Kp_new) {
		// change the P constants used in the live loop
		Kp = Kp_new;
}


void PID_Controller::setKi(float Ki_new) {
		// change the I constants used in the live loop
		Ki = Ki_new;
}


void PID_Controller::setKd(float Kd_new) {
		// change the D constants used in the live loop
		Kd = Kd_new;
}


void PID_Controller::setSpeed(float sd_new) {
		// change the D constants used in the live loop
	  linVel = sd_new;
}


void PID_Controller::loopStep(float leftSensor, float rightSensor, Speeds *motor_speed) {
		// Takes in the current left and right sensor values every loop
		// Runs a step to update the motor speeds (speed) based on the error in the PID loop
		// output represented as an angular velocity

		// timing math
		unsigned long now = micros();
		unsigned long dt = now - lastRun;
		lastRun = now;

		// run the loop
		// 	error = setpoint - measured_value <- difference btw sensors
		error = setPoint - (leftSensor - rightSensor); // +- 1023
		// 	integral = integral + error * dt
		integral = integral + (error * dt);
		// 	derivative = (error - previous_error) / dt
		derivative = (error - previousError) / dt;
		// 	output = Kp * error + Ki * integral + Kd * derivative
		output = (Kp * error) + (Ki * integral) + (Kd * derivative);
		Serial.println(String(output));
		// 	previous_error = error
		previousError = error;

		// set speed with speed.left // speed.right = output;
		// idea: output of PID as angular velocity
		
		// Vl = s - (w d)/2
		// vr = s + (w d)/2
		// minimum of output: 0
		// maximum of output:
		motor_speed->left  = linVel - (output * wheelBase) / 2;
		motor_speed->right = linVel + (output * wheelBase) / 2;
};


void serialReader(PID_Controller *pid) {
	// read the serial buffer for new operations which are:
	//     START code -> S
  //     STOP code  -> E
  //     P code     -> P1.0
  //     I code     -> I0.0
	//     D code     -> D0.0
	// NOTE: parse float is blocking, we'll have to see how messy that is.
	// AND: assumes that the end of a line has a line ending character
	if (Serial.available() > 0) {
		if (Serial.peek() == 'S') {
				// start the program
				Serial.read();
				Serial.println("start pid loop");
				run = 1;
		} else if (Serial.peek() == 'E') {
				// end the program
				Serial.read();
				Serial.println("ending the pid loop");
				run = 0;
		} else if (Serial.peek() == 'P') {
				// read new proportional const
				Serial.read();
				float kp = Serial.parseFloat();
				Serial.println("setting p constant");
				pid->setKp(kp);
		} else if (Serial.peek() == 'I') {
				// read new integral const
				Serial.read();
				float ki = Serial.parseFloat();
				Serial.println("setting I constant");
				pid->setKi(ki);
		} else if (Serial.peek() == 'D') {
				// read new derivative value
				Serial.read();
				float kd = Serial.parseFloat();
				Serial.println("setting D constant");
				pid->setKd(kd);
		} else if (Serial.peek() == 'V') {
				// read new derivative value
				Serial.read();
				float vel = Serial.parseFloat();
				Serial.println("setting speed constant");
				pid->setSpeed(vel);
		} else {
			while(Serial.available()) {
				Serial.read();
				delay(10);
			}
		}
		Serial.print("I'm ready!");
	}
}

void setup() {
		// setup the motor shield controller
		AFMS.begin();
		send_motor_cmd(0,0);
		Serial.begin(115200);
		delay(10);
}

void loop() {
		//print out the sensor reading to the serial
		// Serial.println(sensor_test());
		// delay(1);

		float leftRead  = sensor_read(1);
		float rightRead = sensor_read(2);


		// respond to serial operations
		serialReader(&pid);

		if (run == 1) {
			// // check if system should run based on serial input

			// // perform PID calculations for the current step
			pid.loopStep(leftRead, rightRead, &motor_speed);

			// // update motor speeds to the motor controller
			send_motor_cmd(motor_speed.left, motor_speed.right);

			// // log values
			// // LOG,time,left,right,sensor_left, sensor_right
			Serial.print("LOG,Motors,");
			Serial.print(String(motor_speed.left));
			Serial.print(",");
			Serial.print(String(motor_speed.right));
			Serial.print(",Sensors,");
			Serial.print(String(leftRead));
			Serial.print(",");
			Serial.print(String(rightRead));
			Serial.print(",PID,");
			Serial.print(String(pid.Kp));
			Serial.print(",");
			Serial.print(String(pid.Ki));
			Serial.print(",");
			Serial.print(String(pid.Kd));
			Serial.print(",");
			Serial.print(String(run));
			Serial.print(",");
			Serial.println(String(millis()));
		} else {
			myMotor1->setSpeed(0);
			myMotor2->setSpeed(0);
		}
}

