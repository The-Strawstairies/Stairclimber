
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
	float linvel;
	float lf;
	float rf;
	float lb;
	float rb;
};

// class drivetrain {
// public: 
// 	// linear velocity
// 	float linVel = 25.0;

// 	// motors in use in drive train
// 	Adafruit_DCMotor lf;
// 	Adafruit_DCMotor rf;
// 	Adafruit_DCMotor lb;
// 	Adafruit_DCMotor rb;

// 	// constructor
// 	drivetrain();
	
// 	// switch drive type
// 	void setDrivetype(uint8_t cmd);
	
// 	// change linear velocity
// 	void setSpeed(float linVel);
// };

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
// Defines all four motors
Adafruit_DCMotor *lf_motor = AFMS.getMotor(0);
Adafruit_DCMotor *rf_motor = AFMS.getMotor(1);
Adafruit_DCMotor *lb_motor = AFMS.getMotor(2);
Adafruit_DCMotor *rb_motor = AFMS.getMotor(3);

// start/stop variable
int run = 0;

// struct for handling a pair of motor speeds
struct Speeds motor_speed;


// create a new PID controller
// for initial tuning, set the I and D values to 0 so we just have
// to tune the proportional part of the system.

// PID_Controller pid = PID_Controller(0, 1.0, 0.0, 0.0);;

void send_motor_cmd(int lf, int rf, int lb, int rb) {
		// drives motors at desired speeds
		// (int left[-255 to 255], int right[-255 to 255]) -> void
		// handles negative values as reverse direction
		lf_motor->setSpeed(abs(lf));
		rf_motor->setSpeed(abs(rf));
		rb_motor->setSpeed(abs(rb));
		lb_motor->setSpeed(abs(lb));

		if(lf>0) lf_motor->run(FORWARD);
		else lf_motor->run(BACKWARD);

		if(rf>0) rf_motor->run(FORWARD);
		else rf_motor->run(BACKWARD);

		if(lb>0) lb_motor->run(FORWARD);
		else lb_motor->run(BACKWARD);

		if(rb>0) rb_motor->run(FORWARD);
		else rb_motor->run(BACKWARD);
}

void drive_all(int speed) {
		// drive all wheels at the same speed
		send_motor_cmd(speed, speed, speed, speed);
}

void drive_front(int speed) {
		// drive front wheels of the robot
		send_motor_cmd(speed, speed, 0, 0);
}

void drive_back(int speed) {
		// drive back wheels of the robot
		send_motor_cmd(0, 0, speed, speed);
}

// drivetrain::drivetrain() {
	
// }

// void drivetrain::setDrivetype(uint8_t cmd) {
// 		switch (cmd) {
// 		case ALL:
// 			// drive all wheels
			
// 			break;
// 		case FRONT:
// 			// drive front wheels
// 			break;
// 		case BACK:
// 			// drive back wheels
// 			break;
// 		}
// }

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
		
		//motor_speed->left  = linVel - (output * wheelBase) / 2;
		//motor_speed->right = linVel + (output * wheelBase) / 2;
};


void serialReader() {
	// read the serial buffer for new operations which are:
	// 	   START code -> S
  	//     STOP code  -> E	
	// 	   VEL code   -> V50.0
	// NOTE: parse float is blocking, we'll have to see how messy that is.
	// AND: assumes that the end of a line has a line ending character
	char input = Serial.peek();
	if (Serial.available()>0){
		switch(input){
			case 'S': case 's':
				// start the program
				Serial.read();
				Serial.println("start running!");
				run = 1;
				break;
			case 'E': case 'e':
				// end the program
				Serial.read();
				Serial.println("stop running!");
				run = 0;
				break;
			case 'V': case 'v':
				// change the linear velocity
				Serial.read();
				float vel = Serial.parseFloat();
				Serial.println("setting linear velocity");
				motor_speed.linvel = vel;
				break;
			default:
				// clears buffer
				while(Serial.available()){
					Serial.read();
					delay(10);
				}	
		}		
	}
	
	// if (Serial.available() > 0) {
	// 	if (Serial.peek() == 'S') {
	// 			// start the program
	// 			Serial.read();
	// 			Serial.println("start pid loop");
	// 			run = 1;
	// 	} else if (Serial.peek() == 'E') {
	// 			// end the program
	// 			Serial.read();
	// 			Serial.println("ending the pid loop");
	// 			run = 0;
	// 	} else if (Serial.peek() == 'V') {
	// 			// read new derivative value
	// 			Serial.read();
	// 			float vel = Serial.parseFloat();
	// 			Serial.println("setting speed constant");
	// 			motor_speed.linvel = vel;
	// 	} else {
	// 		while(Serial.available()) {
	// 			Serial.read();
	// 			delay(10);
	// 		}
	// 	}
	// 	Serial.print("I'm ready!");
	// }
	
}

void setup() {
		// setup the motor shield controller
		AFMS.begin();
		send_motor_cmd(0, 0, 0, 0);
		Serial.begin(115200);
		delay(10);
}

void loop() {
		//print out the sensor reading to the serial
		// Serial.println(sensor_test());
		// delay(1);

		// respond to serial operations
		serialReader();

		if (run == 1) {
			// // check if system should run based on serial input

			// // perform PID calculations for the current step
			//pid.loopStep(leftRead, rightRead, &motor_speed);

			// // update motor speeds to the motor controller
			drive_all(motor_speed.linvel);

			// // log values
			// // LOG,time,left,right,sensor_left, sensor_right
			// Serial.print("LOG,Motors,");
			// Serial.print(String(motor_speed.left));
			// Serial.print(",");
			// Serial.print(String(motor_speed.right));
			// Serial.print(",");
			// Serial.print(String(run));
			// Serial.print(",");
			// Serial.println(String(millis()));
		} else {
			drive_all(0);
		}
}

