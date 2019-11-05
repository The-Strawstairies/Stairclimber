#include "pid.h" // header in local directory
#include "Arduino.h"
#include "Adafruit_MotorShield.h"
#include "Wire.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"
//#include "drivetrain.h"
#include "main.h"

pid::pid(float setPoint_new, float Kp_new, float Ki_new, float Kd_new) {
		// construct!
		// PID_Controller PID = PID_Controller(5.0, 6.0, 6.0, 6.0)

		setPoint = setPoint_new;
		Kp = Kp_new;
		Ki = Ki_new;
		Kd = Kd_new;
		lastRun = micros();
};


void pid::setSetpoint(float setPoint_new) {
		// change the set point
		setPoint = setPoint_new;
}


void pid::setKp(float Kp_new) {
		// change the P constants used in the live loop
		Kp = Kp_new;
}


void pid::setKi(float Ki_new) {
		// change the I constants used in the live loop
		Ki = Ki_new;
}


void pid::setKd(float Kd_new) {
		// change the D constants used in the live loop
		Kd = Kd_new;
}

void pid::loopStep(double curr_angle) {
		// Takes in the angular velocity about the x axis

		// timing math
		unsigned long now = micros();
		unsigned long dt = (now - lastRun)/1000000.0;
		lastRun = now;

		// run the loop
		// 	error = setpoint - angle (deg/sec * time interval (sec))
		error = setPoint - (curr_angle);
		integral = integral + (error * dt);
		derivative = (error - previousError) / dt;

		output = (Kp * error) + (Ki * integral) + (Kd * derivative);
		// 	previous_error = error
		previousError = error;

};
