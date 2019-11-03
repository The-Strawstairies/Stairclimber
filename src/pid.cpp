#include "pid.h" // header in local directory
#include "Arduino.h"
#include "Adafruit_MotorShield.h"
#include "Wire.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "drivetrain.h"

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


void pid::setSpeed(float sd_new) {
		// change the D constants used in the live loop
	  linVel = sd_new;
}


void pid::loopStep(float leftSensor, float rightSensor, Speeds *motor_speed) {
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
