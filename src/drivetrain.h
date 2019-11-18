#ifndef DRIVE_H // include guard
#define DRIVE_H

struct Speeds {
	// struct
	float linvel = 30;
	float lf;
	float rf;
	float lb;
	float rb;
};

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

#endif