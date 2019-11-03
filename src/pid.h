#ifndef PID_H // include guard
#define PID_H

#include "drivetrain.h"


    class pid {
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
    pid(float setPoint, float Kp, float Ki, float Kd);

    // empty constructor for initialization
    void setSetpoint(float setPoint);
    void setKp(float Kp);
    void setKi(float Ki);
    void setKd(float Kd);
    void setSpeed(float sd_new);

    // Defines a function that steps the motor speeds based on PID
    void loopStep(float leftSensor, float rightSensor, Speeds *motor_speed);
};

#endif