#ifndef PID_H // include guard
#define PID_H

    class pid {
    public:
        // error tracking
        long previousError = 0;
        long error = 0;

        // track prior deriv and int
        long integral = 0;
        long derivative = 0;

        // prior output
        long output = 0;

        // tuning params
        float setPoint;
        float Kp;
        float Ki;
        float Kd;

    // last micros reading
    unsigned long lastRun;

    // initialization function for tuning PID
    pid(float setPoint, float Kp, float Ki, float Kd);

    // empty constructor for initialization
    void setSetpoint(float setPoint);
    void setKp(float Kp);
    void setKi(float Ki);
    void setKd(float Kd);

    // Defines a function that steps the motor speeds based on PID
    void loopStep(double x);
};

#endif