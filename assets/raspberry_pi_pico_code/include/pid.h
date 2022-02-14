#ifndef _PID_H_
#define _PID_H_

#include "pico/stdlib.h"

#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1
#define P_ON_M 0
#define P_ON_E 1

class PID
{
    public:
        PID(double*, double*, double*, double, double, double, int, int);
        PID(double*, double*, double*, double, double, double, int);
        void SetMode(int Mode);
        bool Compute();
        void SetOutputLimits(double, double);
        void SetTunings(double, double, double);
        void SetTunings(double, double, double, int);
        void SetControllerDirection(int);
        void SetSampleTime(int);
        double GetKp();
        double GetKi();
        double GetKd();
        int GetMode();
        int GetDirection();

    private:
        void Initialize();
        double dispKp;
        double dispKi;
        double dispKd;
        double kp;
        double ki;
        double kd;
        int controllerDirection;
        int pOn;
        double *myInput;
        double *myOutput;
        double *mySetpoint;
        uint32_t lastTime;
        double outputSum, lastInput;
        uint32_t SampleTime;
        double outMin, outMax;
        bool inAuto, pOnE;
};

#endif