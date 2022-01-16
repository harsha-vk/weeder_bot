/**********************************************************************************************
 * Originally:
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 * This Library is licensed under the MIT License
 **********************************************************************************************/
#include <pid.h>
#include "pico/stdlib.h"

PID::PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int POn, int ControllerDirection)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;
    PID::SetOutputLimits(0, 255);
    SampleTime = 100 * 1000;
    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd, POn);
    lastTime = time_us_32() - SampleTime;
}

PID::PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int ControllerDirection):PID::PID(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection)
{

}

bool PID::Compute()
{
    if(!inAuto) return false;
    uint32_t now = time_us_32();
    uint32_t timeChange = (now - lastTime);
    if(timeChange >= SampleTime)
    {
        double input = *myInput;
        double error = *mySetpoint - input;
        double dInput = (input - lastInput);
        outputSum += (ki * error);
        if(!pOnE) outputSum -= kp * dInput;
        if(outputSum > outMax) outputSum = outMax;
        else if(outputSum < outMin) outputSum = outMin;
        double output;
        if(pOnE) output = kp * error;
        else output = 0;
        output += outputSum - kd * dInput;
        if(output > outMax) output = outMax;
        else if(output < outMin) output = outMin;
	    *myOutput = output;
        lastInput = input;
        lastTime = now;
	    return true;
   }
   else return false;
}

void PID::SetTunings(double Kp, double Ki, double Kd, int POn)
{
    if(Kp<0 || Ki<0 || Kd<0) return;
    pOn = POn;
    pOnE = POn == P_ON_E;
    dispKp = Kp; dispKi = Ki; dispKd = Kd;
    double SampleTimeInSec = ((double)SampleTime) / 1'000'000;
    kp = Kp;
    ki = Ki * SampleTimeInSec;
    kd = Kd / SampleTimeInSec;
    if(controllerDirection == REVERSE)
    {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
    }
}

void PID::SetTunings(double Kp, double Ki, double Kd)
{
    SetTunings(Kp, Ki, Kd, pOn); 
}

void PID::SetSampleTime(int NewSampleTime)
{
    if (NewSampleTime > 0)
    {
        double ratio = (double)NewSampleTime / (double)SampleTime;
        ki *= ratio;
        kd /= ratio;
        SampleTime = (uint32_t)NewSampleTime;
    }
}

void PID::SetOutputLimits(double Min, double Max)
{
    if(Min >= Max) return;
    outMin = Min;
    outMax = Max;
    if(inAuto)
    {
        if(*myOutput > outMax) *myOutput = outMax;
        else if(*myOutput < outMin) *myOutput = outMin;
        if(outputSum > outMax) outputSum= outMax;
        else if(outputSum < outMin) outputSum= outMin;
    }
}

void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {
        PID::Initialize();
    }
    inAuto = newAuto;
}

void PID::Initialize()
{
    outputSum = *myOutput;
    lastInput = *myInput;
    if(outputSum > outMax) outputSum = outMax;
    else if(outputSum < outMin) outputSum = outMin;
}

void PID::SetControllerDirection(int Direction)
{
    if(inAuto && Direction !=controllerDirection)
    {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
    }
    controllerDirection = Direction;
}

double PID::GetKp(){ return  dispKp; }
double PID::GetKi(){ return  dispKi;}
double PID::GetKd(){ return  dispKd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}
