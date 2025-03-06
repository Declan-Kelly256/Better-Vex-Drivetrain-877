#include "vex.h"
#include <iostream>
#include <string>
#include <sstream>
#include "Pid.h"
#pragma once 
using namespace vex;

PID::PID(float p, float i, float d, double error){
    
            kP = p;
            kI = i;
            kD = d;

            err = error; 
            errLast = err;
            integral = 0; 
            interval = 1; 


}
PID::PID(float p, float i, float d, double error, double loopInterval){
    
            kP = p;
            kI = i;
            kD = d;

            err = error; 
            errLast = err;
            integral = 0; 
            interval = loopInterval; 

}


PID::PID(float p, float i, float d){

            kP = p;
            kI = i;
            kD = d;

            err = 0; 
            errLast = err;
            integral = 0; 
            interval = 1;
}

double PID::getPwr(double error){
    errLast = err; 
    err = error; 
    integral += err * interval; 

    double dTerm = (err - errLast)/integral;
    //     P          I               D
    return err * kP + integral * kI + dTerm * kD; 
}

void PID::setKP(float p)
    kP = p;
void PID::setKI(float i);
    kI = i;
void PID::setKD(float d); 
    kD =  d; 

void PID::refreshError(double error){
    integral = 0;
    err = error;
    errLast = err;  
}
void PID::refreshError(double error,bool resetIntegral){
    if(resetIntegral){
        integral = 0;
    }
    err = error;
    errLast = err;  
}



