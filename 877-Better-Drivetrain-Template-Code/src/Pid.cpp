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
        
}


PID::PID(float p, float i, float d){

            kP = p;
            kI = i;
            kD = d;

            err = 0; 
            errLast = err;
            integral = 0; 
}


