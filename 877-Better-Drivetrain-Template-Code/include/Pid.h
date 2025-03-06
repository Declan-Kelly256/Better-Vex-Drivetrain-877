#include "vex.h"
#include <iostream>
#include <string>
#include <sstream>
#pragma once 
using namespace vex;


class PID {
    private:
    //constant vars for tuning
        float kP;
        float kI;
        float kD;
    
        //variables responsible for calculating stuff 

        double err;
        double errLast; 
        double integral; 

        double interval; 

    public: 
        PID(float p, float i, float d, double error);
        PID(float p, float i, float d, double error, double loopInterval);
        // 
        PID(float p, float i, float d);
        // {
        //     kP = p;
        //     kI = i;
        //     kD = d;

        //     err = 0; 
        //     errLast = err;
        //     integral = 0;
        // }

        
        double getPwr(double error);

        double getError();

        void setKP(float p); 
        void setKI(float i);
        void setKD(float d); 

        void refreshError(double error); 

        void refreshError(double error , bool resetIntegral)



};
