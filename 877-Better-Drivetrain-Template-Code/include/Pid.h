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

    public: 
        PID(float p, float i, float d, double error);
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

        void setKP(float p); 
        void setKI(float i);
        void setKD(float d); 

        void setErorr(double err); // for foolProofing, integalReset is true by default 
        void setErorr(double err, bool integralReset);




};
