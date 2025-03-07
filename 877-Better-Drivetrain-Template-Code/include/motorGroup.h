#include "vex.h"
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#pragma once
using namespace vex; 

class driveMotorGroup(){
    //aight so hear me out. it would be really annoying to rewrite ALL of the vex motor code so we not gonna
    
    public:
    std::vector<motor> group; 

    driveMotorGroup(vex::motor& m1, vex::motor& m2);
    driveMotorGroup(vex::motor& m1, vex::motor& m2, vex::motor& m3);
    driveMotorGroup(std::vector<motor> vec);




    









}