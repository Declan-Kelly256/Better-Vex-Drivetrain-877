#include "vex.h"
#include <iostream>
#include <string>
#include <sstream>
#include "position.h"
#include "odomMath.h"
using namespace vex;

Odom::Odom(vex::rotation& leftSensor, vex::rotation& rightSensor, vex::rotation& rearSensor, double offsetLeft, double offsetRight, double offsetRear,double radiusWheel): 
leftWheel(leftSensor),
rightWheel(rightSensor),
backWheel(rearSensor),
IMU(vex::inertial(PORT21)),// to satisfy object creation. thanks to bool threeWheel, it will not be used NOTE: may cause problems if there is something on port21 (but hopefully not)
leftOffset(offsetLeft),
rightOffset(offsetRight),
backOffset(offsetRear),
wheelRadius(radiusWheel)
{
    threeWheel = true; 
    sensorLastLeft = leftWheel.position(turns)*2*pi*wheelRadius; 
    sensorLastRight = rightWheel.position(turns)*2*pi*wheelRadius; 
    sensorLastBack = backWheel.position(turns)*2*pi*wheelRadius; 
}

Odom::Odom(vex::rotation& forwardSensor, vex::rotation& rearSensor, vex::inertial& inertialSensor, double offsetForward, double offsetRear,double radiusWheel):
leftWheel(forwardSensor),// to satisfy object creation. thanks to bool threeWheel, it will not be used 
rightWheel(forwardSensor),
backWheel(rearSensor),
IMU(inertialSensor),
leftOffset(offsetForward),
rightOffset(offsetForward),
backOffset(offsetRear),
wheelRadius(radiusWheel)
{
    threeWheel = false;
    sensorLastLeft = leftWheel.position(turns)*2*pi*wheelRadius; 
    sensorLastRight = rightWheel.position(turns)*2*pi*wheelRadius; 
    sensorLastBack = 0; 
    sensorLastIMU = IMU.rotation(turns) * 2 * pi ;
}

double Odom::calculateDeltaTheta(double deltaLeft, double deltaRight){
    return (deltaLeft - deltaRight)/(leftOffset+rightOffset) ;
}

double Odom::calculateY(double deltaRight,double deltaTheta){

    return 2 * sin(deltaTheta/2)*( ( deltaRight / deltaTheta) + rightOffset) ; 

}

double Odom::calculateX(double deltaBack,double deltaTheta){
    return 2*sin(deltaTheta/2) * ( (deltaBack / deltaTheta) + backOffset ); 
}

Position Odom::coordinateRotator( Position p, double rotater){

    //convert to polar 
    
    double radius = sqrt(p.getX()*p.getX()+ p.getY()+p.getY());
    double polarTheta;
    if( p.getX() == 0){
        if(p.getY() >= 0)
            polarTheta = pi/2;
        else    
            polarTheta = 3*pi/2; 
    }else{
        polarTheta = atan( p.getY() / p.getX());
    }
    if (p.getX() < 0){
        radius *= -1;
    }

    p.setX( radius*cos(polarTheta + rotater)) ; 
    p.setY( radius*sin(polarTheta + rotater)) ;
    





}

Position Odom::updatePosition(){
    if (threeWheel){
        return updatePositionThreeWheel();
    }else {
        return updatePositionIMU();
    }
}

Position Odom::updatePositionThreeWheel(){

}
