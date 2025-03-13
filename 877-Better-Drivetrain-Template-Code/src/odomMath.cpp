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
    //exception -- prevents roundoff errors 
    if( fabs(deltaLeft-deltaRight) < .0001 ){
        return 0; 
    }

    return (deltaLeft - deltaRight)/(leftOffset+rightOffset) ;
}

double Odom::calculateY(double deltaRight,double deltaTheta){

    return 2 * sin(deltaTheta/2)*( ( deltaRight / deltaTheta) + rightOffset) ; 

}

double Odom::calculateX(double deltaBack,double deltaTheta){
    return 2*sin(deltaTheta/2) * ( (deltaBack / deltaTheta) + backOffset ); 
}

Position Odom::coordinateRotator( Position& p, double radianRotater){

    //convert to polar 
    
    double radius = sqrt(p.getX()*p.getX()+ p.getY()+p.getY());
    double polarTheta;
    // y axis exceptions for arctan domain 
    if( p.getX() == 0){
        if(p.getY() >= 0)
            polarTheta = pi/2;
        else    
            polarTheta = 3*pi/2; 
    }else{
        polarTheta = atan( p.getY() / p.getX());
    }
    // converts to quadrent II or III for angles in interval (pi/2 , 3pi/2)
    if (p.getX() < 0){
        radius *= -1;
    }

    //rotate by the correct amount

    p.setX( radius*cos(polarTheta + radianRotater)) ; 
    p.setY( radius*sin(polarTheta + radianRotater)) ;

    /*
    please note that p.theta is not touched. for our purposes, the rotational offset of the local plane only 
    affects x,y positon the arc angle measurement is an effective measurement of change in orientation without
    processing. 
    */

    return p; // returns modified position 
}

Position Odom::updatePosition(){
    if (threeWheel){
        return updatePositionThreeWheel();
    }else {
        return updatePositionIMU();
    }
}

Position Odom::updatePositionThreeWheel(){

    double sensChangeLeft = leftWheel.position(turns) * 2 * pi * wheelRadius  - sensorLastLeft;  
    double sensChangeRight = rightWheel.position(turns) * 2 * pi * wheelRadius - sensorLastRight;
    double sensChangeBack = backWheel.position(turns) * 2 * pi * wheelRadius -sensorLastBack;

    //now done with last loop sensor Values , reset for next Loop 
    sensorLastLeft = sensChangeLeft;
    sensorLastRight = sensChangeRight;
    sensorLastBack = sensChangeBack; 

    Position localPlane = Position();
    localPlane.setTheta(calculateDeltaTheta(sensChangeLeft , sensChangeLeft )); 
    localPlane.setX( calculateX( sensChangeBack, localPlane.getTheta() ) ); 
    localPlane.setY( calculateY( sensChangeRight , localPlane.getTheta() ) ); 

    //made it pass by reference, so localPlane is properly rotated w/o more function calls
    coordinateRotator(localPlane, -(localPlane.getTheta() * .5  + theta)); 

    //we can now add this to the current position and it should be updated 

    x += localPlane.getX(); 
    y += localPlane.getY(); 
    theta += localPlane.getTheta();

    return  Position(x , y, theta); //small blunder, but i think i cant return the inherited object 
    
}

Position Odom::updatePositionIMU(){
    //
    double sensChangeRight = rightWheel.position(turns) * 2 * pi * wheelRadius - sensorLastRight;
    double sensChangeBack = backWheel.position(turns) * 2 * pi * wheelRadius -sensorLastBack;

    sensorLastRight = sensChangeRight;
    sensorLastBack = sensChangeBack; 

    

}


