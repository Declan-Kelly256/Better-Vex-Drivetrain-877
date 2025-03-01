#include "vex.h"
#include <iostream>
#include <string>
#include <sstream>

#include "position.h"

Position::Position(){
        x = 0;
y = 0;
theta = pi/2; 
};
Position::Position(double xVal, double yVal){
    x = xVal;
    y = yVal;
    theta = pi/2; 
};
Position::Position(double xVal, double yVal,double dir){
    x = xVal;
    y = yVal;
    theta = dir; 
};
double Position::getX(){
    return x;
};
double Position::getY(){
    return y;
};
double Position::getTheta(){
    return theta;
};
double Position::setX(double newX){
    x = newX;
    return x;
};
double Position::setY(double newY){
    y = newY;
    return y;
};
double Position::setTheta(double newTheta){
    theta = newTheta;
    return theta;
};

void Position::setPosition(Position p){
    x = p.getX();
    y = p.getY();
    theta = p.getTheta();
};
   
