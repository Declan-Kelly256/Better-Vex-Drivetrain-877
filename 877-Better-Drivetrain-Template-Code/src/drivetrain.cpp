#include "vex.h"
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include "odom.h"
#include "drivetrain.h"
using namespace vex;

customDrivetrain::customDrivetrain(vex::motor& l1, vex::motor& l2,vex::motor& l3, vex::motor& r1, vex::motor& r2, vex::motor& r3, vex::rotation odom1, vex::rotation odom2, vex::rotation odom3, double off1, double off2, double off3 ):
    left1(l1),
    left2(l2), 
    left3(l3),
    right1(r1),
    right2(r2),
    right3(r3),
    o1(odom1),
    o2(odom2),
    o3(odom3), 
    offset1(off1),
    offset2(off2),
    offset3(off3),
    odomPosition(off1, off2, off3)
    {
    
    }
//sorry <3

void customDrivetrain :: driveRight (double pwr)
{
    right1.spin(forward,pwr,percent);
    right2.spin(forward,pwr,percent);
    right3.spin(forward,pwr,percent);
}

void customDrivetrain :: driveLeft(double pwr)
{
    left1.spin(forward,pwr,percent);
    left2.spin(forward,pwr,percent);
    left3.spin(forward,pwr,percent);
}

void customDrivetrain :: driveVelocityInput(double speed)
{
    right1.spin(forward,speed,rpm);
    right2.spin(forward,speed,rpm);
    right3.spin(forward,speed,rpm);
    left1.spin(forward,speed,rpm);
    left2.spin(forward,speed,rpm);
    left3.spin(forward,speed,rpm);
}

void customDrivetrain :: spinVelocityInput(double speed)
{
    right1.spin(forward,speed,rpm);
    right2.spin(forward,speed,rpm);
    right3.spin(forward,speed,rpm);
    left1.spin(reverse,speed,rpm);
    left2.spin(reverse,speed,rpm);
    left3.spin(reverse,speed,rpm);
}

void customDrivetrain :: stopCoast ()
{
    right1.setStopping(coast);
    right2.setStopping(coast);
    right3.setStopping(coast);
    
    left1.setStopping(coast);
    left2.setStopping(coast);
    left3.setStopping(coast);

    right1.stop();
    right2.stop();
    right3.stop();

    left1.stop();
    left2.stop();
    left3.stop();
}

void customDrivetrain::arcadeDrive(double forwardAxis, double turnAxis, float deadzone, float fwdConst, float trnConst){
    if(fabs(forwardAxis) < deadzone)
        forwardAxis = 0;
    if(fabs(turnAxis) < deadzone)
        turnAxis = 0; 
    forwardAxis *= fwdConst;
    turnAxis *= trnConst; 
    driveLeft( forwardAxis - turnAxis);
    driveRight( forwardAxis + turnAxis);
}

void customDrivetrain::arcadeDrive(double forwardAxis, double turnAxis, float deadzone){
    arcadeDrive(forwardAxis , turnAxis, deadzone, 1, 1); 
}


double customDrivetrain::P(double err, double K){
    return err*K;
}
double customDrivetrain::I(double & integral ,double err, double deltaT, double K, bool turn){
    if(err <= 0)
        integral =0;
    if( (!turn&&(err > 40)) || (turn && (err > 2*pi))){
        integral = 0; 
    }                   
    integral += err;
    return integral*K;
}
double customDrivetrain::D(double err, double errLast, double deltaT, double K){
    return ((err-errLast)/deltaT) * K;
}

double customDrivetrain::LinearPID(double kP, double kI, double kD, double maxPowerPercent, double distance){
    double error = distance; 
    double Integral = 0; 
    double errorLast = distance;
    int deltaTime = 10;
    double wheelCircumfrence= 2*pi*1.375; 
    int errorStableCount = 0;
    o1.setPosition(0,turns); 
    for(int time = 0; errorStableCount < 5; time += deltaTime ){// temp, sets 1.5s timeout 

        error -= -o1.position(turns)*wheelCircumfrence; // assumes travel in the right direction 
        o1.setPosition(0,turns);
        
        double power = P(error, kP) + I(Integral , error, deltaTime , kI, false) + D(error,errorLast,deltaTime,kD); 

        if(fabs(error) < 1)
            errorStableCount++;
        else    
            errorStableCount = 0;

        errorLast = error; 

        driveLeft(maxPowerPercent*power); 
        driveRight(maxPowerPercent*power);

        wait(deltaTime, msec);
    }
    driveLeft(0); 
    driveRight(0);

    return error;


}
double customDrivetrain::turnPID(double kP, double kI, double kD, double maxPowerPercent, double distance){
    distance *= pi/180; // convert from degrees to radians 

    double error = distance; 
    double Integral = 0; 
    double errorLast = distance;
    int deltaTime = 10;// loop itre
    double wheelCircumfrence= 2*pi*1.36924; 
    Odom odom = Odom(1.5625,2.3125,1.4375,0,0,0);
    o1.setPosition(.5 , turns); 
    o2.setPosition(.5 , turns);
    o3.setPosition(.5 , turns);
    int errorStableCount = 0;
    for(int time = 0; errorStableCount < 10; time += deltaTime ){// temp, sets 10s timeout 
        double s1 = (o1.position(turns) - .5) * wheelCircumfrence; 
        double s2 = (o2.position(turns) - .5) * wheelCircumfrence;
        double s3 = (o3.position(turns) - .5) * wheelCircumfrence;

        odom.newPosition(s1,s2,s3);

        error = distance - odom.getTheta(); 
        o1.setPosition(.5 , turns); 
        o2.setPosition(.5 , turns);
        o3.setPosition(.5 , turns);

        
        
        double power = P(error, kP) + I(Integral , error, deltaTime , kI, true) + D(error,errorLast,deltaTime,kD); 

        if(fabs(error) < .05)
            errorStableCount++;
        else    
            errorStableCount = 0;

        
        errorLast = error; 

        driveLeft(maxPowerPercent*power); 
        driveRight(-maxPowerPercent*power);
        wait(deltaTime, msec);
    }
    driveLeft(0); 
    driveRight(0);

    return error*180/pi;


}