#include "vex.h"
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include "odom.h"
class customDrivetrain {
    private:
        //left
        std::vector<vex::motor> leftWell; 
        //right
        std::vector<vex::motor> rightWell; 

        //parallel odom 

        rotation o1;
        rotation o2; 

        // perpendicular odom 

        rotation o3; 

        // odom offsets

        double offset1;
        double offset2;
        double offset3; 

        const double pi = 3.14159; 
        public:



        Odom odomPosition; 

        

        //dear god im so sorry about these constructors 
        customDrivetrain(vex::motor& l1, vex::motor& l2,vex::motor& l3, vex::motor& r1, vex::motor& r2, vex::motor& r3, vex::rotation odom1, vex::rotation odom2, vex::rotation odom3, double off1, double off2, double off3 );

        void driveLeft(double power);// moves the left motors at the given power
        void driveRight(double power);//moves the right motors at the given power

        void driveVelocityInput(double speed);// moves forward at the given speed

        void spinVelocityInput(double speed);// spins at the given speed

        void stopCoast();// stops on coast mode 



        void arcadeDrive(double forwardAxis, double turnAxis, float deadzone , float fwdConst, float trnConst); // same as below, but you can change the sensitivities of the forward and turn axises 
        void arcadeDrive(double forwardAxis, double turnAxis, float deadzone); // moves the motors based off the the axis inputs, and rules out the deadzone

        double P(double err, double K);//calculates PID proportional term depending on the input 
        double I(double & integral ,double err, double deltaT, double K, bool turn); // calculates PID intergral term depending on the input
        double D(double err, double errLast, double deltaT, double K);  // calculates PID derivative term depending on the input

        double LinearPID(double kP, double kI, double kD, double maxPowerPercent, double distance); // uses the pid constants to move the target distance, positive is in the clamp direction

        double turnPID(double kP, double kI, double kD, double maxPowerPercent, double distance); // uses the pid constants to spin the target distance, positive is counterclockwise from top view

        



};