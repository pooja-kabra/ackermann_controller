/**
 * @file inversekinematics.cpp
 * @author Markose Jacob, Pooja Kabra
 * @brief This file defines the Inverse Kinematics class, it calculates 
 *        individual wheel angular speeds and wheel angles w.r.t. robot axisIt provides functions    
 * @version 0.1
 * @date 2021-10-16
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "../include/inversekinematics.hpp"
#include <math.h>
#define PI 3.14159265



ackermann::InverseKinematics::headings
ackermann::InverseKinematics::calculateWheelHeadings(double actual_heading, double actual_speed, double dt, char direction, Robot& car) {
    double inner=0,outer=0;
    // setRobot(car);
    // std::cout<<"Actual heading ::::::: "<<actual_heading<<std::endl;
    if (direction = 'l'){
        //inner wheel heading
        inner = atan2(2*car.getWheelBase()*sin(actual_heading*PI/180), 2*car.getWheelBase()*cos(actual_heading*PI/180) - car.getTrackLength()*sin(actual_heading*PI/180));
        inner = inner*180/PI;

        //outer wheel heading
        outer = atan2(2*car.getWheelBase()*sin(actual_heading*PI/180), 2*car.getWheelBase()*cos(actual_heading*PI/180) + car.getTrackLength()*sin(actual_heading*PI/180));
        outer = outer*180/PI;


        // std::cout<<"Inner heading : "<<inner<<std::endl;
        // std::cout<<"compared againt : "<<car.getInsMaxRot()*dt<<std::endl;
        if(inner > car.getThetaIncrPerSecMax()*dt ){
            inner = car.getThetaIncrPerSecMax()*dt;
            actual_heading = ((PI/2) - atan2((1/tan(inner * (PI/180)) + (car.getTrackLength()/2*car.getWheelBase())), 1)) * 180/PI;
            // std::cout<<"actual heading : "<<actual_heading<<std::endl;
            outer = atan2(2*car.getWheelBase()*sin(actual_heading*PI/180), 2*car.getWheelBase()*cos(actual_heading*PI/180) + car.getTrackLength()*sin(actual_heading*PI/180));
            outer = outer*180/PI;
            // std::cout<<"New outer heading : "<<outer<<std::endl;
        }
    }
    
    else if (direction = 'r'){
        //inner wheel heading
        inner = atan2(2*car.getWheelBase()*sin(actual_heading*PI/180), 2*car.getWheelBase()*cos(actual_heading*PI/180) + car.getTrackLength()*sin(actual_heading*PI/180));
        inner = inner*180/PI;

        //outer wheel heading
        outer = atan2(2*car.getWheelBase()*sin(actual_heading*PI/180), 2*car.getWheelBase()*cos(actual_heading*PI/180) - car.getTrackLength()*sin(actual_heading*PI/180));
        outer = outer*180/PI;


        // std::cout<<"outer heading : "<<outer<<std::endl;
        // std::cout<<"compared againt : "<<car.getInsMaxRot()*dt<<std::endl;
        if(outer > car.getThetaIncrPerSecMax()*dt ){
            outer = car.getThetaIncrPerSecMax()*dt;
            actual_heading = ((PI/2) - atan2((1/tan(inner * (PI/180)) - (car.getTrackLength()/2*car.getWheelBase())), 1)) * 180/PI;
            // std::cout<<"actual heading : "<<actual_heading<<std::endl;
            inner = atan2(2*car.getWheelBase()*sin(actual_heading*PI/180), 2*car.getWheelBase()*cos(actual_heading*PI/180) - car.getTrackLength()*sin(actual_heading*PI/180));
            inner = outer*180/PI;
            // std::cout<<"New inner heading : "<<inner<<std::endl;
        }
    }

    else{
        inner = 0;
        outer = 0;
    }

    car.setInnerWheelHeading(car.getInnerWheelHeading()+inner);
    car.setOuterWheelHeading(car.getOuterWheelHeading()+outer);

    std::cout<<"inner wheel heading : "<<car.getInnerWheelHeading()<<std::endl;
    std::cout<<"outer wheel heading : "<<car.getOuterWheelHeading()<<std::endl;
    headings head;
    head.inner = inner;
    head.outer = outer;
    return head;

}


ackermann::InverseKinematics::speed
ackermann::InverseKinematics::calculateWheelSpeeds(double actual_heading, double actual_speed, double dt, char direction, Robot& car) {
    double r=0, angular_speed =0, iws=0, ows = 0, iww = 0, oww = 0;
    double rps_max = dt * car.getRpsMax();
    double wheel_r = car.getWheelRadius();
    // std::cout<<"rps_max   : "<<rps_max<<std::endl;

    // Turning radius
    r = (car.getWheelBase()/sin(actual_heading*PI/180));
    // std::cout<<"Get wheel base : "<<car.getWheelBase()<<std::endl;
	// std::cout<<"Heading : "<<actual_heading<<std::endl;
    // std::cout<<"turning radius : "<<r<<std::endl;
    angular_speed = actual_speed/r;
    // std::cout<<"angular speed : "<<angular_speed<<std::endl;
    iws = angular_speed * (car.getWheelBase()/(sin(car.getInnerWheelHeading()*PI/180)));
    ows = angular_speed * (car.getWheelBase()/(sin(car.getOuterWheelHeading()*PI/180)));
    // std::cout << "Inner wheel speed : "<<iws<<std::endl;
    // std::cout << "Outer wheel speed : "<<ows<<std::endl;

    iww = iws / wheel_r;  // angular speed in rps for left wheel
    oww = ows / wheel_r;  // angular speed in rps for right wheel
    // std::cout <<"Turn::: " << direction <<std::endl;
    if(direction == 'r'){
        // std::cout <<"Turn is r" <<std::endl;
        // std::cout<<"anglar speed ::: "<<angular_speed<<std::endl;
        // std::cout<<"wheel base ::: "<<car.getWheelBase()<<std::endl;
        // std::cout<<"Inner wheel heading ::: "<<car.getInnerWheelHeading()<<std::endl;
        // std::cout<<"Outer wheel heading ::: "<<car.getOuterWheelHeading()<<std::endl;

        if (iww > rps_max) {
            iww = rps_max;
            iws = rps_max*wheel_r;
            angular_speed = iws/(car.getWheelBase()/(sin(car.getInnerWheelHeading()*PI/180)));
            ows = angular_speed * (car.getWheelBase()/(sin(car.getOuterWheelHeading()*PI/180)));
            oww = oww = ows / wheel_r;
        }
    } 
    else if (direction == 'l') {
        // std::cout <<"Turn is l" <<std::endl;
        if (oww > rps_max) {
            oww = rps_max;
            ows = rps_max*wheel_r;
            angular_speed = ows/(car.getWheelBase()/(sin(car.getOuterWheelHeading()*PI/180)));
            iws = angular_speed * (car.getWheelBase()/(sin(car.getInnerWheelHeading()*PI/180)));
            iww = iws / wheel_r;
        }

    }

    else{
        // std::cout <<"Going straight" <<std::endl;
        if(angular_speed > rps_max)
            angular_speed = rps_max;
            actual_speed = angular_speed*r;
        
        oww = angular_speed;
        iww = angular_speed;
        iws = actual_speed;
        ows = actual_speed;
    }



    car.setInnerWheelRps(car.getInnerWheelRps() + iww);
    car.setOuterWheelRps(car.getOuterWheelRps() + oww);
    car.setInnerWheelSpeed(car.getInnerWheelSpeed() + iws);
    car.setOuterWheelSpeed(car.getOuterWheelSpeed() + ows);

    std::cout <<"Inner linear speed ::: " << car.getInnerWheelSpeed() << std::endl;
    std::cout <<"Outer linear speed ::: " << car.getOuterWheelSpeed() << std::endl;

    speed spd;
    spd.inner_speed = iws;
    spd.outer_speed = ows;

    return spd;

    }

void ackermann::InverseKinematics::calculateNewRobotHeadingandSpeed(double inner_heading_incr,
double outer_heading_incr, double inner_speed_incr, double outer_speed_incr, Robot& car, Sensor& sensor) {
    double xl_curr = 0, yl_curr = 0, xr_curr = 0, yr_curr = 0, xl_next = 0, yl_next = 0, xr_next = 0, yr_next = 0;
}