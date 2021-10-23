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
#include <cmath>
#define PI 3.14159265


void ackermann::InverseKinematics::calculateWheelHeadings(double actual_heading, double actual_speed, double dt, Robot& car) {
    double inner=0,outer=0;
    setRobot(car);
    // std::cout<<"Actual heading ::::::: "<<actual_heading<<std::endl;
    
    //inner wheel heading
    inner = atan2(2*car.getWheelBase()*sin(actual_heading*PI/180), 2*car.getWheelBase()*cos(actual_heading*PI/180) - car.getTrackLength()*sin(actual_heading*PI/180));
    inner = inner*180/PI;
    if(inner > car.getThetaMax()){
        car.setInnerWheelHeading(45);
    }
    else {
        car.setInnerWheelHeading(inner);
    }

    std::cout<<"inner wheel heading : "<<car.getInnerWheelHeading()<<std::endl;

    //outer wheel heading
    outer = atan2(2*car.getWheelBase()*sin(actual_heading*PI/180), 2*car.getWheelBase()*cos(actual_heading*PI/180) + car.getTrackLength()*sin(actual_heading*PI/180));
    outer = outer*180/PI;
    if(outer > car.getThetaMax()){
        car.setOuterWheelHeading(45);
    }
    else {
        car.setOuterWheelHeading(outer);
    }
    std::cout<<"outer wheel heading : "<<car.getOuterWheelHeading()<<std::endl;
}



void ackermann::InverseKinematics::calculateWheelSpeeds(double actual_heading, double actual_speed, double dt, Robot& car) {
    double r=0, angular_speed =0, lws=0, rws = 0;

    //Angular speed
    r = (car.getWheelBase()/sin(actual_heading*PI/180));
    // std::cout<<"Get wheel base : "<<car.getWheelBase()<<std::endl;
	// std::cout<<"Heading : "<<actual_heading<<std::endl;
    // std::cout<<"turning radius : "<<r<<std::endl;
    angular_speed = actual_speed/r;
    // std::cout<<"angular speed : "<<angular_speed<<std::endl;
    if(actual_heading == 0){
        // std::cout<<"case 1"<<std::endl;
        car.setInnerWheelSpeed(actual_speed);
        car.setOuterWheelSpeed(actual_speed);
    }
    else if(actual_heading > 0){
        // std::cout<<"case 2"<<std::endl;
        lws = angular_speed * (car.getWheelBase()/(sin(car.getInnerWheelHeading()*PI/180)));
        rws = angular_speed * (car.getWheelBase()/(sin(car.getOuterWheelHeading()*PI/180)));
        // std::cout<<"anglar speed ::: "<<angular_speed<<std::endl;
        // std::cout<<"wheel base ::: "<<car.getWheelBase()<<std::endl;
        // std::cout<<"Inner wheel heading ::: "<<car.getInnerWheelHeading()<<std::endl;
        // std::cout<<"Outer wheel heading ::: "<<car.getOuterWheelHeading()<<std::endl;
        std::cout<<"Inner wheel speed : "<<lws<<std::endl;
        std::cout<<"Outer wheel speed : "<<rws<<std::endl;
        if (lws > car.getSpeedMax()){ //
            car.setInnerWheelSpeed(16.667); // change this later
            if (rws > car.getSpeedMax() ){
                car.setOuterWheelSpeed(16.667);
            }
            else{
                car.setOuterWheelSpeed(rws);
            }
        }
        else if (rws > car.getSpeedMax()){
            car.setOuterWheelSpeed(16.667); //change this
            if (lws > 60){
                car.setInnerWheelSpeed(16.667);
            }
            else{
                car.setInnerWheelSpeed(lws);
            }
        }

        else{
            car.setInnerWheelSpeed(lws);
            car.setOuterWheelSpeed(rws);
        }
    }
    else{
        std::cout<<"case 3"<<std::endl;
        lws = angular_speed * (car.getWheelBase()/(sin(car.getInnerWheelHeading()*PI/180)));
        rws = angular_speed * (car.getWheelBase()/(sin(car.getOuterWheelHeading()*PI/180)));
        if (lws > car.getSpeedMax()){ //
            car.setInnerWheelSpeed(16.667); // change this later
            if (rws > car.getSpeedMax() ){
                car.setOuterWheelSpeed(16.667);
            }
            else{
                car.setOuterWheelSpeed(rws);
            }
        }
        else if (rws > car.getSpeedMax()){
            car.setOuterWheelSpeed(16.667); //change this
            if (lws > car.getSpeedMax()){
                car.setInnerWheelSpeed(16.667);
            }
            else{
                car.setInnerWheelSpeed(lws);
            }
        }

        else{
            car.setInnerWheelSpeed(lws);
            car.setOuterWheelSpeed(rws);
        }
    }
    // std::cout<<"Inner wheel speed : "<<car.getInnerWheelSpeed()<<std::endl;
    // std::cout<<"Outter wheel speed : "<<car.getOuterWheelSpeed()<<std::endl;


}
