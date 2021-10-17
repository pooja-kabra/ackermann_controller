#include "../include/robotkinematics.hpp"

double ackermann::RobotKinematics::getTrackLength(){
    return track_length_;
}

double ackermann::RobotKinematics::getWheelBase(){
    return wheel_base_;
}

double ackermann::RobotKinematics::getWheelRadius(){
    return wheel_radius_;
}

double ackermann::RobotKinematics::getTuringRadius(){
    return turning_radius_;
}

double ackermann::RobotKinematics::getInnerWheelHeading(){
    return inner_wheel_heading_;
}

double ackermann::RobotKinematics::getOutterWheelHeading(){
    return outter_wheel_heading_;
}

double ackermann::RobotKinematics::getOutterWheelSpeed(){
    return outter_wheel_speed_;
}

double ackermann::RobotKinematics::getInnerWheelSpeed(){
    return inner_wheel_speed_;
}

double ackermann::RobotKinematics::getTheataMax(){
    return theata_max_;
}

void ackermann::RobotKinematics::setInnerWheelHeading(double inner_wheel_heading){
    inner_wheel_heading_ = inner_wheel_heading;
}

void ackermann::RobotKinematics::setOutterWheelHeading(double outter_wheel_heading){
    outter_wheel_heading_ = outter_wheel_heading;
}

void ackermann::RobotKinematics::setInnerWheelSpeed(double inner_wheel_speed){
    inner_wheel_speed_ = inner_wheel_speed;
}

void ackermann::RobotKinematics::setOutterWheelSpeed(double outter_wheel_speed){
    outter_wheel_speed_ = outter_wheel_speed;
}