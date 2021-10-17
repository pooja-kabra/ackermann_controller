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
