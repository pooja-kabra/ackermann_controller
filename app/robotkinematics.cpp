#include "../include/robotkinematics.hpp"

double ackermann::RobotKinamatics::getTrackLength(){
    return track_length_;
}

double ackermann::RobotKinamatics::getWheelBase(){
    return wheel_base_;
}

double ackermann::RobotKinamatics::getWheelRadius(){
    return wheel_radius_;
}

double ackermann::RobotKinamatics::getTuringRadius(){
    return turning_radius_;
}

double ackermann::RobotKinamatics::getInnerWheelHeading(){
    return inner_wheel_heading_;
}

double ackermann::RobotKinamatics::getOutterWheelHeading(){
    return outter_wheel_heading_;
}

double ackermann::RobotKinamatics::getOutterWheelSpeed(){
    return outter_wheel_speed_;
}

double ackermann::RobotKinamatics::getInnerWheelSpped(){
    return inner_wheel_speed_;
}

double ackermann::RobotKinamatics::getTheataMax(){
    return theata_max_;
}
