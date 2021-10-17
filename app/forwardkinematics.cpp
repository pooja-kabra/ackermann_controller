#include "../include/forwardkinematics.hpp"


// need to write calculteHeadingError and calculateSpeedError method


void ackermann::ForwardKinematics::setHeadingError(double heading_error) {
    heading_error_ = heading_error;
}

void ackermann::ForwardKinematics::setSpeedError(double speed_error) {
    speed_error_ = speed_error;
}

double ackermann::ForwardKinematics::getHeadingError(){
    return heading_error_;
}

double ackermann::ForwardKinematics::getSpeedError(){
    return speed_error_;
}