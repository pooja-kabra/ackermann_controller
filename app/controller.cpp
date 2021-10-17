#include "../include/controller.hpp"


// need to write solve method


void ackermann::Controller::setGoalHeading(double goal_heading) {
    goal_heading_ = goal_heading;
}

void ackermann::Controller::setGoalSpeed(double goal_speed) {
    goal_speed_ = goal_speed;
}

double ackermann::Controller::getGoalHeading(){
    return goal_heading_;
}

double ackermann::Controller::getGoalSpeed(){
    return goal_speed_;
}

double ackermann::Controller::getKi(){
    return ki_;
}

double ackermann::Controller::getKp(){
    return kp_;
}