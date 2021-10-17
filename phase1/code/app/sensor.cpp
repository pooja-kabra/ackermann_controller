#include "../include/sensor.hpp"


double ackermann::Sensor::getActualHeading(){
    std::cout<<"getActualHeading called"<<std::endl;
    return actual_heading_;
}

double ackermann::Sensor::getActualSpeed(){
    std::cout<<"getActualSpeed called"<<std::endl;
    return actual_speed_;
}

void ackermann::Sensor::setActualHeading(double actual_heading){
    std::cout<<"setActualHeading called"<<std::endl;
    actual_heading_ = actual_heading;
}

void ackermann::Sensor::setActualSpeed(double actual_speed){
    std::cout<<"setActualSpeed called"<<std::endl;
    actual_speed_ = actual_speed;
}