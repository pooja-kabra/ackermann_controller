#include <iostream>
#include "../include/sensor.hpp"
// #include "sensor.cpp"
#include "../include/robotkinematics.hpp"
// #include "robotkinematics.cpp"


int main()
{
    ackermann::Sensor sensor;
    ackermann::RobotKinematics rk;
    sensor.setActualHeading(30.5);
    sensor.setActualSpeed(100.56);
    std::cout<<sensor.getActualHeading()<<std::endl;
    std::cout<<sensor.getActualSpeed()<<std::endl;
    return 0;
}
