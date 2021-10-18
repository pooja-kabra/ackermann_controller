/**
 * @file main.cpp
 * @author Markose Jacob, Pooja Kabra
 * @brief 
 * @version 0.1
 * @date 2021-10-16
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include "../include/sensor.hpp"
#include "../include/robot.hpp"
#include "../include/controller.hpp"
#include "../include/forwardkinematics.hpp"



int main() {
    ackermann::Sensor sensor;
    ackermann::Robot r;
    sensor.setActualHeading(30.5);
    sensor.setActualSpeed(100.56);
    std::cout << sensor.getActualHeading() << std::endl;
    std::cout << sensor.getActualSpeed() << std::endl;
    return 0;
}
