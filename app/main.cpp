/**
 * @file main.cpp
 * @author Markose Jacob, Pooja Kabra
 * @brief Defines the main file. The program starts from here.
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
// #include "sensor.cpp"




int main() {
    double goal_heading, goal_speed;
    std::cout<<"Enter the goal heading for the robot wrt to the world: "<<std::endl;
    std::cin>>goal_heading;
    std::cout<<"Enter the goal speed for the robot (MAXIMUM SPEED IS 16.667 m/s) : "<<std::endl;
    std::cin>>goal_speed;
    while (goal_speed > 16.667){
        std::cout<<"Enter the goal speed again for the robot (MAXIMUM SPEED IS 16.667 m/s) : "<<std::endl;
        std::cin>>goal_speed;
    }
    ackermann::Robot robo;
    ackermann::Sensor sen(0,0);
    ackermann::ForwardKinematics forkin(0,0);
    ackermann::InverseKinematics inkin;
    ackermann::Controller control(goal_heading,goal_speed,0.5,0.001,0.01,0.1,robo,sen,forkin,inkin,3,0.27);
    std::cout<<"\n \n"<<std::endl;
    control.solve();
    std::cout<<"\n \n"<<std::endl;    ackermann::Sensor sensor;
    ackermann::Robot r;
    sensor.setActualHeading(30.5);
    sensor.setActualSpeed(100.56);
    std::cout << sensor.getActualHeading() << std::endl;
    std::cout << sensor.getActualSpeed() << std::endl;
    return 0;
}
