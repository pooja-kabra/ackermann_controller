/**
 * @file robot.hpp
 * @author Markose Jacob, Pooja Kabra
 * @brief This file declares the Robot class
 * @version 0.1
 * @date 2021-10-16
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once
#include <iostream>

namespace ackermann {
/**
 * @brief Robot class
 * 
 */
class Robot {
 public:
 /**
 * @brief Getters and Setters for Robot properties
 * 
 */
    double getTrackLength();
    double getWheelBase();
    double getWheelRadius();
    double getTurningRadius();
    double getInnerWheelHeading();
    void setInnerWheelHeading(double inner_wheel_heading);
    double getOuterWheelHeading();
    void setOuterWheelHeading(double outer_wheel_heading);
    double getInnerWheelSpeed();
    void setInnerWheelSpeed(double inner_wheel_speed);
    double getOuterWheelSpeed();
    void setOuterWheelSpeed(double outer_wheel_speed);
    double getThetaMax();

    /**
     * @brief Construct a new Robot object
     * 
     */
    Robot(double track_length = 1, double wheel_base = 1, double wheel_radius = 1,
    double turning_radius = 1, double inner_wheel_heading = 1, double outer_wheel_heading = 1,
    double inner_wheel_speed = 1, double outer_wheel_speed = 1, double theta_max = 45) :
    track_length_{track_length}, wheel_base_{wheel_base}, wheel_radius_{wheel_radius}, turning_radius_{turning_radius},
    inner_wheel_heading_{inner_wheel_heading}, outer_wheel_heading_{outer_wheel_heading},
    inner_wheel_speed_{inner_wheel_speed}, outer_wheel_speed_{outer_wheel_speed}, theta_max_{theta_max} {
        std::cout << "Constructor for Robot class called " <<std::endl;
    };

    /**
     * @brief Destroy the Robot object
     * 
     */
    ~Robot() {
        std::cout << "Destructor for Robot class called" << std::endl;
    }

 private:
    double track_length_;  // track length (distance between two wheels on an axle)
    double wheel_base_;  // wheel base (distance between two axles)
    double wheel_radius_;  // wheel radius
    double turning_radius_;  // desired turning radius
    double inner_wheel_heading_;  // inner wheel angle with respect to robot axis
    double outer_wheel_heading_;  // outer wheel angle with respect to robot axis
    double inner_wheel_speed_;  // angular speed of inner wheel
    double outer_wheel_speed_;  // angular speed of outer wheel
    double theta_max_;  // maximum angle wheel can turn with respect to robot axis
};
}   // namespace ackermann
