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
    double getSpeedMax();
    double getComOffset();
    double getInsMaxRot();
    double getInsSpeedMax();  //change this

    /**
     * @brief Construct a new Robot object
     * 
     */
    Robot(double track_length = 2, double wheel_base = 4, double wheel_radius = 0.3,
    double turning_radius = 0, double inner_wheel_heading = 0, double outer_wheel_heading = 0,
    double inner_wheel_speed = 0, double outer_wheel_speed = 0, double theta_max = 45, double speed_max = 16.667, double com_offset = 1, double ins_rot_max = 3, double ins_spped_max = 1) :
    track_length_{track_length}, wheel_base_{wheel_base}, wheel_radius_{wheel_radius}, turning_radius_{turning_radius},
    inner_wheel_heading_{inner_wheel_heading}, outer_wheel_heading_{outer_wheel_heading},
    inner_wheel_speed_{inner_wheel_speed}, outer_wheel_speed_{outer_wheel_speed}, theta_max_{theta_max}, speed_max_{speed_max}, com_offset_{com_offset}, ins_rot_max_{ins_rot_max}, ins_speed_max_{ins_spped_max} {
        // std::cout << "Constructor for Robot class called " <<std::endl;
    };

    /**
     * @brief Destroy the Robot object
     * 
     */
    ~Robot() {
        // std::cout << "Destructor for Robot class called" << std::endl;
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
    double speed_max_;
    double com_offset_;
    double ins_rot_max_;  // change this
    double ins_speed_max_;  //change this
};
}   // namespace ackermann
