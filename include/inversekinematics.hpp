/**
 * @file forwardkinematics.hpp
 * @author Pooja Kabra, Markose Jacob
 * @brief This file declares the Inverse Kinematics class, it calculates 
 *        individual wheel angular speeds and wheel angles w.r.t. robot axis
 * @version 0.1
 * @date 2021-10-16
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once
#include <iostream>
#include "../include/robot.hpp"

namespace ackermann {

/**
 * @brief Inverse Kinematics class
 * 
 */
class InverseKinematics {
 public:
    /**
     * @brief Calculates the individual wheel angular speeds
     * 
     */
    void calculateWheelSpeed();

    /**
     * @brief Calculates the wheel angles w.r.t. robot axis
     * 
     */
    void calculateWheelAngles();


    /**
     * @brief Create an object of Inverse Kinematics class
     */ 
    explicit InverseKinematics(Robot robot, double desired_heading, double desired_speed):
    robot_{robot}, desired_heading_{desired_heading}, desired_speed_{desired_speed} {
        std::cout << "Constructor for Inverse kinematics class called" << std::endl;
    }

    /**
     * @brief Destroys an object of Inverse Kinematics class
     */ 
    ~InverseKinematics() {
        std::cout << "Destructor for Inverse kinematics class called" << std::endl;
    }

 private:
    Robot robot_;
    double desired_heading_;
    double desired_speed_;
};
}  // namespace ackermann

