/**
 * @file inversekinematics.cpp
 * @author Markose Jacob, Pooja Kabra
 * @brief This file defines the Inverse Kinematics class, it calculates 
 *        individual wheel angular speeds and wheel angles w.r.t. robot axisIt provides functions    
 * @version 0.1
 * @date 2021-10-16
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "../include/inversekinematics.hpp"
#include <cmath>

/**
 * @brief Calculates the individual wheel angular speeds
 * 
 */
void ackermann::InverseKinematics:: calculateWheelSpeed() {
    // TODO(Pooja): implement method
    robot_.setInnerWheelSpeed(100.2);
    robot_.setOuterWheelSpeed(150.5);
}

/**
 * @brief Calculates the wheel angles w.r.t. robot axis
 * 
 */
void ackermann::InverseKinematics::calculateWheelAngles() {
    // TODO(Pooja): implement method
    robot_.setInnerWheelHeading(15.2);
    robot_.setOuterWheelHeading(10.5);
}

