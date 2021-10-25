/**
 * @file sensor.cpp
 * @author Markose Jacob, Pooja Kabra
 * @version 0.1
 * @date 2021-10-16
 * @copyright Copyright (c) 2021
 * 
 * @section Sensor class
 * 
 * @brief This is where the actual sensors of the robot will be added to.
 * For now this class stores the actual heading and speed of the
 * robot in global frame
 * 
 */

#include "../include/sensor.hpp"

/**
 * @brief Get the heading angle of the robot in the global frame
 * 
 * @return double
 */
double ackermann::Sensor::getActualHeading() {
    return actual_heading_;
}

/**
 * @brief Get the linear speed of the robot
 * 
 * @return double
 */
double ackermann::Sensor::getActualSpeed() {
    return actual_speed_;
}

/**
 * @brief Set the heading angle of the robot in the global frame
 *    
 * @param double
 */
void ackermann::Sensor::setActualHeading(double actual_heading) {
    // std::cout << "setActualHeading called" << std::endl;
    actual_heading_ = actual_heading;
}

/**
 * @brief Set the linear speed of the robot
 *    
 * @param double
 */
void ackermann::Sensor::setActualSpeed(double actual_speed) {
    // std::cout << "setActualSpeed called" << std::endl;
    actual_speed_ = actual_speed;
}
