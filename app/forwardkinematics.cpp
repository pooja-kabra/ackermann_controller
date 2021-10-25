/**
 * @file forwardkinematics.cpp
 * @author Markose Jacob, Pooja Kabra
 * @brief This file defines the Forward Kinematics class. It provides functions
 *        to calculate error in heading of the robot in robot frame and its linear speed
 * @version 0.1
 * @date 2021-10-16
 * @copyright Copyright (c) 2021
 */

#include "../include/forwardkinematics.hpp"

/**
 * @brief Calculates the error in heading angle of the robot in the global frame
 * 
 * @param desired_heading 
 * @param actual_heading 
 * @return double 
 */
double ackermann::ForwardKinematics::calculateHeadingError(double
desired_heading, double actual_heading) {
    setHeadingError(desired_heading - actual_heading);
    return heading_error_;
}

/**
 * @brief Calculates the error in linear speed of the robot
 * 
 * @param desired_speed 
 * @param actual_speed 
 * @return double 
 */
double ackermann::ForwardKinematics::calculateSpeedError(double
desired_speed, double actual_speed) {
    double speed_error = desired_speed - actual_speed;
    return speed_error;
}

/**
 * @brief Sets the error in heading angle of the robot in the global frame
 * 
 * @param heading_error 
 */
void ackermann::ForwardKinematics::setHeadingError(double heading_error) {
    heading_error_ = heading_error;
}

/**
 * @brief Sets the error in linear speed of the robot
 * 
 * @param speed_error 
 */
void ackermann::ForwardKinematics::setSpeedError(double speed_error) {
    speed_error_ = speed_error;
}

/**
 * @brief Gets the error in heading angle of the robot in the global frame
 * 
 * @return double 
 */
double ackermann::ForwardKinematics::getHeadingError() {
    return heading_error_;
}

/**
 * @brief Gets the error in linear speed of the robot
 * 
 * @return double 
 */
double ackermann::ForwardKinematics::getSpeedError() {
    return speed_error_;
}
