/**
 * @file controller.hpp
 * @author Markose Jacob, Pooja Kabra
 * @brief This file defines the controller class
 * @version 0.1
 * @date 2021-10-16
 * @copyright Copyright (c) 2021
 */

#include "../include/controller.hpp"


// TODO(Pooja): need to write solve method
/**
 * @brief This method makes the output heading angle of the robot in the robot frame
 *        and its linear speed converge to goal values
 *
 * @param double
 */
void ackermann::Controller::solve() {
}


/**
 * @brief Set the desired heading angle of the robot in the robot frame
 *
 * @param double
 */
void ackermann::Controller::setGoalHeading(double goal_heading) {
    goal_heading_ = goal_heading;
}

/**
 * @brief Set the desired linear speed of the robot
 *
 * @param double
 */
void ackermann::Controller::setGoalSpeed(double goal_speed) {
    goal_speed_ = goal_speed;
}

/**
 * @brief Get the desired heading angle of the robot in the robot frame
 *
 * 
 * @return double
 */
double ackermann::Controller::getGoalHeading() {
    return goal_heading_;
}

/**
 * @brief Get the desired linear speed of the robot
 * 
 * @return double
 */
double ackermann::Controller::getGoalSpeed() {
    return goal_speed_;
}

/**
 * @brief Get integral gain of controller
 * 
 * @return double
 */
double ackermann::Controller::getKi() {
    return ki_;
}

/**
 * @brief Get proportional gain of controller
 * 
 * @return double
 */
double ackermann::Controller::getKp() {
    return kp_;
}