/**
 * @file robot.cpp
 * @author Markose Jacob, Pooja Kabra
 * @brief This file defines the Robot class
 * @version 0.1
 * @date 2021-10-16
 * @copyright Copyright (c) 2021
 */

#include "../include/robot.hpp"

/**
 * @brief Get the Track length of the Robot
 * 
 * @return double
 */
double ackermann::Robot::getTrackLength() {
    return track_length_;
}

/**
 * @brief Get the Wheel Base of the Robot
 *      
 * @return double
 */
double ackermann::Robot::getWheelBase() {
    return wheel_base_;
}

/**
 * @brief Get the radius for wheels of the Robot
 * 
 * @return double
 */
double ackermann::Robot::getWheelRadius() {
    return wheel_radius_;
}

/**
 * @brief Get the desired turning radius
 * 
 * @return double
 */
double ackermann::Robot::getTurningRadius() {
    return turning_radius_;
}

/**
 * @brief Get the inner wheel angle with respect to robot axis
 * 
 * @return double
 */
double ackermann::Robot::getInnerWheelHeading() {
    return inner_wheel_heading_;
}

/**
 * @brief Get the outer wheel angle with respect to robot axis
 * 
 * @return double
 */
double ackermann::Robot::getOuterWheelHeading() {
    return outer_wheel_heading_;
}

/**
 * @brief Get angular speed of inner wheel
 * 
 * @return double
 */
double ackermann::Robot::getInnerWheelSpeed() {
    return inner_wheel_speed_;
}

/**
 * @brief Get angular speed of outer wheel
 * 
 * @return double
 */
double ackermann::Robot::getOuterWheelSpeed() {
    return outer_wheel_speed_;
}

/**
 * @brief Get angular speed of inner wheel in rps
 * 
 * @return double
 */
double ackermann::Robot::getInnerWheelRps(){

}

/**
 * @brief Set angular speed of inner wheel in rps
 * 
 * @param double
 */
void ackermann::Robot::setInnerWheelRps(double inner_wheel_rps){

}

/**
 * @brief Get angular speed of outer wheel in rps
 * 
 * @return double
 */
double ackermann::Robot::getOuterWheelRps(){

}

/**
 * @brief Set angular speed of outer wheel in rps
 * 
 * @param double
 */
void ackermann::Robot::setOuterWheelRps(double outer_wheel_rps){

}


/**
 * @brief Get the maximum angle wheel can turn with respect to robot axis in degrees
 * 
 * @return double
 */
double ackermann::Robot::getThetaMax() {
    return theta_max_;
}

/**
 * @brief Get the maximum speed with which the robot can move in km/hr
 * 
 * @return double
 */
double ackermann::Robot::getRpsMax() {
    return rps_max_;
}

/**
 * @brief Get the maximum angle wheel can turn with respect to robot axis in a sec
 * 
 * @return double
 */
double ackermann::Robot::getThetaIncrPerSecMax(){
    return theta_inc_per_sec_max_;
}

/**
 * @brief Get the maximum the wheel motor rps can increase in a sec
 * 
 * @return double
 */
double ackermann::Robot::getRpsIncrPerSecMax(){
    return rps_incr_per_sec_max_;
}

/**
 * @brief Get the Center of Mass offset of the car
 * 
 * @return double
 */
double ackermann::Robot::getComOffset() {
    return com_offset_;
}

/**
 * @brief Set the inner wheel angle with respect to robot axis
 *    
 * @param double
 */
void ackermann::Robot::setInnerWheelHeading(double inner_wheel_heading) {
    inner_wheel_heading_ = inner_wheel_heading;
}

/**
 * @brief Set the outer wheel angle with respect to robot axis
 *
 * @param double
 */
void ackermann::Robot::setOuterWheelHeading(double outer_wheel_heading) {
    outer_wheel_heading_ = outer_wheel_heading;
}

/**
 * @brief Set the angular speed of inner wheel
 *      
 * @param double
 */
void ackermann::Robot::setInnerWheelSpeed(double inner_wheel_speed) {
    inner_wheel_speed_ = inner_wheel_speed;
}

/**
 * @brief Set the angular speed of outer wheel
 *      
 * @param double
 */
void ackermann::Robot::setOuterWheelSpeed(double outer_wheel_speed) {
    outer_wheel_speed_ = outer_wheel_speed;
}