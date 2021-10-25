/**
 * @file robot.cpp
 * @author Markose Jacob, Pooja Kabra
 * @brief This file defines the Robot class
 * @version 2.0
 * @date 2021-10-24
 * @copyright Copyright (c) 2021
 * 
 * @section Robot class
 * 
 * @brief This class contains all the physical parametes of the robot as well and 
 * stores the heading and speed for the fornt wheels
 */

#include "../include/robot.hpp"


ackermann::Robot::Robot() {
    track_length_ = 2;
    wheel_base_ = 4;
    wheel_radius_ = 0.3;
    inner_wheel_heading_ = 0;
    outer_wheel_heading_ = 0;
    inner_wheel_speed_ = 0;
    outer_wheel_speed_ = 0;
    inner_wheel_rps_ = 0;
    outer_wheel_rps_ = 0;
    theta_max_ = 45;
    rps_max_ = 16.667;
    theta_inc_per_sec_max_ = 15;
    rps_incr_per_sec_max_ = 1;
}

/**
* @brief Construct a new Robot object
* 
*/
ackermann::Robot::Robot(double track_length = 2, double wheel_base = 4,
double wheel_radius = 0.3,
    double inner_wheel_heading = 0, double outer_wheel_heading = 0,
    double inner_wheel_speed = 0, double outer_wheel_speed = 0,
    double inner_wheel_rps = 0,
    double outer_wheel_rps = 0, double theta_max = 45,
    double rps_max = 16.667,
    double theta_inc_per_sec_max = 15, double rps_incr_per_sec_max = 1) :

    track_length_{track_length}, wheel_base_{wheel_base},
    wheel_radius_{wheel_radius},
    inner_wheel_heading_{inner_wheel_heading},
    outer_wheel_heading_{outer_wheel_heading},
    inner_wheel_speed_{inner_wheel_speed},
    outer_wheel_speed_{outer_wheel_speed},
    inner_wheel_rps_{inner_wheel_rps}, outer_wheel_rps_{outer_wheel_rps},
    theta_max_{theta_max}, rps_max_{rps_max},
    theta_inc_per_sec_max_{theta_inc_per_sec_max},
    rps_incr_per_sec_max_{rps_incr_per_sec_max} {
    }

/**
 * @brief Get the Track length of the Robot in meters
 * 
 * @return double
 */
double ackermann::Robot::getTrackLength() {
    return track_length_;
}

/**
 * @brief Get the Wheel Base of the Robot in meters
 *      
 * @return double
 */
double ackermann::Robot::getWheelBase() {
    return wheel_base_;
}

/**
 * @brief Get the radius for wheels of the Robot in meters
 * 
 * @return double
 */
double ackermann::Robot::getWheelRadius() {
    return wheel_radius_;
}

/**
 * @brief Get the inner wheel angle with respect to robot longitudinal axis in degrees
 * 
 * @return double
 */
double ackermann::Robot::getInnerWheelHeading() {
    return inner_wheel_heading_;
}

/**
 * @brief Get the outer wheel angle with respect to robot longitudinal axis in degrees
 * 
 * @return double
 */
double ackermann::Robot::getOuterWheelHeading() {
    return outer_wheel_heading_;
}

/**
 * @brief Get linear speed of inner wheel in m/s
 * 
 * @return double
 */
double ackermann::Robot::getInnerWheelSpeed() {
    return inner_wheel_speed_;
}

/**
 * @brief Get linear speed of outer wheel in m/s
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
double ackermann::Robot::getInnerWheelRps() {
    return inner_wheel_rps_;
}

/**
 * @brief Get angular speed of outer wheel in rps
 * 
 * @return double
 */
double ackermann::Robot::getOuterWheelRps() {
    return outer_wheel_rps_;
}

/**
 * @brief Get the maximum angle wheel can turn with respect to robot longitudinal axis in degrees
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
double ackermann::Robot::getThetaIncrPerSecMax() {
    return theta_inc_per_sec_max_;
}

/**
 * @brief Get the maximum the wheel motor rps can increase in a sec
 * 
 * @return double
 */
double ackermann::Robot::getRpsIncrPerSecMax() {
    return rps_incr_per_sec_max_;
}

/**
 * @brief Set the inner wheel angular speed in rps
 *    
 * @param double
 */
void ackermann::Robot::setInnerWheelRps(double inner_wheel_rps) {
    inner_wheel_rps_ = inner_wheel_rps;
}

/**
 * @brief Set the outer wheel angular speed in rps
 *    
 * @param double
 */
void ackermann::Robot::setOuterWheelRps(double outer_wheel_rps) {
    outer_wheel_rps_ = outer_wheel_rps;
}

/**
 * @brief Set the inner wheel angle with respect to longitudinal axis in degrees 
 *    
 * @param double
 */
void ackermann::Robot::setInnerWheelHeading(double inner_wheel_heading) {
    inner_wheel_heading_ = inner_wheel_heading;
}

/**
 * @brief Set the outer wheel angle with respect to longitudinal axis in degrees
 *
 * @param double
 */
void ackermann::Robot::setOuterWheelHeading(double outer_wheel_heading) {
    outer_wheel_heading_ = outer_wheel_heading;
}

/**
 * @brief Set the linear speed of inner wheel in m/s
 *      
 * @param double
 */
void ackermann::Robot::setInnerWheelSpeed(double inner_wheel_speed) {
    inner_wheel_speed_ = inner_wheel_speed;
}

/**
 * @brief Set the linear speed of outer wheel in m/s
 *      
 * @param double
 */
void ackermann::Robot::setOuterWheelSpeed(double outer_wheel_speed) {
    outer_wheel_speed_ = outer_wheel_speed;
}
