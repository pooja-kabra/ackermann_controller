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
 * @brief Get the Track length of the Robot in meters
 * 
 * @return double
 */
double ackermann::Robot::getTrackLength()
{
    return track_length_;
}

/**
 * @brief Get the Wheel Base of the Robot in meters
 *      
 * @return double
 */
double ackermann::Robot::getWheelBase()
{
    return wheel_base_;
}

/**
 * @brief Get the radius for wheels of the Robot in meters
 * 
 * @return double
 */
double ackermann::Robot::getWheelRadius()
{
    return wheel_radius_;
}

/**
 * @brief Get the inner wheel angle with respect to robot longitudinal axis in degrees
 * 
 * @return double
 */
double ackermann::Robot::getInnerWheelHeading()
{
    return inner_wheel_heading_;
}

/**
 * @brief Get the outer wheel angle with respect to robot longitudinal axis in degrees
 * 
 * @return double
 */
double ackermann::Robot::getOuterWheelHeading()
{
    return outer_wheel_heading_;
}

/**
 * @brief Get linear speed of inner wheel in m/s
 * 
 * @return double
 */
double ackermann::Robot::getInnerWheelSpeed()
{
    return inner_wheel_speed_;
}

/**
 * @brief Get linear speed of outer wheel in m/s
 * 
 * @return double
 */
double ackermann::Robot::getOuterWheelSpeed()
{
    return outer_wheel_speed_;
}

/**
 * @brief Get angular speed of inner wheel in rps
 * 
 * @return double
 */
double ackermann::Robot::getInnerWheelRps()
{
    return inner_wheel_rps_;
}

/**
 * @brief Get angular speed of outer wheel in rps
 * 
 * @return double
 */
double ackermann::Robot::getOuterWheelRps()
{
    return outer_wheel_rps_;
}

/**
 * @brief Get the maximum angle wheel can turn with respect to robot longitudinal axis in degrees
 * 
 * @return double
 */
double ackermann::Robot::getThetaMax()
{
    return theta_max_;
}

/**
 * @brief Get the maximum speed with which the robot can move in km/hr
 * 
 * @return double
 */
double ackermann::Robot::getRpsMax()
{
    return rps_max_;
}

/**
 * @brief Get the maximum angle wheel can turn with respect to robot axis in a sec
 * 
 * @return double
 */
double ackermann::Robot::getThetaIncrPerSecMax()
{
    return theta_inc_per_sec_max_;
}

/**
 * @brief Get the maximum the wheel motor rps can increase in a sec
 * 
 * @return double
 */
double ackermann::Robot::getRpsIncrPerSecMax()
{
    return rps_incr_per_sec_max_;
}

/**
 * @brief Set the inner wheel angular speed in rps
 *    
 * @param double
 */
void ackermann::Robot::setInnerWheelRps(double inner_wheel_rps)
{
    inner_wheel_rps_ = inner_wheel_rps;
}

/**
 * @brief Set the outer wheel angular speed in rps
 *    
 * @param double
 */
void ackermann::Robot::setOuterWheelRps(double outer_wheel_rps)
{
    outer_wheel_rps_ = outer_wheel_rps;
}

/**
 * @brief Set the inner wheel angle with respect to longitudinal axis in degrees 
 *    
 * @param double
 */
void ackermann::Robot::setInnerWheelHeading(double inner_wheel_heading)
{
    inner_wheel_heading_ = inner_wheel_heading;
}

/**
 * @brief Set the outer wheel angle with respect to longitudinal axis in degrees
 *
 * @param double
 */
void ackermann::Robot::setOuterWheelHeading(double outer_wheel_heading)
{
    outer_wheel_heading_ = outer_wheel_heading;
}

/**
 * @brief Set the linear speed of inner wheel in m/s
 *      
 * @param double
 */
void ackermann::Robot::setInnerWheelSpeed(double inner_wheel_speed)
{
    inner_wheel_speed_ = inner_wheel_speed;
}

/**
 * @brief Set the linear speed of outer wheel in m/s
 *      
 * @param double
 */
void ackermann::Robot::setOuterWheelSpeed(double outer_wheel_speed)
{
    outer_wheel_speed_ = outer_wheel_speed;
}