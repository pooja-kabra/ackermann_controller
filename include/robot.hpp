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
    double getInnerWheelRps();
    void setInnerWheelRps(double inner_wheel_rps);
    double getOuterWheelRps();
    void setOuterWheelRps(double outer_wheel_rps);
    double getThetaMax();
    double getThetaIncrPerSecMax();
    double getRpsMax();
    double getRpsIncrPerSecMax();
    double getComOffset();


    double inner = 0;
    double outer = 0;

    /**
     * @brief Construct a new Robot object
     * 
     */
    Robot(double track_length = 2, double wheel_base = 4, double wheel_radius = 0.3,
    double turning_radius = 0, double inner_wheel_heading = 0, double outer_wheel_heading = 0,
    double inner_wheel_speed = 0, double outer_wheel_speed = 0, double inner_wheel_rps = 0,
    double outer_wheel_rps = 0, double theta_max = 45, double rps_max = 16.667, 
    double theta_inc_per_sec_max = 15, double rps_incr_per_sec_max = 1, double com_offset = 1) :

    track_length_{track_length}, wheel_base_{wheel_base}, wheel_radius_{wheel_radius}, turning_radius_{turning_radius},
    inner_wheel_heading_{inner_wheel_heading}, outer_wheel_heading_{outer_wheel_heading},
    inner_wheel_speed_{inner_wheel_speed}, outer_wheel_speed_{outer_wheel_speed}, 
    inner_wheel_rps_{inner_wheel_rps}, outer_wheel_rps_{outer_wheel_rps},
    theta_max_{theta_max}, rps_max_{rps_max}, theta_inc_per_sec_max_{theta_inc_per_sec_max}, 
    rps_incr_per_sec_max_{rps_incr_per_sec_max} ,com_offset_{com_offset} {
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
    double xl_;  // x1-coordinate of Robot in global frame
    double yl_;  // y1-coordinate of Robot in global frame
    double xr_;  // x2-coordinate of Robot in global frame
    double yr_;  // y2-coordinate of Robot in global frame
    double inner_wheel_heading_;  // inner wheel angle with respect to robot axis
    double outer_wheel_heading_;  // outer wheel angle with respect to robot axis
    double inner_wheel_speed_;  // linear speed of inner wheel in m/s
    double outer_wheel_speed_;  // linear speed of outer wheel in m/s
    double inner_wheel_rps_;  // angular speed of inner wheel in rps
    double outer_wheel_rps_;  // angular speed of outer wheel in rps
    double theta_max_;  // maximum angle wheel can turn with respect to robot axis
    double rps_max_;  // maximum rps of the wheel motor
    double theta_inc_per_sec_max_;  // maximum angle wheel can turn with respect to robot axis in a sec
    double rps_incr_per_sec_max_;  // maximum the wheel motor rps can increase in a sec
    double com_offset_;
};
}   // namespace ackermann
