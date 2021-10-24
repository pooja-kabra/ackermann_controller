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
#include "../include/sensor.hpp"

namespace ackermann {

/**
 * @brief Inverse Kinematics class
 * 
 */
class InverseKinematics {
 public:


    struct headings{
        double inner = 0;
        double outer = 0;
    };

    struct speed{
        double inner_speed = 0;
        double outer_speed = 0;
    };

    /**
     * @brief Calculates the individual wheel angular speeds
     * 
     */
    headings calculateWheelHeadings(double actual_heading,double actual_speed,double dt, char direction, Robot& car);

    /**
     * @brief Calculates the individual wheel speeds
     * 
     */
    speed calculateWheelSpeeds(double actual_heading, double actual_speed, double dt, char direction,  Robot& car);

    /**
     * @brief Calculates the new heading and speed in global frame
     * 
     */
    void calculateNewRobotHeadingandSpeed(double inner_heading_incr,
    double outer_heading_incr, double inner_speed_incr, double outer_speed_incr, Sensor& sensor);

    // /**
    //  * @brief Calculates the wheel angles w.r.t. robot axis
    //  * 
    //  */
    // void calculateWheelAngles();

    // InverseKinematics();
    /**
     * @brief Create an object of Inverse Kinematics class
     */ 
    explicit InverseKinematics(double desired_heading = 0, double desired_speed = 0):
    desired_heading_{desired_heading}, desired_speed_{desired_speed} {
        // std::cout << "Constructor for Inverse kinematics class called" << std::endl;
    }

    // void setRobot(Robot car){
    //     car1 = car;
    // }

    // InverseKinematics(double desired_heading = 0, double desired_speed = 0, Robot car){
    //     desired_heading_ = desired_heading;
    //     desired_speed_ = desired_speed;
    //     car1 = car;
    // }

    /**
     * @brief Destroys an object of Inverse Kinematics class
     */ 
    ~InverseKinematics() {
        // std::cout << "Destructor for Inverse kinematics class called" << std::endl;
    }
    // Robot car1;

 private:
    double desired_heading_;
    double desired_speed_;
};
}  // namespace ackermann
