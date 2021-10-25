/**
 * @file controller.hpp
 * @author Markose Jacob, Pooja Kabra
 * @brief This file declares the controller class
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
#include "../include/inversekinematics.hpp"
#include "../include/forwardkinematics.hpp"

namespace ackermann
{
    /**
 * @brief controller class
 * 
 */
class Controller {
 public:
        /**
         * @brief Getters and setters for the desired heading angle of the robot in the robot frame and 
         *        desired linear speed of the robot
         * 
         */
        void setGoalHeading(double goalHeading);
        void setGoalSpeed(double goalSpeed);
        double getGoalHeading();
        double getGoalSpeed();
        /**
         * @brief solves for convergence
         * 
         * @param sen sensor
         */
        void solve(Sensor &sen);

        Robot car;
        ForwardKinematics fk;
        InverseKinematics ik;
        char direction;
        Controller() {}

    /**
     * @brief Create an object of the controller class
     */
    Controller(double goal_heading, double goal_speed, double kp, double ki, double kd, double time_step,
                                Robot robo, ForwardKinematics forkin, InverseKinematics inkin, char dir) {
        goal_heading_ = goal_heading;
        goal_speed_ = goal_speed;
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        time_step_ = time_step;
        car = robo;
        fk = forkin;  // forward kinematics object for the robot
        ik = inkin;   // inverse kinematics object for the robot
        direction = dir;
    }

    /**
     * @brief Destroys an object of the controller class
     */
    ~Controller() {
        // std::cout << "Destructor for Controller class called" << std::endl;
     }

 private:
    double goal_heading_;  // desired heading angle of the robot in the robot frame
    double goal_speed_;    // desired linear speed of the robot
    double kp_;            // integral gain
    double ki_;            // proportional gain
    double kd_;            // derivative gain
    double time_step_;     // in sec
};
}  // namespace ackermann
