/**
 * @file controller.hpp
 * @author Markose Jacob, Pooja Kabra
 * @copyright Copyright (c) 2021
 * @version 2.0
 * @date 2021-10-24
 * @section Controller class
 * 
 * @brief This is the core of the software which makes use of other classes
 * to calculate the headings and wheel speeds for the front wheels.
 * We first find the error in heading and speeds and the use a PID
 * controller to calculate the increment needed. Using this we use the
 * Ackermann method to calculate the heading and speeds for the front
 * wheels. Finally using the wheel headings and speed we calculate the
 * speed and heading of the robot in global frame
 */

#pragma once
#include <iostream>
#include "../include/robot.hpp"
#include "../include/sensor.hpp"
#include "../include/inversekinematics.hpp"
#include "../include/forwardkinematics.hpp"

namespace ackermann {
class Controller {
 /**
 * @brief Controller class
 * 
 */
 public:
        /**
         * @brief Setter function to update the target heading of the robot
         * with the user input.
         * @param goalHeading double
         * @return nil
         */
        void setGoalHeading(double goalHeading);

        /**
         * @brief Setter function to update the target speed of the robot
         * with the user input.
         * @param goalSpeed double
         * @return nil
         */
        void setGoalSpeed(double goalSpeed);

        /**
         * @brief Getter function to get the target heading of the robot
         * with the user input.
         * @param nil
         * @return goalHeading_
         */
        double getGoalHeading();

         /**
         * @brief Getter function to get the target speed of the robot
         * with the user input.
         * @param nil
         * @return goalSpeed_
         */
        double getGoalSpeed();

        /**
         * @brief solves for convergence. Makes use of other classes and
         * methods till the heading and speed converges. This is the 
         * heart of the software. 
         * @param sen sensor
         * @return nil
         */
        void solve(Sensor &sen);

        Robot car;
        ForwardKinematics fk;
        InverseKinematics ik;
        char direction;
        Controller() {}

    /**
     * @brief Constructor for controller class
     */
    Controller(double goal_heading, double goal_speed, double kp, double ki,
    double kd, double time_step, Robot robo, ForwardKinematics forkin,
    InverseKinematics inkin, char dir) {
        goal_heading_ = goal_heading;  // stores target heading
        goal_speed_ = goal_speed;  // stores target speed
        kp_ = kp;  // Propotional gain
        ki_ = ki;  // Intergral gain
        kd_ = kd;  // Deravative gain
        time_step_ = time_step;  // time step
        car = robo;  // object of robot class
        fk = forkin;  // object of forwardkinematics class
        ik = inkin;  // object of inversekimenatics class
        direction = dir;  // stores the direction of robot
    }

    /**
     * @brief Destroys an object of the controller class
     */
    ~Controller() {
     }

 private:
    double goal_heading_;  // desired heading angle of the robot in
                           // the robot frame
    double goal_speed_;    // desired linear speed of the robot
    double kp_;            // integral gain
    double ki_;            // proportional gain
    double kd_;            // derivative gain
    double time_step_;     // in sec
};  // class Controller
}  // namespace ackermann
