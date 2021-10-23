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
#include "robot.hpp"
#include "sensor.hpp"
#include "../include/inversekinematics.hpp"
#include "../include/forwardkinematics.hpp"

namespace ackermann {
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
     * @brief Getters for controller gains
     */ 
    double getKi();
    double getKp();
    // need to write solve method

    /**
     * @brief This method makes the output heading angle of the robot in the robot frame
     *        and its linear speed converge to goal values
     *
     * @param double
     */
    void solve();

    /**
     * @brief Create an object of the controller class
     */ 
    explicit Controller(double goal_heading = 0, double goal_speed = 0, double ki = 0.01, double kp = 0.03) :
    goal_heading_{goal_heading}, goal_speed_{goal_speed}, ki_{ki}, kp_{kp} {
        std::cout << "Constructor for Controller class called" << std::endl;
    };

    /**
     * @brief Destroys an object of the controller class
     */ 
    ~Controller() {
        std::cout << "Destructor for Controller class called" << std::endl;
    }


 private:
    double goal_heading_;  // desired heading angle of the robot in the robot frame
    double goal_speed_;  // desired linear speed of the robot
    double ki_;  // integral gain
    double kp_;  // proportional gain
};
}  // namespace ackermann
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
#include "robot.hpp"
#include "sensor.hpp"
#include "../include/inversekinematics.hpp"
#include "../include/forwardkinematics.hpp"

namespace ackermann {
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
     * @brief Getters for controller gains
     */ 
    double getKi();
    double getKp();
    // need to write solve method

    /**
     * @brief This method makes the output heading angle of the robot in the robot frame
     *        and its linear speed converge to goal values
     *
     * @param double
     */
    void solve();

    /**
     * @brief Create an object of the controller class
     */ 
    // explicit Controller(double goal_heading = 0, double goal_speed = 0, double kp = 0.01, double ki = 0.03, double kd = 0.01,
    // double time_step = 0.2, Robot car1 = (0,0,0,0,0,0,0,0,0), Sensor sensor1 = (0,0), ForwardKinematics fk1, InverseKinematics ik1) :
    // goal_heading_{goal_heading}, goal_speed_{goal_speed}, kp_{kp}, ki_{ki}, kd_{kd}, time_step_{time_step},
    // car{car1}, sensor{sensor1}, fk{fk1}, ik{ik1} {
    //     std::cout << "Constructor for Controller class called" << std::endl;
    // };
    Controller(double goal_heading, double goal_speed, double kp, double ki, double kd, double time_step, Robot robo, Sensor sen, ForwardKinematics forkin, InverseKinematics inkin, double instantaneous_rotation, double instantaneous_speed) {
        // std::cout << "Constructor for Controller class called" << std::endl;
        goal_heading_ = goal_heading;
        goal_speed_ = goal_speed;
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        time_step_ = time_step;
        car = robo;
        sensor = sen;
        fk = forkin;
        ik = inkin;
        max_instantaneous_wheel_rotation_ = instantaneous_rotation;
        max_instantaneous_speed_increment_ = instantaneous_speed;
    };


    /**
     * @brief Destroys an object of the controller class
     */ 
    ~Controller() {
        // std::cout << "Destructor for Controller class called" << std::endl;
    }

    Robot car;
    Sensor sensor;
    ForwardKinematics fk;
    InverseKinematics ik;
    char direction;



 private:
    double goal_heading_;  // desired heading angle of the robot in the robot frame
    double goal_speed_;  // desired linear speed of the robot
    double kp_;  // integral gain
    double ki_;  // proportional gain
    double kd_;
    double time_step_;
    double max_instantaneous_wheel_rotation_;
    double max_instantaneous_speed_increment_;
};
}  // namespace ackermann
