/**
 * @file forwardkinematics.hpp
 * @author Markose Jacob, Pooja Kabra
 * @brief This file declares the Forward Kinematics class
 * @version 0.1
 * @date 2021-10-16
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once
#include <iostream>

namespace ackermann
{

    /**
 * @brief Forward Kinematics class
 * 
 */
    class ForwardKinematics
    {
    public:
        /**
     * @brief Calculates the error in heading angle of the robot in the robot frame
     * 
     */
        double calculateHeadingError(double desired_heading, double actual_heading);

        /**
     * @brief Calculates the error in linear speed of the robot
     * 
     */
        double calculateSpeedError(double desired_speed, double actual_speed);

        /**
     * @brief Getters and setters for the errors in heading angle of the robot in the robot frame and its linear speed
     * 
     */
        void setHeadingError(double);
        void setSpeedError(double);
        double getHeadingError();
        double getSpeedError();

        /**
     * @brief Create an object of Forward Kinematics class
     */
        explicit ForwardKinematics(double heading_error = 0, double speed_error = 0) : heading_error_{heading_error}, speed_error_{speed_error}
        {
            // std::cout << "Constructor for Forwardkinematics class called" << std::endl;
        }

        /**
     * @brief Destroys an object of Forward Kinematics class
     */
        ~ForwardKinematics()
        {
            // std::cout << "Destructor for Forwardkinematics class called" << std::endl;
        }

    private:
        double heading_error_;
        double speed_error_;
    };
} // namespace ackermann
