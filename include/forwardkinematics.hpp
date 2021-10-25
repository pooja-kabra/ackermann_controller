/**
 * @file forwardkinematics.hpp
 * @author Markose Jacob, Pooja Kabra
 * @version 2.0
 * @date 2021-10-24
 * @copyright Copyright (c) 2021
 * 
 * @section ForwardKinematics class
 * 
 * @brief This is where we are calculating the error between the traget heading
 * and speed against the robots global heading and speed
 */

#pragma once
#include <iostream>

namespace ackermann {
 /**
 * @brief ForwardKinematics class
 * 
 */
class ForwardKinematics {
 public:
   /**
    * @brief Calculates the error in heading angle of the robot
    * @param desired_heading double. This the target heading for the robot
    * @param actual_heading double. This the current heading of the robot in global frame
    * @return double. The error in heading
    */
    double calculateHeadingError(double desired_heading, double actual_heading);

   /**
    * @brief Calculates the error in linear speed of the robot
    * @param desired_speed double. This the target linear speed in m/s for the robot
    * @param actual_heading double. This the current linear speed (m/s) of the robot in global frame
    * @return double. The error in speed
    */
    double calculateSpeedError(double desired_speed, double actual_speed);

   /**
   * @brief Setter function to update the error in heading of the robot
   * @param heading_error double. The error in heading for each iteration
   * @return nil
   */
    void setHeadingError(double);

   /**
   * @brief Setter function to update the error in speed of the robot
   * @param speed_error double. The error in speed for each iteration
   * @return nil
   */
    void setSpeedError(double);

   /**
   * @brief Getter function to get the error in heading of the robot
   * @param nil
   * @return heading_error. double
   */
    double getHeadingError();

   /**
   * @brief Getter function to get the error in speed of the robot
   * @param nil
   * @return speed_error. double
   */
    double getSpeedError();

   /**
    * @brief Constructor for ForwardKinematics class
    */
    ForwardKinematics(double heading_error = 0,
    double speed_error = 0) : heading_error_{heading_error},
    speed_error_{speed_error} {
    }

   /**
    * @brief Destroys an object of Forward Kinematics class
    */
      ~ForwardKinematics() {
      }

 private:
      double heading_error_;  // stores heading error for each iteration
      double speed_error_;  // stores speed error for each iteration
};  // ForwardKinematics class
}  // namespace ackermann
