/**
 * @file inversekinematics.hpp
 * @author Pooja Kabra, Markose Jacob
 * @version 2.0
 * @date 2021-10-24 
 * @copyright Copyright (c) 2021
 * 
 * @section InverseKinematics class
 * 
 * @brief This class uses the Ackermann method to calculte the headings and 
 * speed for each of the front wheels which taking into consideration the
 * physical limits of the robot. It then calculates the global heading
 * and linear speed for the robot.
 */

#pragma once
#include <iostream>
#include <vector>
#include "../include/robot.hpp"
#include "../include/sensor.hpp"



namespace ackermann {

 /**
 * @brief Inverse Kinematics class
 * 
 */
class InverseKinematics {
 public:
    /* struct to return the inner and outer wheel headings for each
    iteration */
    struct headings {
        double inner = 0;
        double outer = 0;
    };

    /* struct to return the inner and outer wheel speeds for each
    iteration */
    struct speed {
        double inner_speed = 0;
        double outer_speed = 0;
    };

    /**
     * @brief Calculates the individual wheel headings in degrees for front two
     * wheels using the Ackermann method. This also takes into consideration 
     * the physical limits of the robot.
     * @param actual_heading double. The PID output for heading
     * @param dt double. Time step for which PID gave the output
     * @param diection char. Stores the direction of turn
     * @param car Robot. Passing object car by reference to access parameters 
     * of the car like wheelbase, radius of wheel etc and also to set the
     * wheel headings based on ackermann method.
     * @return headings struct. Returns the heading for each of the wheel with
     * respect to robot frame for each iteration.
     */
    headings calculateWheelHeadings(double actual_heading, double dt,
    char direction, Robot &car);

    /**
     * @brief Calculates the individual wheel linear speeds in m/s for front two
     * wheels using the Ackermann method. This also takes into consideration 
     * the physical limits of the robot.
     * 
     * @param actual_heading double. The PID output for heading
     * @param actual_speed double. The PID output for speed
     * @param dt double. Time step for which PID gave the output
     * @param diection char. Stores the direction of turn
     * @param car Robot. Passing object car by reference to access parameters 
     * of the car like wheelbase, radius of wheel etc and also to set the
     * wheel speed based on ackermann method.
     * @return speed struct. Returns the speed for each of the wheel with
     * respect to robot frame for each iteration.
     */
    speed calculateWheelSpeeds(double actual_heading, double actual_speed,
    double dt, char direction, Robot &car);

    /**
     * @brief Calculates the global heading in degrees and linear speed in m/s
     * for the robot based on the heading and speeds of the front wheels.
     * 
     * @param inner_heading_incr double. Increment in heading for
     * inner wheel for each iteration
     * @param outer_heading_incr double. Increment in heading for
     * outer wheel for each iteration.
     * @param senor Sensor. Passing object sensor by reference to set
     * parameters like actualheading and actualspeed
     * @param car Robot. Passing object car by reference to access parameters 
     * of the car like wheelbase, radius of wheel etc and also to set the
     * wheel speed based on ackermann method.
     * @param dt double. Time step for which PID gave the output
     * @return nil
     */
    void calculateNewRobotHeadingandSpeed(double inner_heading_incr,
                       double outer_heading_incr, Sensor &sensor,
                       Robot &car, double dt);

    /**
     * @brief Constructor for InverseKinematics class
     */
    InverseKinematics(double desired_heading = 0,
    double desired_speed = 0) :
    desired_heading_{desired_heading}, desired_speed_{desired_speed} {
    }

    /**
     * @brief Destroys an object of Inverse Kinematics class
     */
    ~InverseKinematics() {
    }

 private:
    double desired_heading_;
    double desired_speed_;
};  // InverseKinematics class
}  // namespace ackermann
