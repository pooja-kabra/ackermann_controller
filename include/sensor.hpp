/**
 * @file sensor.hpp
 * @author Markose Jacob, Pooja Kabra
 * @brief This file declares the sensor that senses the Robot's 
 *        heading and linear speed in global frame
 * @version 0.1
 * @date 2021-10-16
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#pragma once
#include <iostream>
#include <vector>
namespace ackermann {
/**
 * @brief Sensor class
 * 
 */
class Sensor {
 public:
    std::vector<double> actual_heading_record;  // keeps record of actual heading of robot for each iteration
    std::vector<double> actual_speed_record;    // keeps record of actual speed of robot for each iteration
    std::vector<double> time_record;            // keeps record of time stamp at every iteration
    /**
     * @brief Getters and setters for the heading angle of the robot in the robot frame and its linear speed
     * 
     */
    double getActualHeading();
    double getActualSpeed();
    void setActualHeading(double);
    void setActualSpeed(double);

    /**
     * @brief Create an object of Sensor class
     */
    explicit Sensor(double actualHeading = 1, double actualSpeed = 1): actual_heading_{actualHeading},
                                                                        actual_speed_{actualSpeed} {
        // std::cout << "Constructor for Sensor class called" << std::endl;
    };

    /**
     * @brief Destroy an object of Sensor class
     */
    ~Sensor() {
        // std::cout << "Destructor for Sensor class called" << std::endl;
    }

 private:
        double actual_heading_;  // heading angle of the robot in the global frame
        double actual_speed_;   // linear speed of the robot
};
}  // namespace ackermann
