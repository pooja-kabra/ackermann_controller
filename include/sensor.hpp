/**
 * @file sensor.hpp
 * @author Markose Jacob, Pooja Kabra
 * @version 2.0
 * @date 2021-10-24 
 * @copyright Copyright (c) 2021
 * 
 * @section Sensor class
 * 
 * @brief This is where the actual sensors of the robot will be added to.
 * For now this class stores the actual heading and speed of the
 * robot in global frame
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
    std::vector<double> actual_heading_record;  // keeps record of actual
                                        // heading of robot for each iteration
    std::vector<double> actual_speed_record;    // keeps record of actual
                                        // speed of robot for each iteration
    std::vector<double> time_record;            // keeps record of time
                                        // stamp at every iteration

    /**
    * @brief Getter function to get the actual heading of the robot in 
    * global frame
    * @param nil
    * @return actual_heading. double
    */
    double getActualHeading();

    /**
    * @brief Getter function to get the actual linear speed of the robot in 
    * global frame
    * @param nil
    * @return actual_speed. double
    */
    double getActualSpeed();

    /**
    * @brief Setter function to set the actual heading of the robot in 
    * global frame
    * @param actual_heading double. actual heading of robot in global frame
    * @return nil
    */
    void setActualHeading(double);

    /**
    * @brief Setter function to set the actual speed of the robot in 
    * global frame
    * @param actual_speed double. actual speed of robot in global frame
    * @return nil
    */
    void setActualSpeed(double);

    /**
     * @brief Constructor for Sensor class
     */
    explicit Sensor(double actualHeading = 1, double actualSpeed = 1):
    actual_heading_{actualHeading}, actual_speed_{actualSpeed} {
    };

    /**
     * @brief Destroy an object of Sensor class
     */
    ~Sensor() {
    }

 private:
        double actual_heading_;  // heading angle of the
                                // robot in the global frame
        double actual_speed_;   // linear speed of the robot
};  // Sensor Class
}  // namespace ackermann
