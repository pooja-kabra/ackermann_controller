/**
 * @file robot.hpp
 * @author Markose Jacob, Pooja Kabra
 * @version 2.0
 * @date 2021-10-24
 * @copyright Copyright (c) 2021
 *
 * @section Robot class
 * 
 * @brief This class contains all the physical parametes of the robot as well and 
 * stores the heading and speed for the fornt wheels
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
    * @brief Getter function to get the track length of the robot
    * @param nil
    * @return track_length_. double
    */
    double getTrackLength();

    /**
    * @brief Getter function to get the wheel base of the robot
    * @param nil
    * @return wheel_base_. double
    */
    double getWheelBase();

    /**
    * @brief Getter function to get the wheel radius of the robot
    * @param nil
    * @return wheel_radius_. double
    */
    double getWheelRadius();

    /**
    * @brief Getter function to get the turning radius of the robot
    * @param nil
    * @return wheel_base_. double
    */
    double getTurningRadius();

    /**
    * @brief Getter function to get the inner wheel heading of the robot
    * @param nil
    * @return inner_wheel_heading_. double
    */
    double getInnerWheelHeading();

    /**
    * @brief Setter function to set the inner wheel heading
    * @param inner_wheel_heading double. inner wheel heading
    * @return nil
    */
    void setInnerWheelHeading(double inner_wheel_heading);

    /**
    * @brief Getter function to get the outer wheel heading of the robot
    * @param nil
    * @return outer_wheel_heading_. double
    */
    double getOuterWheelHeading();

    /**
    * @brief Setter function to set the outter wheel heading
    * @param outter_wheel_heading double. outter wheel heading
    * @return nil
    */
    void setOuterWheelHeading(double outer_wheel_heading);

    /**
    * @brief Getter function to get the inner wheel speed of the robot
    * @param nil
    * @return inner_wheel_speed_. double
    */
    double getInnerWheelSpeed();

    /**
    * @brief Setter function to set the inner wheel speed
    * @param inner_wheel_speed double. inner wheel speed
    * @return nil
    */
    void setInnerWheelSpeed(double inner_wheel_speed);

    /**
    * @brief Getter function to get the outer wheel speed of the robot
    * @param nil
    * @return outer_wheel_speed_. double
    */
    double getOuterWheelSpeed();

    /**
    * @brief Setter function to set the outer wheel speed
    * @param outer_wheel_speed double. outer wheel speed
    * @return nil
    */
    void setOuterWheelSpeed(double outer_wheel_speed);

    /**
    * @brief Getter function to get the inner wheel angular speed of the robot
    * @param nil
    * @return inner_wheel_rps_. double
    */
    double getInnerWheelRps();

    /**
    * @brief Setter function to set the inner wheel rps
    * @param inner_wheel_rps double. inner wheel rps
    * @return nil
    */
    void setInnerWheelRps(double inner_wheel_rps);

    /**
    * @brief Getter function to get the outer wheel angular speed of the robot
    * @param nil
    * @return outer_wheel_rps_. double
    */
    double getOuterWheelRps();

    /**
    * @brief Setter function to set the outer wheel rps
    * @param outer_wheel_rps double. outer wheel rps
    * @return nil
    */
    void setOuterWheelRps(double outer_wheel_rps);

    /**
    * @brief Getter function to get the maximum turning angle for front wheels
    * @param nil
    * @return theta_max_. double
    */
    double getThetaMax();

    /**
    * @brief Getter function to get the maximum angle by which the front
    * wheels can turn by per second
    * @param nil
    * @return theta_inc_per_sec_max_. double
    */
    double getThetaIncrPerSecMax();

    /**
    * @brief Getter function to get the maximum angular speed
    * @param nil
    * @return rps_max_. double
    */
    double getRpsMax();

    /**
    * @brief Getter function to get the maximum angular acceleration for
    * the front wheels per second
    * @param nil
    * @return rps_inc_per_sec_max_. double
    */
    double getRpsIncrPerSecMax();

    double inner = 0;
    double outer = 0;
    double xl = 0;     // x1-coordinate of Robot in global frame
    double yl = 0;     // y1-coordinate of Robot in global frame
    double xr = 0;     // x2-coordinate of Robot in global frame
    double yr = -2;    // y2-coordinate of Robot in global frame

    /**
    * @brief Construct a new Robot object
    * 
    */
    Robot(double track_length = 2, double wheel_base = 4,
    double wheel_radius = 0.3,
        double inner_wheel_heading = 0, double outer_wheel_heading = 0,
        double inner_wheel_speed = 0, double outer_wheel_speed = 0,
        double inner_wheel_rps = 0,
        double outer_wheel_rps = 0, double theta_max = 45,
        double rps_max = 16.667,
        double theta_inc_per_sec_max = 15, double rps_incr_per_sec_max = 1) :

        track_length_{track_length}, wheel_base_{wheel_base},
        wheel_radius_{wheel_radius},
        inner_wheel_heading_{inner_wheel_heading},
        outer_wheel_heading_{outer_wheel_heading},
        inner_wheel_speed_{inner_wheel_speed},
        outer_wheel_speed_{outer_wheel_speed},
        inner_wheel_rps_{inner_wheel_rps}, outer_wheel_rps_{outer_wheel_rps},
        theta_max_{theta_max}, rps_max_{rps_max},
        theta_inc_per_sec_max_{theta_inc_per_sec_max},
        rps_incr_per_sec_max_{rps_incr_per_sec_max} {
        };

    /**
    * @brief Destroy the Robot object
    * 
    */
    ~Robot() {
    }


 private:
    double track_length_;           // track length (distance between two
                                    // wheels on an axle) in meters
    double wheel_base_;             // wheel base (distance between two axles)
                                    // in meters
    double wheel_radius_;           // wheel radius in meters
    double inner_wheel_heading_;    // inner wheel angle with respect to robot
                                    // longitudinal axis in degrees
    double outer_wheel_heading_;    // outer wheel angle with respect to robot
                                    // longitudinal axis in degrees
    double inner_wheel_speed_;      // linear speed of inner wheel in m/s
    double outer_wheel_speed_;      // linear speed of outer wheel in m/s
    double inner_wheel_rps_;        // angular speed of inner wheel in rps
    double outer_wheel_rps_;        // angular speed of outer wheel in rps
    double theta_max_;              // maximum angle wheel can turn with respect
                                    // to robot longitudinal
                                    // axis in degrees
    double rps_max_;                // maximum rps of the wheel motor
    double theta_inc_per_sec_max_;  // maximum angle wheel can turn with respect
                                    // to robot longitudinal axis in a sec
                                    // in degrees
    double rps_incr_per_sec_max_;   // maximum the wheel motor rps can increase
                                    // in a sec
};  // Robot class
}  // namespace ackermann
