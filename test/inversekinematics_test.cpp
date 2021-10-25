/**
 * @file inversekinematics_test.cpp
 * @author Pooja Kabra, Markose Jacob
 * @brief This file tests the Inverse Kinematics class, checks if it calculates 
 *        individual wheel angular speeds and wheel angles w.r.t. robot axis correctly
 * @version Iteration 1
 * @date 2021-10-16
 * @copyright Copyright (c) 2021
 */

#include <gtest/gtest.h>
#include "../include/inversekinematics.hpp"
#include "../include/robot.hpp"

ackermann::Robot robot(2, 4, 0.3, 1.34984, 1.34984, 0.449966, 0.50001, 16.667, 1.6667, 45, 16.667, 15, 1);
ackermann::InverseKinematics ik(59.3438, 7.36751);
/**
 * @brief This test checks for correct individual wheel angles w.r.t. the robot axis
 * @return none
 **/
TEST(test_inverseKinematics_1, check_calculated_wheel_angles)
{
  ackermann::InverseKinematics::headings head_res_increment;
  head_res_increment = ik.calculateWheelHeadings(59.3438, 0.1, 'l', robot);
  EXPECT_NEAR(head_res_increment.inner, 1.5, 0.1);
  EXPECT_NEAR(head_res_increment.outer, 1.34984, 0.1);
}

/**
 * @brief This test checks for correct individual wheel speeds
 * @return none
 **/
TEST(test_inverseKinematics_2, check_calculated_wheel_speeds)
{
  ackermann::InverseKinematics::speed spd_res_increment;
  spd_res_increment = ik.calculateWheelSpeeds(59.3438, 7.36751, 0.1, 'l', robot);
  EXPECT_NEAR(spd_res_increment.inner_speed, 0.449995, 0.1);
  EXPECT_NEAR(spd_res_increment.outer_speed, 0.50001, 0.1);
}

/**
 * @brief This test checks for correct individual wheel speeds
 * @return none
 **/
TEST(test_inverseKinematics_3, check_new_robot_heading_and_speed)
{
  ackermann::Robot robot(2, 4, 0.3, 2.69968, 2.69968, 0.899961, 1.00002, 2.99987, 3.3334, 45, 16.667, 15, 1);
  ackermann::Sensor sensor(0.223182, 0.474988);
  ik.calculateNewRobotHeadingandSpeed(1.5, 1.34984, sensor, robot, 0.1);
  EXPECT_NEAR(sensor.getActualHeading(), 0.447228, 0.1);
  EXPECT_NEAR(sensor.getActualSpeed(), 0.94999, 0.1);
}
