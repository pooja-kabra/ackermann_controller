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

ackermann:: Robot robot;
ackermann::InverseKinematics ik(33, 12);

/**
 * @brief This test checks for correct individual wheel angles w.r.t. the robot axis
 * @return none
 **/
TEST(test_inverseKinematics_1, check_wheel_angles) {
  // ik.calculateWheelHeadings();
  EXPECT_NEAR(robot.getInnerWheelHeading(), 37, 0.1);
  EXPECT_NEAR(robot.getOuterWheelHeading(), 30, 0.1);
}

/**
 * @brief This test checks for correct individual wheel speeds
 * @return none
 **/
TEST(test_inverseKinematics_2, check_wheel_speeds) {
  // ik.calculateWheelSpeed();
  EXPECT_NEAR(robot.getInnerWheelSpeed(), 50, 0.1);
  EXPECT_NEAR(robot.getOuterWheelSpeed(), 70, 0.1);
}
