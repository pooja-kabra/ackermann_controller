
/**
 * @file robotkinematics_test.cpp
 * @author Pooja Kabra, Markose Jacob
 * @brief Unit tests for RobotKinematics class
 * @version Iteration 1
 * @date 2021-10-16
 * @copyright Copyright (c) 2021
 */

#include <gtest/gtest.h>
#include "../include/robot.hpp"

ackermann::Robot rk(5, 6, 7, 8);

/**
 * @brief Testing getters and setters for inner_wheel_heading
 * @return none
 **/
TEST(test_robotkinematics_1, check_set_inner_wheel_heading)
{
  rk.setInnerWheelHeading(20.22);
  EXPECT_EQ(rk.getInnerWheelHeading(), 20.22);
}

/**
 * @brief Testing getters and setters for outter_wheel_heading
 * @return none
 **/
TEST(test_robotkinematics_2, check_set_outter_wheel_heading)
{
  rk.setOuterWheelHeading(40.52);
  EXPECT_EQ(rk.getOuterWheelHeading(), 40.52);
}

/**
 * @brief Testing getters and setters for inner_wheel_speed
 * @return none
 **/
TEST(test_robotkinematics_3, check_set_inner_wheel_speed)
{
  rk.setInnerWheelSpeed(10.85);
  EXPECT_EQ(rk.getInnerWheelSpeed(), 10.85);
}

/**
 * @brief Testing getters and setters for outter_wheel_speed
 * @return none
 **/
TEST(test_robotkinematics_4, check_set_outter_wheel_speed)
{
  rk.setOuterWheelSpeed(14.87);
  EXPECT_EQ(rk.getOuterWheelSpeed(), 14.87);
}

/**
 * @brief Testing getter track_length
 * @return none
 **/
TEST(test_robotkinematics_5, check_get_track_length)
{
  EXPECT_EQ(rk.getTrackLength(), 5);
}

/**
 * @brief Testing getter wheel_base
 * @return none
 **/
TEST(test_robotkinematics_6, check_get_wheel_base)
{
  EXPECT_EQ(rk.getWheelBase(), 6);
}

/**
 * @brief Testing getter wheel_radius
 * @return none
 **/
TEST(test_robotkinematics_7, check_get_wheel_radius)
{
  EXPECT_EQ(rk.getWheelRadius(), 7);
}

/**
 * @brief Testing getter turing_radius
 * @return none
 **/
TEST(test_robotkinematics_8, check_get_turning_radius)
{
  EXPECT_EQ(rk.getTurningRadius(), 8);
}

/**
 * @brief Testing getter theata_max
 * @return none
 **/
TEST(test_robotkinematics_9, check_get_theata_max)
{
  EXPECT_EQ(rk.getThetaMax(), 45);
}
