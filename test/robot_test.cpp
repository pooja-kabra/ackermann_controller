
/**
 * @file robot_test.cpp
 * @author Pooja Kabra, Markose Jacob
 * @brief Unit tests for Robot class
 * @version Iteration 1
 * @date 2021-10-16
 * @copyright Copyright (c) 2021
 */

#include <gtest/gtest.h>
#include "../include/robot.hpp"

/*  track_length, wheel_base,  wheel_radius,
    inner_wheel_heading,  outer_wheel_heading,
    inner_wheel_speed,  outer_wheel_speed,
    inner_wheel_rps, outer_wheel_rps,
    theta_max,  rps_max,
    theta_inc_per_sec_max,  rps_incr_per_sec_max  */

ackermann::Robot rk(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0);

/**
 * @brief Testing getter for track length
 * @return none
 **/
TEST(test_robot_1, check_get_track_length)
{
  EXPECT_EQ(rk.getTrackLength(), 1.0);
}

/**
 * @brief Testing getter for wheel base
 * @return none
 **/
TEST(test_robot_2, check_get_wheel_base)
{
  EXPECT_EQ(rk.getWheelBase(), 2.0);
}

/**
 * @brief Testing getter for wheel radius
 * @return none
 **/
TEST(test_robot_3, check_get_wheel_radius)
{
  EXPECT_EQ(rk.getWheelRadius(), 3.0);
}


/**
 * @brief Testing getters and setters for inner_wheel_heading
 * @return none
 **/
TEST(test_robot_4, check_set_inner_wheel_heading)
{
  rk.setInnerWheelHeading(20.22);
  EXPECT_EQ(rk.getInnerWheelHeading(), 20.22);
}

/**
 * @brief Testing getters and setters for outter_wheel_heading
 * @return none
 **/
TEST(test_robot_5, check_set_outer_wheel_heading)
{
  rk.setOuterWheelHeading(40.52);
  EXPECT_EQ(rk.getOuterWheelHeading(), 40.52);
}

/**
 * @brief Testing getters and setters for inner_wheel_speed
 * @return none
 **/
TEST(test_robot_6, check_set_inner_wheel_speed)
{
  rk.setInnerWheelSpeed(10.85);
  EXPECT_EQ(rk.getInnerWheelSpeed(), 10.85);
}

/**
 * @brief Testing getters and setters for outter_wheel_speed
 * @return none
 **/
TEST(test_robot_7, check_set_outer_wheel_speed)
{
  rk.setOuterWheelSpeed(14.87);
  EXPECT_EQ(rk.getOuterWheelSpeed(), 14.87);
}

/**
 * @brief Testing getters and setters for inner wheel rps
 * @return none
 **/
TEST(test_robot_8, check_set_inner_wheel_rps)
{
  rk.setInnerWheelRps(10.85);
  EXPECT_DOUBLE_EQ(rk.getInnerWheelRps(), 10.85);
}

/**
 * @brief Testing getters and setters for outer wheel rps
 * @return none
 **/
TEST(test_robot_9, check_set_outer_wheel_rps)
{
  rk.setOuterWheelRps(14.87);
  EXPECT_EQ(rk.getOuterWheelRps(), 14.87);
}

/**
 * @brief Testing getter for max theta. Theta is wheel orientation
 *        w.r.t. robot longitudinal axis
 * @return none
 **/
TEST(test_robot_10, check_get_theta_max)
{
  EXPECT_EQ(rk.getThetaMax(), 10.0);
}

/**
 * @brief Testing getter for max theta increase per second
 * @return none
 **/
TEST(test_robot_12, check_get_theta_increase_per_second_max)
{
  EXPECT_EQ(rk.getThetaIncrPerSecMax(), 12.0);
}

/**
 * @brief Testing getter for max rotation per second of wheels
 * @return none
 **/
TEST(test_robot_11, check_get_rps_max)
{
  EXPECT_EQ(rk.getRpsMax(), 11.0);
}

/**
 * @brief Testing getter for max rps increase per second of wheels
 * @return none
 **/
TEST(test_robot_13, check_get_rps_increase_per_second_max)
{
  ackermann::Robot* rk = new ackermann::Robot(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0);
  EXPECT_EQ(rk->getRpsIncrPerSecMax(), 13.0);
}