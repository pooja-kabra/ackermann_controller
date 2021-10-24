/**
 * @file controller_test.cpp
 * @author Pooja Kabra, Markose Jacob
 * @brief Unit tests for controller class
 * @version Iteration 1
 * @date 2021-10-16
 * @copyright Copyright (c) 2021
 */

#include <gtest/gtest.h>
#include "../include/controller.hpp"
// #include "../include/controller.hpp"

ackermann::Controller controller;

// ackermann::Controller controller;

// need to write test case for solve method

/**
 * @brief Testing getters and setters for goal_heading
 * @return none
 **/
TEST(test_controller_1, check_set_goal_heading)
{
  controller.setGoalHeading(70.3);
  EXPECT_EQ(controller.getGoalHeading(), 70.3);
}

/**
 * @brief Testing getters and setters for goal_speed
 * @return none
 **/
TEST(test_controller_2, check_set_goal_speed)
{
  controller.setGoalSpeed(34.36);
  EXPECT_EQ(controller.getGoalSpeed(), 34.36);
}

/**
 * @brief Testing getters for Ki
 * @return none
 **/
TEST(test_controller_3, check_get_Ki)
{
  EXPECT_EQ(controller.getKi(), 0.01);
}

/**
 * @brief Testing getters for Kp
 * @return none
 **/
TEST(test_controller_4, check_get_Kp)
{
  EXPECT_EQ(controller.getKp(), 0.03);
}
