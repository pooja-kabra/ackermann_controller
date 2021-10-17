/**
 * @file controller_test.cpp
 * @author Pooja Kabra, Markose Jacob
 * @brief 
 * @version 0.1
 * @date 2021-10-16
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <gtest/gtest.h>
#include "../include/controller.hpp"

ackermann::Controller controller;

/**
 * Document
 */
TEST(test_controller_1, check_set_goal_heading) {
  controller.setGoalHeading(70.3);
  EXPECT_EQ(controller.getGoalHeading(), 70.3);

}

/**
 * Document
 */
TEST(test_controller_2, check_set_goal_speed) {
  controller.setGoalSpeed(34.36);
  EXPECT_EQ(controller.getGoalSpeed(), 34.36);

}

/**
 * Document
 */
TEST(test_controller_3, check_get_Ki) {
  EXPECT_EQ(controller.getKi(), 0.01);
}

/**
 * Document
 */
TEST(test_controller_4, check_get_Kp) {
  EXPECT_EQ(controller.getKp(), 0.03);
}