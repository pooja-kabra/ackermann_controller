/**
 * @file forwardkinematics_test.cpp
 * @author Pooja Kabra, Markose Jacob
 * @brief Unit tests for ForwardKinematics class
 * @version Iteration 1
 * @date 2021-10-16
 * @copyright Copyright (c) 2021
 */

#include <gtest/gtest.h>
#include "../include/forwardkinematics.hpp"

ackermann::ForwardKinematics fk;

// need to write test case for solve method

/**
 * @brief Testing getters and setters for heading_error
 * @return none
 **/
TEST(test_forwardKinematics_1, check_set_heading_error) {
  fk.setHeadingError(20.54);
  EXPECT_EQ(fk.getHeadingError(), 20.54);
}

/**
 * @brief Testing getters and setters for speed_error
 * @return none
 **/
TEST(test_forwardKinematics_2, check_set_speed_error) {
  fk.setSpeedError(27.54);
  EXPECT_EQ(fk.getSpeedError(), 27.54);
}
