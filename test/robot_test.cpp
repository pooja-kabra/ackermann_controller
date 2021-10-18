/**
 * @file robotkinematics_test.cpp
 * @author Pooja Kabra, Markose Jacob
 * @brief 
 * @version 0.1
 * @date 2021-10-16
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <gtest/gtest.h>
#include "../include/robot.hpp"

ackermann::Robot rk(5, 6, 7, 8);

/**
 * document
 */
TEST(test_robotkinematics_1, check_set_inner_wheel_heading) {
  rk.setInnerWheelHeading(20.22);
  EXPECT_EQ(rk.getInnerWheelHeading(), 20.22);
}

/**
 *document
 */
TEST(test_robotkinematics_2, check_set_outter_wheel_heading) {
  rk.setOuterWheelHeading(40.52);
  EXPECT_EQ(rk.getOuterWheelHeading(), 40.52);
}

/**
 * document
 */
TEST(test_robotkinematics_3, check_set_inner_wheel_speed) {
  rk.setInnerWheelSpeed(10.85);
  EXPECT_EQ(rk.getInnerWheelSpeed(), 10.85);
}

/**
 * document
 */
TEST(test_robotkinematics_4, check_set_outter_wheel_speed) {
  rk.setOuterWheelSpeed(14.87);
  EXPECT_EQ(rk.getOuterWheelSpeed(), 14.87);
}

/**
 * document
 */
TEST(test_robotkinematics_5, check_get_track_length) {
  EXPECT_EQ(rk.getTrackLength(), 5);
}

/**
 * document
 */
TEST(test_robotkinematics_6, check_get_wheel_base) {
  EXPECT_EQ(rk.getWheelBase(), 6);
}

/**
 * document
 */
TEST(test_robotkinematics_7, check_get_wheel_radius) {
  EXPECT_EQ(rk.getWheelRadius(), 7);
}

/**
 * document
 */
TEST(test_robotkinematics_8, check_get_turning_radius) {
  EXPECT_EQ(rk.getTurningRadius(), 8);
}

/**
 * document
 */
TEST(test_robotkinematics_9, check_get_theata_max) {
  EXPECT_EQ(rk.getThetaMax(), 45);
}