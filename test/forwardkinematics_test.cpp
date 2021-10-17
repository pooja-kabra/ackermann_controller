#include <gtest/gtest.h>
#include "../include/forwardkinematics.hpp"

ackermann::ForwardKinematics fk;

// need to write test case for solve method

/**
 * Document
 */
TEST(test_forwardKinematics_1, check_set_heading_error) {
  fk.setHeadingError(20.54);
  EXPECT_EQ(fk.getHeadingError(), 20.54);

}

/**
 * Document
 */
TEST(test_forwardKinematics_1, check_set_heading_error) {
  fk.setSpeedError(27.54);
  EXPECT_EQ(fk.getSpeedError(), 27.54);

}

// /**
//  * Document
//  */
// TEST(test_forwardKinematics_3, check_calculate_heading_error) {
//   EXPECT_EQ(controller.getKi(), 0.01);
// }

// /**
//  * Document
//  */
// TEST(test_forwardKinematics_4, check_calculate_speed_error) {
//   EXPECT_EQ(controller.getKp(), 0.03);
// }