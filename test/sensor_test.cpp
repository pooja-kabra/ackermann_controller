/**
 * @file sensor_test.cpp
 * @author Pooja Kabra, Markose Jacob
 * @brief 
 * @version 0.1
 * @date 2021-10-16
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <gtest/gtest.h>
#include "../include/sensor.hpp"

ackermann::Sensor sensor;

/**
 * This test checks the setters and getters for actual heading of the robot
 */
TEST(test_sensor_1, check_set_actual_heading) {
  
  sensor.setActualHeading(30.3);
  EXPECT_EQ(sensor.getActualHeading(), 30.3);

}

/**
 * This test checks the setters and getters for actual speed of the robot
 */
TEST(test_sensor_2, check_set_actual_speed) {
  
  sensor.setActualSpeed(100.4);
  EXPECT_EQ(sensor.getActualSpeed(), 100.4);

}