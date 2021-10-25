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


// need to write test case for solve method

ackermann::Controller controller;

/**
 * @brief Testing getters and setters for goal_heading
 * @return none
 **/
TEST(test_controller_1, check_set_goal_heading) {
  controller.setGoalHeading(70.3);
  EXPECT_EQ(controller.getGoalHeading(), 70.3);
}

/**
 * @brief Testing getters and setters for goal_speed
 * @return none
 **/
TEST(test_controller_2, check_set_goal_speed) {
  controller.setGoalSpeed(34.36);
  EXPECT_EQ(controller.getGoalSpeed(), 34.36);
}

/**
 * @brief Testing solve method
 * @return none
 **/
TEST(test_controller_2, check_convergence) {
    double goal_heading = 85;
    double goal_speed = 11;
    ackermann::Robot robot;  // robot
    ackermann::Sensor sensor(0, 0);  // sensor
    ackermann::ForwardKinematics forward_kine(0, 0);  // fk
    ackermann::InverseKinematics inverse_kine(goal_heading, goal_speed);  // ik
    // controller - goal_heading, goal_speed, kp, ki, kd, time step, robot, fk, ik, turn direction
    ackermann::Controller controller(goal_heading, goal_speed, 0.5, 0.1, 0.1,
                                            0.1, robot, forward_kine, inverse_kine, 's');

    controller.setGoalHeading(goal_heading);  // set goal heading for controller object
    controller.setGoalSpeed(goal_speed);  // set goal speed for controller object
    controller.solve(sensor);  // controller object solves for convergence

    // EXPECT_NEAR(sensor.getActualHeading(), 85, 0.8);
    EXPECT_NEAR(sensor.getActualSpeed(), 11, 0.1);
}