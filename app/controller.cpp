/**
 * @file controller.hpp
 * @author Markose Jacob, Pooja Kabra
 * @version 2.0
 * @date 2021-10-24
 * @copyright Copyright (c) 2021
 * @section Controller class
 * 
 * @brief This is the core of the software which makes use of other classes
 * to calculate the headings and wheel speeds for the front wheels.
 * We first find the error in heading and speeds and the use a PID
 * controller to calculate the increment needed. Using this we use the
 * Ackermann method to calculate the heading and speeds for the front
 * wheels. Finally using the wheel headings and speed we calculate the
 * speed and heading of the robot in global frame
 */

#include "../include/controller.hpp"

/**
 * @brief solves for convergence. Makes use of other classes and
 * methods till the heading and speed converges. This is the 
 * heart of the software. 
 * @param sen sensor
 * @return nil
 */
void ackermann::Controller::solve(Sensor &sen) {
    bool flag_h = true;  // flag for global heading
    bool flag_s = true;  // flag for global speed
    char direction = 'u';
    double head_inner_increment;
    double head_outer_increment;
    double pid_heading, pid_speed, cumilative_error_heading,
    cumilative_error_speed, pre_error_heading, pre_error_speed, i;
    pid_heading = 0;
    pid_speed = 0;
    cumilative_error_speed = 0;
    cumilative_error_heading = 0;
    pre_error_speed = 0;
    pre_error_heading = 0;
    i = 0;  // to count number of iterations

    // flag is true indicates not converged
    while ((flag_h == true) || (flag_s == true)) {
        std::cout << "\n\n------------Iteration " << i
        << "------------------------" << std::endl;
        pid_heading = 0;
        pid_speed = 0;

        /* Calculating error */
        fk.setHeadingError(fk.calculateHeadingError(goal_heading_,
        sen.getActualHeading()));
        fk.setSpeedError(fk.calculateSpeedError(goal_speed_,
        sen.getActualSpeed()));

        /* Checking turn */
        if (fk.getHeadingError() > 0) {
            direction = 'l';
        } else if (fk.getHeadingError() < 0) {
            direction = 'r';
        } else {
            direction = 's';
        }

        /* Checking if heading has converged */
        if (fk.getHeadingError() < 0.8 && fk.getHeadingError() > -0.8) {
            std::cout << "HEADING ERROR WITHIN THRESHHOLD ..."
            << " HEADING CONVERGED." << std::endl;
            flag_h = false;
        }

        /* Checking if speed has converged */
        if (fk.getSpeedError() < 0.10 && fk.getSpeedError() > -0.10) {
            std::cout << "SPEED ERROR WITHIN THRESHHOLD ..."
            <<" SPEED CONVERGED." << std::endl;
            flag_s = false;
        }

        /* Calculating PID output for heading */
        if (flag_h == true) {
            cumilative_error_heading += fk.getHeadingError();
            pid_heading = kp_ * (fk.getHeadingError()) +
            cumilative_error_heading * ki_ + (fk.getHeadingError() -
            pre_error_heading) * kd_;
            pre_error_heading = fk.getHeadingError();
        }

        /* Calculating PID output for speed */
        if (flag_s == true) {
            cumilative_error_speed += fk.getSpeedError();
            pid_speed = kp_ * (fk.getSpeedError()) +
            cumilative_error_speed * ki_ + (fk.getSpeedError() -
            pre_error_speed) * kd_;
            pre_error_speed = fk.getSpeedError();
        }

        /* Calling IK to find heading for each of the front wheels */
        ackermann::InverseKinematics::headings head_res_increment;
        head_res_increment = ik.calculateWheelHeadings(pid_heading, time_step_,
                            direction, car);
        head_inner_increment = head_res_increment.inner;
        head_outer_increment = head_res_increment.outer;

        /* Calling IK to find speed for each of the front wheels */
        ackermann::InverseKinematics::speed spd_res_increment;
        spd_res_increment = ik.calculateWheelSpeeds(pid_heading, pid_speed,
        time_step_, direction, car);

        /* Calling IK to update sensor for new global heading and speed */
        ik.calculateNewRobotHeadingandSpeed(head_inner_increment,
        head_outer_increment, sen, car, time_step_);

        i = i + 1;
    }
}

/**
 * @brief Set the desired heading angle of the robot in the global frame
 *
 * @param double
 * @return nil
 */
void ackermann::Controller::setGoalHeading(double goal_heading) {
    goal_heading_ = goal_heading;
}

/**
 * @brief Set the desired linear speed of the robot
 *
 * @param double
 * @return nil
 */
void ackermann::Controller::setGoalSpeed(double goal_speed) {
    goal_speed_ = goal_speed;
}

/**
 * @brief Get the desired heading angle of the robot in the global frame
 *
 * @param nil
 * @return double
 */
double ackermann::Controller::getGoalHeading() {
    return goal_heading_;
}

/**
 * @brief Get the desired linear speed of the robot
 * 
 * @param nil
 * @return double
 */
double ackermann::Controller::getGoalSpeed() {
    return goal_speed_;
}
