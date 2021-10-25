/**
 * @file controller.hpp
 * @author Markose Jacob, Pooja Kabra
 * @brief This file defines the controller class
 * @version 0.1
 * @date 2021-10-16
 * @copyright Copyright (c) 2021
 */

#include "../include/controller.hpp"

/**
 * @brief This method makes the output heading angle of the robot in the robot frame
 *        and its linear speed converge to goal values
 *
 * @param sen sensor
 */
void ackermann::Controller::solve(Sensor &sen) {
    bool flag_h = true;  // flag for global heading
    bool flag_s = true;  // flag for global speed
    char direction = 'u';
    double pid_heading, pid_speed, cumilative_error_heading,         // pid variables
    cumilative_error_speed, pre_error_heading, pre_error_speed, i;   // and iteration counter
    pid_heading = 0;
    pid_speed = 0;
    cumilative_error_speed = 0;
    cumilative_error_heading = 0;
    pre_error_speed = 0;
    pre_error_heading = 0;
    i = 0;

    while ((flag_h == true) || (flag_s == true)) {  // flag is true indicates not converged
        std::cout << "\n\n------------Iteration " << i << "------------------------" << std::endl;
        pid_heading = 0;
        pid_speed = 0;

        /* Calculating error */
        fk.setHeadingError(fk.calculateHeadingError(goal_heading_, sen.getActualHeading()));
        std::cout << "Error in heading(deg): " << fk.getHeadingError() << std::endl;
        fk.setSpeedError(fk.calculateSpeedError(goal_speed_, sen.getActualSpeed()));
        std::cout  << "Error in speed(m/s): " << fk.getSpeedError() << std::endl;

        /* Checking turn */
        if (fk.getHeadingError() > 0) {
            direction = 'l';
        } else if (fk.getHeadingError() < 0) {
            direction = 'r';
        } else {
            direction = 's';
        }

        if (fk.getHeadingError() < 0.8 && fk.getHeadingError() > -0.8) {
            std::cout << "HEADING ERROR WITHIN THRESHHOLD ... HEADING CONVERGED." << std::endl;
            flag_h = false;
        }
        if (fk.getSpeedError() < 0.10 && fk.getSpeedError() > -0.10) {
            std::cout << "SPEED ERROR WITHIN THRESHHOLD ... SPEED CONVERGED." << std::endl;
            flag_s = false;
        }

        /* Calculating PID output for heading */
        if (flag_h == true) {
            cumilative_error_heading += fk.getHeadingError();
            pid_heading = kp_ * (fk.getHeadingError()) +
            cumilative_error_heading * ki_ + (fk.getHeadingError()- pre_error_heading) * kd_;
            pre_error_heading = fk.getHeadingError();
        }

        /* Calculating PID output for speed */
        if (flag_s == true) {
            cumilative_error_speed += fk.getSpeedError();
            pid_speed = kp_ * (fk.getSpeedError()) +
            cumilative_error_speed * ki_ + (fk.getSpeedError() - pre_error_speed) * kd_;
            pre_error_speed = fk.getSpeedError();
        }

        /* Calling IK to find heading for each of the front wheels */
        double head_inner_increment;
        double head_outer_increment;

        ackermann::InverseKinematics::headings head_res_increment;
        head_res_increment = ik.calculateWheelHeadings(pid_heading, time_step_,
                            direction, car);
        head_inner_increment = head_res_increment.inner;
        head_outer_increment = head_res_increment.outer;
        std::cout << "Inner heading increment(deg): " << head_inner_increment <<
          " and " << "outer heading increment(deg): "<< head_outer_increment << std::endl;

        /* Calling IK to find speed for each of the front wheels */
        double spd_inner_increment;
        double spd_outer_increment;

        ackermann::InverseKinematics::speed spd_res_increment;
        spd_res_increment = ik.calculateWheelSpeeds(pid_heading, pid_speed,
        time_step_, direction, car);
        spd_inner_increment = spd_res_increment.inner_speed;
        spd_outer_increment = spd_res_increment.outer_speed;
        std::cout << "Inner speed increment(m/s): " << spd_inner_increment <<
          " and " << "Outer speed increment(m/s): " << spd_outer_increment << std::endl;

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
 */
void ackermann::Controller::setGoalHeading(double goal_heading) {
    goal_heading_ = goal_heading;
}

/**
 * @brief Set the desired linear speed of the robot
 *
 * @param double
 */
void ackermann::Controller::setGoalSpeed(double goal_speed) {
    goal_speed_ = goal_speed;
}

/**
 * @brief Get the desired heading angle of the robot in the global frame
 *
 * 
 * @return double
 */
double ackermann::Controller::getGoalHeading() {
    return goal_heading_;
}

/**
 * @brief Get the desired linear speed of the robot
 * 
 * @return double
 */
double ackermann::Controller::getGoalSpeed() {
    return goal_speed_;
}
