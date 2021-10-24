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
 * @param double
 */
void ackermann::Controller::solve() {
    bool flag1 = true;
    bool flag2 = true;
    char direction = 'u';
    double pid_heading, pid_speed, cumilative_error_heading,cumilative_error_speed, pre_error_heading, pre_error_speed,i;
    pid_heading = 0;
    pid_speed = 0;
    cumilative_error_speed = 0;
    cumilative_error_heading = 0;
    pre_error_speed = 0;
    pre_error_heading = 0;
    i = 0;

    while((flag1 == true) || (flag2 == true )){
        if(i>50){
            break;
        }
        i = i+1;
        std::cout<<"\n"<<std::endl;
        std::cout<<"iteration - "<<i<<std::endl;
        pid_heading = 0;
        pid_speed = 0;

        // Calculating error
        fk.setHeadingError(fk.calculateHeadingError(goal_heading_,sensor.getActualHeading())); //should be replaced with heading in global frame
        // std::cout<<"Error in heading : "<<fk.getHeadingError()<<std::endl;
        fk.setSpeedError(fk.calculateSpeedError(goal_speed_,sensor.getActualSpeed()));
        // std::cout<<"Error in speed : "<<fk.getSpeedError()<<std::endl;

        // Checking if its a left or right turn
        if (fk.getHeadingError() > 0){
            direction = 'l';
        }
        else if(fk.getHeadingError() < 0){
            direction = 'r';
        }
        else{
            direction = 's';
        }

        if(fk.getHeadingError()<0.02 && fk.getHeadingError() > -0.02){
            std::cout<<"Heading error threshold reached. Done!!!!!!!!0"<<std::endl;
            flag1 = false;
        }
        if(fk.getSpeedError()<0.10 && fk.getSpeedError() > -0.10){
            std::cout<<"speed error threshold reached. Done!!!!!!!!0"<<std::endl;
            flag2 = false;
        }

        // Calculating PID output for heading
        if(flag1 == true){
            cumilative_error_heading += fk.getHeadingError();
            pid_heading = kp_*(fk.getHeadingError()) + cumilative_error_heading*ki_ + (fk.getHeadingError()-pre_error_heading)*kd_;
            pre_error_heading = fk.getHeadingError();
            std::cout<<"Still entering here with flag1 = " <<flag1<<std::endl;
        }

        // Calculating PID output for speed
        if(flag2 == true){
            cumilative_error_speed += fk.getSpeedError();
            pid_speed = kp_*(fk.getSpeedError()) + cumilative_error_speed*ki_ + (fk.getSpeedError()-pre_error_speed)*kd_;  
            pre_error_speed = fk.getSpeedError();
        }

        double head_inner_increment; double head_outer_increment;

        ackermann::InverseKinematics::headings head_res_increment;
        head_res_increment = ik.calculateWheelHeadings(pid_heading,pid_speed, time_step_, direction, car);
        head_inner_increment = head_res_increment.inner;
        head_outer_increment = head_res_increment.outer;
        std::cout << "inner heading increment is: " << head_inner_increment << " and " << "outer heading increment is: " << head_outer_increment << std::endl; 

        double spd_inner_increment; double spd_outer_increment;
        ackermann::InverseKinematics::speed spd_res_increment;

        // Calculating IK to find speed for each of the front wheels
        spd_res_increment = ik.calculateWheelSpeeds(pid_heading,pid_speed, time_step_,direction, car);
        spd_inner_increment = spd_res_increment.inner_speed;
        spd_outer_increment = spd_res_increment.outer_speed;
        std::cout << "inner speed increment is: " << spd_inner_increment << " and " << "outer speed increment is: " << spd_outer_increment << std::endl; 
       
        ik.calculateNewRobotHeadingandSpeed(head_inner_increment, head_outer_increment, spd_inner_increment, spd_outer_increment, sensor, car, time_step_);
}


/**
 * @brief Set the desired heading angle of the robot in the robot frame
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
 * @brief Get the desired heading angle of the robot in the robot frame
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

/**
 * @brief Get integral gain of controller
 * 
 * @return double
 */
double ackermann::Controller::getKi() {
    return ki_;
}

/**
 * @brief Get proportional gain of controller
 * 
 * @return double
 */
double ackermann::Controller::getKp() {
    return kp_;
}