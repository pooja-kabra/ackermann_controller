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
        if(i>3){
            break;
        }
        i = i+1;
        std::cout<<"\n"<<std::endl;
        std::cout<<"iteration - "<<i<<std::endl;
        pid_heading = 0;
        pid_speed = 0;

        /***
         * 
         * 
         * Controller
         * 
         * 
        ***/

        // Calculating error from 

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
            // std::cout<<"Heading error threshold reached. Done!!!!!!!!0"<<std::endl;
            flag1 = false;
        }
        if(fk.getSpeedError()<0.10 && fk.getSpeedError() > -0.10){
            // std::cout<<"speed error threshold reached. Done!!!!!!!!0"<<std::endl;
            flag2 = false;
        }

        // Calculating PID output for heading
        if(flag1 == true){
            cumilative_error_heading += fk.getHeadingError();
            pid_heading = kp_*(fk.getHeadingError()) + cumilative_error_heading*ki_ + (fk.getHeadingError()-pre_error_heading)*kd_;
            // std::cout<<"PID output for heading : "<<pid_heading<<std::endl;
            pre_error_heading = fk.getHeadingError();
        }

        // Calculating PID output for speed
        if(flag2 == true){
            cumilative_error_speed += fk.getSpeedError();
            pid_speed = kp_*(fk.getSpeedError()) + cumilative_error_speed*ki_ + (fk.getSpeedError()-pre_error_speed)*kd_;  // check if 'D' term has to be divided by dt
            // std::cout<<"PID output for speed : "<<pid_speed<<std::endl;
            pre_error_speed = fk.getSpeedError();
        }

        // // Caping heading and speed increment for each time step
        // if(pid_heading>max_instantaneous_wheel_rotation_){
        //     pid_heading = max_instantaneous_wheel_rotation_;
        // }

        // if(pid_speed>max_instantaneous_speed_increment_){
        //     pid_speed = max_instantaneous_speed_increment_;
        // }
        // std::cout<<"Heading output of PID after caping : " <<pid_heading<<std::endl;
        // std::cout<<"speed output of PID after caping : "<<pid_speed<<std::endl;

        // // Checking if heading and speed is exceeding physical constrains of car and updating actual speed and heading
        // // heading
        // if(sensor.getActualHeading()+pid_heading > car.getThetaMax()){
        //     sensor.setActualHeading(45);
        // }
        // else if(sensor.getActualHeading() + pid_heading < (-1 * car.getThetaMax())){
        //     sensor.setActualHeading(-45);
        // }
        // else
        // {
        //     sensor.setActualHeading(sensor.getActualHeading()+pid_heading);
        // }

        // //speed
        // if(sensor.getActualSpeed()+pid_speed > car.getSpeedMax()){
        //     sensor.setActualSpeed(16.667);
        // }
        // else
        // {
        //     sensor.setActualSpeed(sensor.getActualSpeed()+pid_speed);
        // }
        // std::cout<<"Actual speed : "<<sensor.getActualSpeed()<<std::endl;
        // std::cout<<"Actual heading : "<<sensor.getActualHeading()<<std::endl;
        // // std::cout<<"flag1 : "<<flag1<<std::endl;
        // // std::cout<<"flag2 : "<<flag2<<std::endl;

        // Calculating IK to find heading for each of the front wheels
        // ik.calculateWheelHeadings(sensor.getActualHeading(),sensor.getActualSpeed(), time_step_, car);

        std::cout<<"pid heading : "<<pid_heading<<std::endl;
        std::cout<<"pid speed : "<<pid_speed<<std::endl;
        double inner; double outer;
        inner = ik.calculateWheelHeadings(pid_heading,pid_speed, time_step_, direction, car).inner; //)theta_1,theta_2);
        outer = ik.calculateWheelHeadings(pid_heading,pid_speed, time_step_, direction, car).outer;
        std::cout << "inner heading is: " << inner << " and " << "outer heading is: " << outer << std::endl; 

        // Calculating IK to find speed for each of the front wheels
        ik.calculateWheelSpeeds(pid_heading,pid_speed, time_step_,direction, car); //v_r,v_l);

        // calculateNewRobotHeadingandSpeed(,theta_1,theta_2,v_r,v_l, speed_increment, theta_increment);


        // sensor.setActualSpeed(sensor.getActualSpeed()+speed_increment);
        // sensor.setActualHeading(sensor.getActualHeading()+theta_increment);



        /***
         * 
         * 
         * Modelling
         * 
         * 
        ***/
    }

    // std::cout<<"FINAL HEADING IS : "<<sensor.getActualHeading()<<std::endl;



    

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