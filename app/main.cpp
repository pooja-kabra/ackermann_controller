/**
 * @file main.cpp
 * @author Markose Jacob, Pooja Kabra
 * @brief Defines the main file. The program starts from here.
 * @version 2.0
 * @date 2021-10-24
 * @copyright Copyright (c) 2021
 * @section DESCRIPTION
 * 
 * Implemented Ackermann steering controller using a PID controller
 * for Acme robotics.
 * 
 * Ackermann steering helps to reduce slippage as the robot make a turn.
 * The controller gives indepedent wheel heading and wheel speed to each
 * of the front wheels and hence the robot is able to make a smoother turn.
 */


#include <iostream>
#include "../include/sensor.hpp"
#include "../include/robot.hpp"
#include "../include/controller.hpp"
#include "../include/forwardkinematics.hpp"
#include "../include/inversekinematics.hpp"
#include "../include/pbPlots.hpp"
#include "../include/supportLib.hpp"


int main() {
    double goal_heading, goal_speed;
    /* accept goal heading from user */
    std::cout << "Enter the goal heading for the robot " <<
                 "wrt to the world: " << std::endl;
    std::cin >> goal_heading;
    /* accept goal speed from user */
    std::cout << "Enter the goal speed for the robot " <<
                 "(MAXIMUM SPEED IS 16.667 m/s) : " << std::endl;
    std::cin >> goal_speed;
    /* if goal speed is over the limit, ask again until its within limit */
    while (goal_speed > 16.667) {
        std::cout << "Enter the goal speed again for the " <<
                    "robot (MAXIMUM SPEED IS 16.667 m/s) : " << std::endl;
        std::cin >> goal_speed;
    }
    ackermann::Robot robo;  // robot
    ackermann::Sensor sen(0, 0);  // sensor
    ackermann::ForwardKinematics forkin(0, 0);  // fk
    ackermann::InverseKinematics inkin(goal_heading, goal_speed);  // ik

    // controller - goal_heading, goal_speed, kp, ki, kd, time step, robot,
    // fk, ik, turn direction
    ackermann::Controller control(goal_heading, goal_speed, 0.5, 0.1, 0.1,
                                            0.1, robo, forkin, inkin, 's');

    control.setGoalHeading(goal_heading);  // set goal heading
    control.setGoalSpeed(goal_speed);  // set goal speed
    control.solve(sen);  // controller object solves for convergence

    /* read plotting data from sensor */
    std::vector<double> h = sen.actual_heading_record;
    std::vector<double> s = sen.actual_speed_record;
    std::vector<double> t = sen.time_record;

    /* creates actual heading convergence plot in build directory */
    RGBABitmapImageReference *imageRef1 = CreateRGBABitmapImageReference();
    DrawScatterPlot(imageRef1, 600, 400, &t, &h);
    std::vector<double> *pngData1 = ConvertToPNG(imageRef1->image);
    WriteToFile(pngData1, "actual heading(deg) vs time(ms).png");
    DeleteImage(imageRef1->image);

    /* creates actual speed convergence plot in build directory */
    RGBABitmapImageReference *imageRef2 = CreateRGBABitmapImageReference();
    DrawScatterPlot(imageRef2, 600, 400, &t, &s);
    std::vector<double> *pngData2 = ConvertToPNG(imageRef2->image);
    WriteToFile(pngData2, "actual speed(meter per sec) vs time(ms).png");
    DeleteImage(imageRef2->image);

    return 0;
}
