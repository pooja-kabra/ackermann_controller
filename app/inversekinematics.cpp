/**
 * @file inversekinematics.cpp
 * @author Markose Jacob, Pooja Kabra
 * @brief This file defines the Inverse Kinematics class, it calculates 
 *        individual wheel angular speeds and wheel angles w.r.t. robot axisIt provides functions    
 * @version 0.1
 * @date 2021-10-16
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "../include/inversekinematics.hpp"
#include <math.h>
#include <time.h>
#define PI 3.14159265

ackermann::InverseKinematics::headings
ackermann::InverseKinematics::calculateWheelHeadings(double
actual_heading, double dt, char direction, Robot &car) {
    double inner = 0, outer = 0;
    if (direction == 'l') {
        // inner wheel heading
        inner = atan2(2 * car.getWheelBase() * sin(actual_heading * PI / 180),
        2 * car.getWheelBase() * cos(actual_heading * PI / 180) -
        car.getTrackLength() * sin(actual_heading * PI / 180));
        inner = inner * 180 / PI;

        // outer wheel heading
        outer = atan2(2 * car.getWheelBase() * sin(actual_heading * PI / 180), 2 * car.getWheelBase() * cos(actual_heading * PI / 180) + car.getTrackLength() * sin(actual_heading * PI / 180));
        outer = outer * 180 / PI;

        if (inner > car.getThetaIncrPerSecMax() * dt) {
            inner = car.getThetaIncrPerSecMax() * dt;
            actual_heading = ((PI / 2) - atan2((1 / tan(inner * (PI / 180)) + (car.getTrackLength() / 2 * car.getWheelBase())), 1)) * 180 / PI;
            outer = atan2(2 * car.getWheelBase() * sin(actual_heading * PI / 180), 2 * car.getWheelBase() * cos(actual_heading * PI / 180) + car.getTrackLength() * sin(actual_heading * PI / 180));
            outer = outer * 180 / PI;
        }
    } else if (direction == 'r') {
        //inner wheel heading
        inner = atan2(2 * car.getWheelBase() * sin(actual_heading * PI / 180), 2 * car.getWheelBase() * cos(actual_heading * PI / 180) + car.getTrackLength() * sin(actual_heading * PI / 180));
        inner = inner * 180 / PI;

        //outer wheel heading
        outer = atan2(2 * car.getWheelBase() * sin(actual_heading * PI / 180), 2 * car.getWheelBase() * cos(actual_heading * PI / 180) - car.getTrackLength() * sin(actual_heading * PI / 180));
        outer = outer * 180 / PI;

        if (outer > car.getThetaIncrPerSecMax() * dt)
        {
            outer = car.getThetaIncrPerSecMax() * dt;
            actual_heading = ((PI / 2) - atan2((1 / tan(inner * (PI / 180)) - (car.getTrackLength() / 2 * car.getWheelBase())), 1)) * 180 / PI;
            inner = atan2(2 * car.getWheelBase() * sin(actual_heading * PI / 180), 2 * car.getWheelBase() * cos(actual_heading * PI / 180) - car.getTrackLength() * sin(actual_heading * PI / 180));
            inner = outer * 180 / PI;
        }
    }

    else
    {
        inner = 0;
        outer = 0;
    }

    car.setInnerWheelHeading(car.getInnerWheelHeading() + inner);
    car.setOuterWheelHeading(car.getOuterWheelHeading() + outer);

    std::cout << "inner wheel heading : " << car.getInnerWheelHeading() << std::endl;
    std::cout << "outer wheel heading : " << car.getOuterWheelHeading() << std::endl;
    headings head;
    head.inner = inner;
    head.outer = outer;
    return head;
}

ackermann::InverseKinematics::speed
ackermann::InverseKinematics::calculateWheelSpeeds(double actual_heading, double actual_speed, double dt, char direction, Robot &car)
{
    double r = 0, angular_speed = 0, iws = 0, ows = 0, iww = 0, oww = 0;
    double rps_max = dt * car.getRpsMax();
    double wheel_r = car.getWheelRadius();

    // Turning radius
    r = (car.getWheelBase() / sin(actual_heading * PI / 180));
    angular_speed = actual_speed / r;
    iws = angular_speed * (car.getWheelBase() / (sin(car.getInnerWheelHeading() * PI / 180)));
    ows = angular_speed * (car.getWheelBase() / (sin(car.getOuterWheelHeading() * PI / 180)));

    iww = iws / wheel_r; // angular speed in rps for left wheel
    oww = ows / wheel_r; // angular speed in rps for right wheel
    if (direction == 'r')
    {
        if (iww > rps_max)
        {
            iww = rps_max;
            iws = rps_max * wheel_r;
            angular_speed = iws / (car.getWheelBase() / (sin(car.getInnerWheelHeading() * PI / 180)));
            ows = angular_speed * (car.getWheelBase() / (sin(car.getOuterWheelHeading() * PI / 180)));
            oww = ows / wheel_r;
        }
    }
    else if (direction == 'l')
    {
        // std::cout <<"Turn is l" <<std::endl;
        if (oww > rps_max)
        {
            oww = rps_max;
            ows = rps_max * wheel_r;
            angular_speed = ows / (car.getWheelBase() / (sin(car.getOuterWheelHeading() * PI / 180)));
            iws = angular_speed * (car.getWheelBase() / (sin(car.getInnerWheelHeading() * PI / 180)));
            iww = iws / wheel_r;
        }
    }

    else
    {
        // std::cout <<"Going straight" <<std::endl;
        if (angular_speed > rps_max)
        {
            angular_speed = rps_max;
            actual_speed = angular_speed * r;
        }
        oww = angular_speed;
        iww = angular_speed;
        iws = actual_speed;
        ows = actual_speed;
    }

    car.setInnerWheelRps(car.getInnerWheelRps() + iww);
    car.setOuterWheelRps(car.getOuterWheelRps() + oww);
    car.setInnerWheelSpeed(car.getInnerWheelSpeed() + iws);
    car.setOuterWheelSpeed(car.getOuterWheelSpeed() + ows);

    std::cout << "Inner linear speed ::: " << car.getInnerWheelSpeed() << std::endl;
    std::cout << "Outer linear speed ::: " << car.getOuterWheelSpeed() << std::endl;

    speed spd;
    spd.inner_speed = iws;
    spd.outer_speed = ows;

    return spd;
}

void ackermann::InverseKinematics::calculateNewRobotHeadingandSpeed(double inner_heading_incr,
                                                                    double outer_heading_incr, Sensor &sensor, Robot &car, double dt)
{
    double dist_left = 0;
    double dist_right = 0;
    double theta = 0;
    dist_left = car.getInnerWheelSpeed() * dt;
    dist_right = car.getOuterWheelSpeed() * dt;
    // std::cout << "left wheel dis : " << dist_left << std::endl;
    // std::cout << "right wheel dis : " << dist_right << std::endl;
    car.yldot = sin(inner_heading_incr) * dist_left + car.yl;
    car.xldot = cos(inner_heading_incr) * dist_left + car.xl;
    car.xrdot = cos(outer_heading_incr) * dist_right + car.xr;
    car.yrdot = sin(outer_heading_incr) * dist_right + car.yr;
    // std::cout << "yldot : " << car.yldot << std::endl;
    // std::cout << "xldot : " << car.xldot << std::endl;
    // std::cout << "yrdot : " << car.yrdot << std::endl;
    // std::cout << "xrdot : " << car.xrdot << std::endl;
    theta = 180 - 180 / PI * atan2(car.xrdot - car.xldot, car.yrdot - car.yldot);
    std::cout << "Slope : " << theta << std::endl;
    car.yl = car.yldot;
    car.xr = car.xrdot;
    car.xl = car.xldot;
    car.yr = car.yrdot;

    sensor.setActualHeading(theta);

    double robot_speed = (car.getInnerWheelSpeed() + car.getOuterWheelSpeed()) / 2;
    sensor.setActualSpeed(robot_speed);
    std::cout<<"Actual heading of robot is : "<<sensor.getActualHeading()<<std::endl;
    std::cout<<"Actual speed of robot is : "<<sensor.getActualSpeed()<<std::endl;

    sensor.actual_heading_record.push_back(sensor.getActualHeading());
    sensor.actual_speed_record.push_back(sensor.getActualSpeed());
    sensor.time_record.push_back(clock());

    
}