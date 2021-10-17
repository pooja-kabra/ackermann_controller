#pragma once
#include <iostream>

namespace ackermann{

class Controller {
    public:
    // --setters
    void setGoalHeading(double goalHeading);
    void setGoalSpeed(double goalSpeed);
    
    // --getters
    double getGoalHeading();
    double getGoalSpeed();
    double getKi();
    double getKp();

    // --constructors
    Controller(double goal_heading = 0, double goal_speed = 0, double ki = 0.01, double kp = 0.03) : goal_heading_{goal_heading}, goal_speed_{goal_speed}, ki_{ki}, kp_{kp} {
        std::cout<<"Constructor for Controller class called"<<std::endl;
    };

    // --Destructor
    ~Controller() {
        std::cout<<"Destructor for Controller class called"<<std::endl;
    }


    private:
    double goal_heading_;
    double goal_speed_;
    double ki_;
    double kp_;

}; //--Controller Class
} //--Namespace ackermann