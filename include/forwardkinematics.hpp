#pragma once
#include <iostream>

namespace ackermann{

class ForwardKinematics {
    public:
    // --methods
    double calculateHeadingError();
    double calculateSpeedError();

    // --setters
    void setHeadingError(double);
    void setSpeedError(double);
    
    // --getters
    double getHeadingError();
    double getSpeedError();

    // --constructors
    ForwardKinematics(double heading_error = 0, double speed_error = 0) : heading_error_{heading_error}, speed_error_{speed_error} {
        std::cout<<"Constructor for Forwardkinematics class called"<<std::endl;
    }

    // --destructors
    ~ForwardKinematics(){
        std::cout<<"Destructor for Forwardkinematics class called"<<std::endl;
    }

    private:
    double heading_error_;
    double speed_error_;

}; //--ForwardKinamatics Class
} //namespace ackermann