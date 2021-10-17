#pragma once
#include <iostream>

namespace ackermann{

class ForwardKinamatics {
    public:
    // --constructors
    ForwardKinamatics(double heading_error = 0, double speed_error = 0) : heading_error_{heading_error}, speed_error_{speed_error} {
        std::cout<<"Constructor for Forwardkinematics class called"<<std::endl;
    }

    // --destructors
    ~ForwardKinamatics(){
        std::cout<<"Destructor for Forwardkinematics class called"<<std::endl;
    }

    private:
    double heading_error_;
    double speed_error_;

}; //--ForwardKinamatics Class
} //namespace ackermann