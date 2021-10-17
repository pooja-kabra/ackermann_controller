#pragma once

namespace ackermann{

class RobotKinematics {
    public:
    double getTrackLength();
    double getWheelBase();
    double getWheelRadius();
    double getTuringRadius();
    double getInnerWheelHeading();
    void setInnerWheelHeading(double);
    double getOutterWheelHeading();
    void setOutterWheelHeading(double);
    double getInnerWheelSpeed();
    void setInnerWheelSpeed(double);
    double getOutterWheelSpeed();
    void setOutterWheelSpeed(double);
    double getTheataMax();

    //--contructor
    RobotKinematics(double track_length = 1, double wheel_base = 1, double wheel_radius = 1, double turning_radius = 1, double inner_wheel_heading = 1, double outter_wheel_heading = 1, double inner_wheel_speed = 1, double outer_wheel_speed = 1, double theata_max = 45) : 
    track_length_{track_length}, wheel_base_{wheel_base}, wheel_radius_{wheel_radius}, turning_radius_{turning_radius}, inner_wheel_heading_{inner_wheel_heading}, outter_wheel_heading_{outter_wheel_heading}, inner_wheel_speed_{inner_wheel_speed}, outter_wheel_speed_{outer_wheel_speed}, theata_max_{theata_max}{
        std::cout<<"Constructor for RobotKinematics class called"<<std::endl;
    };

    //--destructor
    ~RobotKinematics(){
        std::cout<<"Destructor for RobotKinematics class called"<<std::endl;
    }

    private:
    double track_length_;
    double wheel_base_;
    double wheel_radius_;
    double turning_radius_;
    double inner_wheel_heading_;
    double outter_wheel_heading_;
    double inner_wheel_speed_;
    double outter_wheel_speed_;
    double theata_max_;

}; //--RobotKinematics Class
} //--Namespace ackermann