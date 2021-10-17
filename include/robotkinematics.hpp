#pragma once

namespace ackermann{

class RobotKinamatics {
    public:
    double getTrackLength();
    double getWheelBase();
    double getWheelRadius();
    double getTuringRadius();
    double getInnerWheelHeading();
    void setInnerWheelHeading(double);
    double getOutterWheelHeading();
    void setOutterWheelHeading(double);
    double getInnerWheelSpped();
    void setInnerWheelSpeed(double);
    double getOutterWheelSpeed();
    void setOutterWheelSpeed(double);
    double getTheataMax();

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

}; //--RobotKinamatics Class
} //--Namespace ackermann