#pragma once

namespace ackermann{

class Sensor {
    public:
    // -- Getters
    double getActualHeading();
    double getActualSpeed();

    // -- Setters
    void setActualHeading(double);
    void setActualSpeed(double);

    // //--constructors
    // Sensor(double actualHeading=1, double actualSpeed=1) : actual_heading_{actualHeading}, actual_speed_{actualSpeed}{std::cout<<"Constructor for Sensor class called"<<std::endl;};

    // //--Destructor
    // ~Sensor() { std::cout<<"Destructor for Sensor class called"<<std::endl;}

    private:
    // --Atributes
    double actual_heading_;
    double actual_speed_;

}; // Class Sensor
} // namespace ackermann