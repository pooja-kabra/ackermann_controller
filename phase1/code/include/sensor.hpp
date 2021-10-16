#pragma once

namespace ackermann{

class Sensor {
    public:
    // -- Getters
    double getActualHeading();

    //--constructors
    Sensor(double actualHeading=1, double actualSpeed=1) : _actualHeading{actualHeading}, _actualSpeed{actualSpeed}{std::cout<<"Constructor for Sensor class called"<<std::endl;};

    //--Destructor
    ~Sensor() { std::cout<<"Destructor for Sensor class called"<<std::endl;}

    private:
    // --Atributes
    double _actualHeading;
    double _actualSpeed;

}; // Class Sensor
} // namespace ackermann