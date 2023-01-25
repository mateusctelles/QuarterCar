#ifndef MODELBUILDER_HPP
#define MODELBUILDER_HPP
#include "Car.hpp"
#include "Damper.hpp"
#include "Spring.hpp"
#include <string>
#include <iostream>
#include <cmath>
#include <vector>

class ModelBuilder{
private:
    
   // LinearSpring spring;
    //LinearDamper damper;
    
    Spring* spring_;
    Damper* damper_;
    Car car_;

    char repeatsim;
    int paramtype;
    char repeatparam;
    int param;

    double m_s; 
    double k_w;
    double DR; 
    double m_u; 
    double k_t;
    double c_t;
    double k_stopper;
    double travel_limit; 
    double unit=1;

    double getStiffnessFromUser(double unit=1);
    double getSprungMassFromUser(double unit=1);
    double getUnsprungMassFromUser(double unit=1);
    double getDampingRatioFromUser();
    double getTireStiffnessFromUser(double unit=1);
    double getTireDampingFromUser();
    double getStopperStiffnessFromUser(double unit=1);
    double getTravelLimitFromUser(double unit=1);


public:
    ModelBuilder()=default;
    //~ModelBuilder(); 
    void getParams();
    Car getCar() const {return car_;}
    void setCarProperties();
    Spring* getSpring() const;
    Damper* getDamper() const;

};

#endif