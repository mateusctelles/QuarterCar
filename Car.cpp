#include <iostream>
#include "Car.hpp"
#include <cmath>

void Car::setSpring(std::unique_ptr<Spring> spring){
    this->spring = std::move(spring);
}


/*void Car::setTireSpring(Spring* tireSpring){
    this->tireSpring = tireSpring;
}

void Car::setDamper(Damper* damper)
{
    this->damper = damper;
}*/

double Car::CalcRideFreq(){
    return (1/(2*M_PI))*pow((((spring->getStiffness(0) * tireSpring->getStiffness(0)) / (spring->getStiffness(0) + tireSpring->getStiffness(0))) / sprungMass_), 0.5);
}

double Car::CalcSprungNatFreq(){
    return pow((spring->getStiffness()/sprungMass_), 0.5)/(2*M_PI);
}

double Car::CalcSuspDamp(double velocity){
     return damper->getDampingRatio(velocity)*(2*CalcSprungNatFreq()*2*M_PI*sprungMass_);
     //return damper_->getDampingRatio(velocity)*(2*CalcRideFreq()*2*M_PI*sprungMass_);
}


