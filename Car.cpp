#include <iostream>
#include "Car.hpp"
#include <cmath>

Car::Car(Spring* spring, Damper* damper)
{
    this->spring_ = spring;
    this->damper_ = damper;
}

void Car::setSpring(Spring* spring){
    this->spring_ = spring;
}

void Car::setTireSpring(Spring* tireSpring){
    this->tireSpring = tireSpring;
}

void Car::setDamper(Damper* damper)
{
    this->damper_ = damper;
}

double Car::CalcRideFreq(){
    return (1/(2*M_PI))*pow((((spring_->getStiffness(0) * tireSpring->getStiffness(0)) / (spring_->getStiffness(0) + tireSpring->getStiffness(0))) / sprungMass_), 0.5);
}

double Car::CalcSprungNatFreq(){
    return pow((spring_->getStiffness()/sprungMass_), 0.5)/(2*M_PI);
}

double Car::CalcSuspDamp(double velocity){
     return damper_->getDampingRatio(velocity)*(2*CalcSprungNatFreq()*2*M_PI*sprungMass_);
     //return damper_->getDampingRatio(velocity)*(2*CalcRideFreq()*2*M_PI*sprungMass_);
}


