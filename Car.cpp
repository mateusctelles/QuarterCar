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

void Car::setDamper(Damper* damper)
{
    this->damper_ = damper;
}

double Car::CalcRideFreq(){
    return (1/(2*M_PI))*pow((((spring_->getStiffness(0) * tireStiffness_) / (spring_->getStiffness(0) + tireStiffness_)) / sprungMass_), 0.5);
}

double Car::CalcSuspDamp(double velocity){
     return damper_->getDampingRatio(velocity)*(2*CalcRideFreq()*2*M_PI*sprungMass_);
}

/*void Car::printAttributes(){
    
    std::cout << "\n\n===================================== CURRENT MODEL PARAMETERS =====================================\n\n";
    std::cout << "Suspension Stiffness: " << spring_->getStiffness()/model.getUnit() << " N/mm\n";
    std::cout << "Suspension Damping Ratio: " << dampingRatio_ << " | Damping Coefficient: "<< CalcSuspDamp()<<"Ns/m\n";
    std::cout << "Suspension Travel Limit: " << travel_limit_*unit << " mm\n\n";
    std::cout << "Bumpstop Stiffness: " << k_Bstop_/1000 << " N/mm\n";
    std::cout << "Rebound Stop Stiffness: " << k_Rstop_/1000 << " N/mm\n\n";
    std::cout << "Tire Vertical Stiffness: " << tireStiffness_/1000 << " N/mm\n";
    std::cout << "Tire Damping: " << tireDamping_ << "\n\n";
    std::cout << "Sprung Mass: " << sprungMass_ << " Kg\n";
    std::cout << "Unsprung Mass: " << unsprungMass_ << " Kg\n\n";
    std::cout << "Ride Frequency: " << CalcRideFreq() << " Hz";
    std::cout << "\n\n===================================================================================================\n\n";
}*/
