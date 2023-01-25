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
    return (1/(2*M_PI))*pow(((spring_->getStiffness() * k_t) / (spring_->getStiffness() + k_t) / m_s), 0.5);
}

double Car::CalcSuspDamp(){
     return damper_->getDampingCoef()*(2*CalcRideFreq()*2*M_PI*m_s);
}

void Car::printAttributes(){
    
    std::cout << "\n\n===================================== CURRENT MODEL PARAMETERS =====================================\n\n";
    std::cout << "Suspension Stiffness: " << spring_->getStiffness()/1000 << " N/mm\n";
    std::cout << "Suspension Damping Ratio: " << DR << " | Damping Coefficient: "<< CalcSuspDamp()<<"Ns/m\n";
    std::cout << "Suspension Travel Limit: " << travel_limit*1000 << " mm\n\n";
    std::cout << "Bumpstop Stiffness: " << k_Bstop/1000 << " N/mm\n";
    std::cout << "Rebound Stop Stiffness: " << k_Rstop/1000 << " N/mm\n\n";
    std::cout << "Tire Vertical Stiffness: " << k_t/1000 << " N/mm\n";
    std::cout << "Tire Damping: " << c_t << "\n\n";
    std::cout << "Sprung Mass: " << m_s << " Kg\n";
    std::cout << "Unsprung Mass: " << m_u << " Kg\n\n";
    std::cout << "Ride Frequency: " << CalcRideFreq() << " Hz";
    std::cout << "\n\n===================================================================================================\n\n";
}
