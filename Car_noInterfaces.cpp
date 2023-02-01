#include <iostream>
#include "Car.hpp"
#include <cmath>

double Car::CalcRideFreq(){
    return (1/(2*M_PI))*pow(((k_w * k_t) / (k_w + k_t) / m_s), 0.5);
}

double Car::CalcSuspDamp(){
     return DR*(2*CalcRideFreq()*2*M_PI*m_s);
}

void Car::printAttributes(){
    
    std::cout << "====== Car =======\n";
    std::cout << "Suspension Stiffness: " << k_w << " N/m\n";
    std::cout << "Suspension Damping Ratio: " << DR << "-> Damping [Ns/m]: "<< CalcSuspDamp()<<"\n";
    std::cout << "Suspension Travel Limit: " << travel_limit << " m\n\n";
    std::cout << "Bumpstop Stiffness: " << k_Bstop << " N/m\n";
    std::cout << "Rebound Stop Stiffness: " << k_Rstop << " N/m\n\n";
    std::cout << "Tire Vertical Stiffness: " << k_t << " N/m\n";
    std::cout << "Tire Damping: " << c_t << "\n\n";
    std::cout << "Sprung Mass: " << m_s << " Kg\n";
    std::cout << "Unsprung Mass: " << m_u << " Kg\n\n";
    std::cout << "Ride Frequency: " << CalcRideFreq() << " Kg\n";

}
