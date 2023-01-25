#ifndef CAR_H
#define CAR_H
#include <string>
#include "Spring.hpp"
#include "Damper.hpp"

class Car{
    
friend class Simulation;

private:
    
    // Suspension
    Spring* spring_;          // Suspension Stiffness
    Damper* damper_;           // Suspension Damping
    double DR = 0.50;
    double k_Bstop = 900000; // Bumpstop Stiffness
    double k_Rstop = 900000;  // Rebound Stop Stiffness
    double travel_limit = 0.80; // Suspension travel until bumpstop
    
    // Tire
    double k_t = 90000; // Tire Vertical Stiffness
    double c_t = 0;     // Tire Damping
    // Mass
    double m_s = 120; // Sprung Mass
    double m_u = 30;  // Unsprung Mass

public:
    Car()=default;
    Car(Spring* spring, Damper* damper);
    
    //Getters
    inline Spring* getSpring(){return spring_;}
    inline Damper* getDamper(){return damper_;}
    inline double getDR(){return DR;}
    inline double getKBumpStop(){return k_Bstop;}
    inline double getKReboundStop(){return k_Rstop;}
    inline double getMaxBumpTravel(){return travel_limit;}
    
    inline double getKT(){return k_t;}
    inline double getCT(){return c_t;}

    inline double getSprungMass(){return m_s;}
    inline double getUnsprungMass(){return m_u;}
    
    //Setters
    //inline void setKW(double KW){k_w=KW;}
    inline void setDR(double DampingRatio){DR=DampingRatio;}
    inline void setKBumpstop(double KBumpStop){k_Bstop=KBumpStop;}
    inline void setKReboundStop(double KReboundStop){k_Rstop=KReboundStop;}
    inline void setMaxTravel(double maxTravel){travel_limit=maxTravel;}
    
    inline void setKT(double KT){k_t=KT;}
    inline void setCT(double CT){c_t=CT;}

    inline void setSprungMass(double sprungMass){m_s=sprungMass;}
    inline void setUnsprungMass(double unsprungMass){m_u=unsprungMass;}

    void setSpring(Spring* spring);
    void setDamper(Damper* damper);

    double CalcRideFreq();
    double CalcSuspDamp();

    void printAttributes();   

};

#endif