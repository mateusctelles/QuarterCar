#ifndef CAR_H
#define CAR_H
#include <string>
#include "Spring.hpp"
#include "Damper.hpp"
#include <memory>


class Car
{
private:
    // Suspension
    //Spring* spring; // Suspension Stiffness
    std::unique_ptr<Spring> spring;
    std::unique_ptr<Spring> tireSpring ;
    std::unique_ptr<Spring> reboundStopSpring;
    std::unique_ptr<Damper> damper; // Suspension Damping
    std::unique_ptr<Damper> tireDamper ;
    std::unique_ptr<Spring> bumpStopSpring;
    double dampingRatio_;
    double bumpStopStiffness_;    // Bumpstop Stiffness
    double reboundStopStiffness_; // Rebound Stop Stiffness
    double maxJounceTravel_;        // Suspension travel until bumpstop
    double maxReboundTravel_;
    double staticHeight_;
    double tireStaticHeight;
    double suspSpringPreload;
    double sprungMassNaturalFrequency_;
    //double tireHeight=50;
    // Tire
    //double tireStiffness_; // Tire Vertical Stiffness
   // double tireDamping_;       // Tire Damping
    // Mass
    double sprungMass_ ;  // Sprung Mass
    double unsprungMass_; // Unsprung Mass

public:
    Car() =default;
    // Getters
    inline Spring* getSpring() { return spring.get(); }
    inline Spring* getTireSpring(){ return tireSpring.get(); }
    inline Spring* getBumpStopSpring(){ return bumpStopSpring.get();}
    inline Spring* getReboundStopSpring(){ return reboundStopSpring.get();}
    inline Damper* getDamper() { return damper.get(); }
    inline Damper* getTireDamper(){ return tireDamper.get();}
    inline double getDampingRatio() { return dampingRatio_; }
    inline double getStaticHeight(){return staticHeight_;}
    inline double getTireStaticHeight(){ return tireStaticHeight;}

    //inline double getTireStiffness() { return tireStiffness_; }
    //inline double getTireDamping() { return tireDamping_; }

    inline double getSprungMass() { return sprungMass_; }
    inline double getUnsprungMass() { return unsprungMass_; }

    // Setters
    inline void setDR(double DampingRatio) { dampingRatio_ = DampingRatio; }
    //inline void setTireDamping(double tireDamping) { this->tireDamping_ = tireDamping; }
    inline void setStaticHeight(double staticHeight) { staticHeight_ = staticHeight;}
    inline void setTireStaticHeight(double tireStaticHeight) { this->tireStaticHeight = tireStaticHeight;}
    inline void setSprungMassNaturalFrequency(double sprungMassNaturalFrequency_) {this->sprungMassNaturalFrequency_ = sprungMassNaturalFrequency_;}


    inline void setSprungMass(double sprungMass) { sprungMass_ = sprungMass; }
    inline void setUnsprungMass(double unsprungMass) { unsprungMass_ = unsprungMass; }
    
    inline void setTireDamper(std::unique_ptr<Damper>tireDamper){this->tireDamper = std::move(tireDamper);};
    void setSpring(std::unique_ptr<Spring> spring);
    void setDamper(std::unique_ptr<Damper> damper){this->damper = std::move(damper);}
    void setTireSpring(std::unique_ptr<Spring> tireSpring){this->tireSpring = std::move(tireSpring);};;
    void setBumpStopSpring(std::unique_ptr<Spring>  bumpStopSpring){this->bumpStopSpring = std::move(bumpStopSpring);};
    void setReboundStopSpring(std::unique_ptr<Spring> reboundStopSpring){this->reboundStopSpring = std::move(reboundStopSpring);}

    double CalcRideFreq();
    double CalcSprungNatFreq();
    double CalcSuspDamp(double velocity);

    void printAttributes();
};

#endif