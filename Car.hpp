#ifndef CAR_H
#define CAR_H
#include <string>
#include "Spring.hpp"
#include "Damper.hpp"

class Car
{
    friend class Simulation;

private:
    // Suspension
    Spring *spring_; // Suspension Stiffness
    Damper *damper_; // Suspension Damping
    double dampingRatio_;
    double bumpStopStiffness_;    // Bumpstop Stiffness
    double reboundStopStiffness_; // Rebound Stop Stiffness
    double maxJounceTravel_;        // Suspension travel until bumpstop
    double maxReboundTravel_;
    double staticHeight_;
    // Tire
    double tireStiffness_; // Tire Vertical Stiffness
    double tireDamping_;       // Tire Damping
    // Mass
    double sprungMass_ ;  // Sprung Mass
    double unsprungMass_; // Unsprung Mass

public:
    Car() = default;
    Car(Spring *spring, Damper *damper);

    // Getters
    inline Spring *getSpring() { return spring_; }
    inline Damper *getDamper() { return damper_; }
    inline double getDampingRatio() { return dampingRatio_; }
    inline double getBumpStopStiffness() { return bumpStopStiffness_; }
    inline double getReboundStopStiffness() { return reboundStopStiffness_; }
    inline double getMaxBumpTravel() { return maxJounceTravel_; }
    inline double getStaticHeight(){return staticHeight_;}

    inline double getTireStiffness() { return tireStiffness_; }
    inline double getTireDamping() { return tireDamping_; }

    inline double getSprungMass() { return sprungMass_; }
    inline double getUnsprungMass() { return unsprungMass_; }

    // Setters
    // inline void setKW(double KW){k_w=KW;}
    inline void setDR(double DampingRatio) { dampingRatio_ = DampingRatio; }
    inline void setKBumpstop(double bumpStopStiffness) { bumpStopStiffness_ = bumpStopStiffness; }
    inline void setKReboundStop(double reboundStopStiffness) { reboundStopStiffness_ = reboundStopStiffness; }
    inline void setMaxTravel(double maxTravel) { maxJounceTravel_ = maxTravel; }

    inline void setTireStiffness(double tireStiffness) { tireStiffness_ = tireStiffness; }
    inline void setTireDamping(double tireDamping) { tireDamping_ = tireDamping; }
    inline void setStaticHeight(double staticHeight) { staticHeight_ = staticHeight;}

    inline void setSprungMass(double sprungMass) { sprungMass_ = sprungMass; }
    inline void setUnsprungMass(double unsprungMass) { unsprungMass_ = unsprungMass; }

    void setSpring(Spring *spring);
    void setDamper(Damper *damper);

    double CalcRideFreq();
    double CalcSuspDamp(double velocity);

    void printAttributes();
};

#endif