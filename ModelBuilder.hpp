#ifndef MODELBUILDER_HPP
#define MODELBUILDER_HPP
#include "Car.hpp"
#include "Damper.hpp"
#include "Spring.hpp"
#include "Road.hpp"
#include "Simulation.hpp"
#include <string>
#include <iostream>
#include <cmath>
#include <vector>
#include <memory>

class ModelBuilder{

friend class Simulation;

private:
    
   // LinearSpring spring;
    //LinearDamper damper;
    
    //Spring* spring_;
    //Damper* damper_;
    Car car_;
    Simulation sim;
    Road* road_;

    char repeatsim;
    int paramtype;
    char repeatparam;
    int param;
    std::string roadName_;

    int userParam;

    /*int unitType_;
    double dispUnitScaling_;
    double massUnitScaling_;
    double forceUnitScaling_;
    double stiffnessUnitScaling_ ;
    double dampingUnitScaling_ ;
    double gUnitScaling_ = 9810;

    std::string displacementUnit_;
    std::string stiffnessUnit_;
    std::string accelerationUnitSI_ =;
    std::string accelerationUnit = "[G]";
    std::string massUnit_ = "[Kg]";
    std::string dampingUnit_ = "[N.s/mm]";
    std::string forceUnit_ = "[N]";
    std::string velocityUnit_ = "[mm/s]";
    std::string gUnit_ = "[G]";*/

    void getStiffnessFromUser();
    void getSprungMassFromUser();
    void getUnsprungMassFromUser();
    void getDampingRatioFromUser();
    void getTireStiffnessFromUser();
    void getTireDampingFromUser();
    void getBumpStopStiffnessFromUser();
    void getReboundStopStiffnessFromUser();
    void getTravelLimitFromUser();
    void getStaticHeightFromUser();
    void getSpringLengthFromUser();

    
    void getSineRoadFromUser();
    void getCurbRoadFromUser();
    void getSweptSineRoadFromUser();
    void getFileRoadFromUser();
    void getRampRoadFromUser();
    void setSimTotalTime(double t_final);
    void setSimStepSize(double dt);

public:
    ModelBuilder()=default;
    Road* getRoad(){return road_;}
    //~ModelBuilder(); 
    void unitsHandler();
    //void setModelUnit(double unit){unit_= unit;}
    //double getModelUnit(){return unit_;}
    void getVehicleParams();
    void getSimParams();
    Car &getCar() {return car_;}
    Simulation getSim(){return sim;}
    void setCarProperties();
    void printAttributes();
    //Spring* getSpring() const;
    //Damper* getDamper() const;

};

#endif