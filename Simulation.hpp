#ifndef SIMULATION_H
#define SIMULATION_H

#include <vector>
#include <string>
#include <iostream>
#include "Car.hpp"
#include "Road.hpp"
#include "Spring.hpp"
#include <fstream>
#include <sstream>
#include <cstdlib>

class Simulation
{
private:
    double simulationTime_;
    double timeStepSize_;
    //Road *road_;

    std::vector<double> unsprungMassAcc;
    std::vector<double> unsprungMassAccG;
    std::vector<double> unsprungMassVelocity;
    std::vector<double> unsprungMassDisplacement;
    std::vector<double> sprungMassAcc;
    std::vector<double> sprungMassAccG;
    std::vector<double> sprungMassVelocity;
    std::vector<double> sprungMassDisplacement;

    double unsprungMassWeight;
    double sprungMassWeight;
    double gravity = 9.81;
    double sprungMassDeflection_;
    double unsprungMassDeflection_;
    double sprungMassInitialPosition;
    double unsprungMassInitialPosition;
    double suspSpringDeflection_;
    double tireSpringDeflection_;
    //double tireSpringDeflection;

    std::vector<double> roadLine;
    std::vector<double> roadLineVelocity;
    std::vector<double> roadLineAcceleration;
    std::vector<double> sprungMassPosition;
    std::vector<double> unsprungMassPosition;
    std::vector<double> relativeDisplacementVector;
    std::vector<double> relativeTireDisplacementVector;
    std::vector<double> sprungMassNetForce;
    std::vector<double> unsprungMassNetForce;
    std::vector<double> bumpStopForce;
    std::vector<double> reboundStopForce;
    std::vector<double> stopperForce;
    std::vector<double> springForce;
    std::vector<double> damperForce;
    std::vector<double> tireElasticForce;
    std::vector<double> tireDamperForce;
    std::vector<double> tireDefLimit;
    std::vector<double> springBumpLimit;
    std::vector<double> springReboundLimit;
    std::vector<double> bumpStopStiffness;
    std::vector<double> reboundStopStiffness;
    std::vector<double> displacementAttenuation;
    std::vector<double> accelerationAttenuation;
    std::vector<double> sprungMassDisplacementRMS;
    std::vector<double> sprungMassAccelerationRMS;
    std::vector<double> roadLineDisplacementRMS;
    std::vector<double> roadLineAccelerationRMS;
    std::vector<double> displacementRMSRatio;
    std::vector<double> accelerationRMSRatio;

    double displacementUnitScaling_;
    double stiffnessUnitScaling_;
    double massUnitScaling_;
    double gUnitScaling_;
    double dampingUnitScaling_;
    double forceUnitScaling_;
  
    std::string displacementUnit_;
    std::string stiffnessUnit_;
    std::string accelerationUnitSI_;
    std::string accelerationUnit_;
    std::string massUnit_;
    std::string dampingUnit_;
    std::string velocityUnit_;
    std::string forceUnit_;


    std::vector<double> time;

    //Methods called inside Simulate method.
    void ExportResults(int numSteps, Car &car);
    void StaticEquilibrium(Car &car);
    void InitializeVectors(int numSteps, Car &car);
    std::vector<double> CalculateRMS(std::vector<double> &signal);
    std::vector<double> CalculateSignalRatio(std::vector<double> &inputSignal, std::vector<double> &outputSignal);

public:
    Simulation() = default;
    Simulation(double t_final_In, double dtIn);

    void setSimTotalTime(double simulationTime) { simulationTime_ = simulationTime; }
    void setSimStepSize(double timeStepSize) { timeStepSize_ = timeStepSize; }

    double getDisplacementUnitScaling(){return displacementUnitScaling_;}
    double getStiffnessUnitScaling(){ return stiffnessUnitScaling_;}
    double getMassUnitScaling(){return massUnitScaling_;}
    double getGUnitScaling(){return gUnitScaling_;}
    double getDampingUnitScaling(){return dampingUnitScaling_;}

    double getSuspSpringDeflection(){return suspSpringDeflection_;}
    double getTireSpringDeflection(){return tireSpringDeflection_;}
    double getSprungMassDeflection(){return sprungMassDeflection_;}

    std::string getDisplacementUnit(){return displacementUnit_;}
    std::string getStiffnessUnit(){return stiffnessUnit_;}
    std::string getMassUnit(){return massUnit_;}
    std::string getDampingUnit(){return dampingUnit_;}
    std::string getAccelerationUnitSI(){return accelerationUnitSI_;}
    std::string getAccelerationUnit(){return accelerationUnit_;}
    std::string getVelocityUnit(){return velocityUnit_;}
    std::string getForceUnit(){return forceUnit_;}

    void setDisplacementUnitScaling(double displacementUnitScaling){displacementUnitScaling_=displacementUnitScaling;}
    void setStiffnessUnitScaling(double stiffnessUnitScaling){stiffnessUnitScaling_ = stiffnessUnitScaling;}
    void setMassUnitScaling(double massUnitScaling){massUnitScaling_ = massUnitScaling;}
    void setgUnitScaling(double gUnitScaling){gUnitScaling_ = gUnitScaling;}
    void setDampingUnitScaling(double dampingUnitScaling){dampingUnitScaling_=dampingUnitScaling;}
    void setForceUnitScaling(double forceUnitScaling){forceUnitScaling_=forceUnitScaling;}
    
    void setDisplacementUnit(std::string dispUnit){displacementUnit_= dispUnit;}
    void setStiffnessUnit(std::string stiffUnit){stiffnessUnit_= stiffUnit;}
    void setAccelerationUnit(std::string accUnit){accelerationUnit_= accUnit;}
    void setMassUnit(std::string massUnit){massUnit_= massUnit;}
    void setVelocityUnit(std::string velocityUnit){velocityUnit_ = velocityUnit;}
    void setForceUnit(std::string forceUnit){forceUnit_ = forceUnit;}
    void setDampingUnit(std::string dampingUnit){dampingUnit_=dampingUnit;}

    void Simulate(Car &car, Road &road);


    int Graph();
};

#endif