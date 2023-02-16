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
#include <fftw3.h>

class Simulation
{
private:
    double simulationTime_;
    double timeStepSize_;
    // Road *road_;

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
    double availableBumpTravel_;
    double availableReboundTravel_;
    double abruptness;
    double sprungMassAccRMS;
    double airBorneTime;
    // double tireSpringDeflection;

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

    std::vector<double> frequencies;
    std::vector<double> cutFrequencies{0};
    std::vector<double> accelerationPSD;
    std::vector<double> cutPSD{0};

    std::vector<double> sprungMassSnap;
    std::vector<double> roadAccPSD;
    std::vector<double> roadDisplacementPSD;

    std::vector<double> displacementRMSRatio;
    std::vector<double> accelerationRMSRatio;
    std::vector<double> accelerationDelay;
    std::vector<double> displacementDelay;
    std::vector<double> sprungDispDelay;
    std::vector<double> sprungDisplacementPSD;
    std::vector<double> sprungMassJerk;

    std::vector<double> displacementRMSRatioCompare;
    std::vector<double> accelerationRMSRatioCompare;
    std::vector<double> accelerationDelayCompare;
    std::vector<double> displacementDelayCompare;
    std::vector<double> sprungDispDelayCompare;
    std::vector<double> sprungDisplacementPSDCompare;
    std::vector<double> sprungMassJerkCompare;
    std::vector<double> sprungMassSnapCompare;
    std::vector<double> accelerationPSDCompare;
    std::vector<double> sprungMassAccGCompare;
    std::vector<double> sprungMassDisplacementCompare;
    std::vector<double> sprungMassPositionCompare;

    double displacementUnitScaling_;
    double stiffnessUnitScaling_;
    double massUnitScaling_;
    double gUnitScaling_;
    double dampingUnitScaling_;
    double forceUnitScaling_;
    double frequencyStart_;
    double frequencyEnd_;

    std::string displacementUnit_;
    std::string stiffnessUnit_;
    std::string accelerationUnitSI_;
    std::string accelerationUnit_;
    std::string massUnit_;
    std::string dampingUnit_;
    std::string velocityUnit_;
    std::string forceUnit_;

    int numSteps_;

    std::vector<double> time;

    // Methods called inside Simulate method.
    double AirBorneTime();
    void ExportResults(int numSteps, Car &car);
    void StaticEquilibrium(Car &car);
    void InitializeVectors(int numSteps, Car &car);
    void CopyPlots();
    std::vector<double> CalculateRMS(const std::vector<double> &psd);
    std::vector<double> CalculateSignalRatio(std::vector<double> &inputSignal, std::vector<double> &outputSignal);
    void ExportMetrics();
    double CalculateAbruptness(const std::vector<double>& sprungMassAcceleration, double duration);
    double CalcTimeAccRMS(const std::vector<double>& data); 
    void ClearCSV(const std::string& filename);
    

public:
    Simulation() = default;
    Simulation(double t_final_In, double dtIn);

    inline void setSimTotalTime(double simulationTime) { simulationTime_ = simulationTime; }
    inline void setSimStepSize(double timeStepSize) { timeStepSize_ = timeStepSize; }
    inline void setFrequencyStart(double frequencyStart = 0) { frequencyStart_ = frequencyStart; }
    inline void setFrequencyEnd(double frequencyEnd = 0) { frequencyEnd_ = frequencyEnd; }

    inline double getDisplacementUnitScaling() { return displacementUnitScaling_; }
    inline double getStiffnessUnitScaling() { return stiffnessUnitScaling_; }
    inline double getMassUnitScaling() { return massUnitScaling_; }
    inline double getGUnitScaling() { return gUnitScaling_; }
    inline double getDampingUnitScaling() { return dampingUnitScaling_; }

    inline double getSuspSpringDeflection() { return suspSpringDeflection_; }
    inline double getTireSpringDeflection() { return tireSpringDeflection_; }
    inline double getSprungMassDeflection() { return sprungMassDeflection_; }

    inline std::string getDisplacementUnit() { return displacementUnit_; }
    inline std::string getStiffnessUnit() { return stiffnessUnit_; }
    inline std::string getMassUnit() { return massUnit_; }
    inline std::string getDampingUnit() { return dampingUnit_; }
    inline std::string getAccelerationUnitSI() { return accelerationUnitSI_; }
    inline std::string getAccelerationUnit() { return accelerationUnit_; }
    inline std::string getVelocityUnit() { return velocityUnit_; }
    inline std::string getForceUnit() { return forceUnit_; }

    void setDisplacementUnitScaling(double displacementUnitScaling) { displacementUnitScaling_ = displacementUnitScaling; }
    void setStiffnessUnitScaling(double stiffnessUnitScaling) { stiffnessUnitScaling_ = stiffnessUnitScaling; }
    void setMassUnitScaling(double massUnitScaling) { massUnitScaling_ = massUnitScaling; }
    void setgUnitScaling(double gUnitScaling) { gUnitScaling_ = gUnitScaling; }
    void setDampingUnitScaling(double dampingUnitScaling) { dampingUnitScaling_ = dampingUnitScaling; }
    void setForceUnitScaling(double forceUnitScaling) { forceUnitScaling_ = forceUnitScaling; }

    void setDisplacementUnit(std::string dispUnit) { displacementUnit_ = dispUnit; }
    void setStiffnessUnit(std::string stiffUnit) { stiffnessUnit_ = stiffUnit; }
    void setAccelerationUnit(std::string accUnit) { accelerationUnit_ = accUnit; }
    void setMassUnit(std::string massUnit) { massUnit_ = massUnit; }
    void setVelocityUnit(std::string velocityUnit) { velocityUnit_ = velocityUnit; }
    void setForceUnit(std::string forceUnit) { forceUnit_ = forceUnit; }
    void setDampingUnit(std::string dampingUnit) { dampingUnit_ = dampingUnit; }

    void Simulate(Car &car, Road &road);
    std::vector<double> ComputePSD(const std::vector<double> &time_series, int N, double sample_time);
    std::vector<double> ComputeDelay(const std::vector<double> &PSD, const std::vector<double> &frequencies);
    void printStaticEquilibriumResults();
    void printMetrics();

    int Graph();
};

#endif