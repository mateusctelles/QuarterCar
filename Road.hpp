#ifndef ROAD_H
#define ROAD_H

#include <string>
#include <iostream>
#include <cmath>
#include <vector>

class Road{
/*protected:
    std::vector<double> roadLine;*/
public:
    virtual std::vector<double> CalcRoad(double simulationTime, double timeStepSize)=0;
};

class SineRoad : public Road{
private:
    double amplitude_=0.1;
    double frequency_=1;

public:
    SineRoad()=default;
    SineRoad(double amplitude, double frequency);
    std::vector<double> CalcRoad(double simulationTime, double timeStepSize) override;    
};

class SweptSine : public Road{
private:
    double amplitude_=0.1;
    double frequencyStart=0.0;
    double frequencyEnd=10.0;

public:
    SweptSine()=default;
    SweptSine(double amplitude, double freq1, double freq2);
    std::vector<double> CalcRoad(double simulationTime, double timeStepSize) override;
};

class RampRoad : public Road{
private:
    double height_=0.1;
    double inclination_=0.3;

public:
    RampRoad()=default;
    RampRoad(double heightIn, double inclinationIn);

    std::vector<double> CalcRoad(double simulationTime, double timeStepSize) override;
};

class FileRoad : public Road{
private:
    std::vector<double> file_time; 
    std::vector<double> position;
    std::string filename;
    char filterOption;
    int windowSize;
    double scaling;
    void FilterMA(); 
public:
    //FileRoad()=delete;
    FileRoad(std::string fileNameIn="signal.csv", char filterIn='y', int windowSizeIn=100, double scalingIn=1.0);
    std::vector<double> CalcRoad(double simulationTime, double timeStepSize) override;      
    
};

class CurbRoad: public Road{
private:
    double height;

public:
    CurbRoad(double height);
    std::vector<double> CalcRoad(double simulationTime, double timeStepSize) override;
};

#endif