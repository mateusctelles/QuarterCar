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
    virtual std::string getRoadName()=0;
};

class SineRoad : public Road{
private:
    double amplitude_=0.1;
    double frequency_=1;
    std::string roadName = "Sine Road";

public:
    SineRoad()=default;
    SineRoad(double amplitude, double frequency);
    inline std::string getRoadName(){ return roadName;}
    std::vector<double> CalcRoad(double simulationTime, double timeStepSize) override;    
};

class SweptSine : public Road{
private:
    std::string roadName = "Swept Sine";
    double amplitude_;
    double frequencyStart;
    double frequencyEnd;
public:
    inline std::string getRoadName(){return roadName;}
    std::vector<double> frequencyVector;
    SweptSine()=default;
    SweptSine(double amplitude, double freq1, double freq2);
    std::vector<double> CalcRoad(double simulationTime, double timeStepSize) override;
};

class RampRoad : public Road{
private:
    std::string roadName = "Ramp Road";
    double height_;
    double inclination_;
    double inclinationRate_;
    int type_;

public:
    RampRoad()=default;
    RampRoad(double heightIn, double inclinationIn, int type);
    RampRoad(double heightIn, double inclinationIn, int type, double inclinationRate);
    
    inline std::string getRoadName(){return roadName;}
    std::vector<double> CalcRoad(double simulationTime, double timeStepSize) override;
};

class FileRoad : public Road{
private:
    std::string roadName = "File Road";
    std::vector<double> file_time; 
    std::vector<double> position;
    std::string filename;
    char filterOption;
    int windowSize;
    double scaling;
    void FilterMA();
    std::vector<double> SignalResample(const std::vector<double>& signal, double targetFrequency, double originalFrequency); 
public:
    FileRoad(std::string fileNameIn="signal.csv", char filterIn='y', int windowSizeIn=100, double scalingIn=1.0);
    inline std::string getRoadName(){return roadName;}
    std::vector<double> CalcRoad(double simulationTime, double timeStepSize) override;         
};

class CurbRoad: public Road{
private:
    double height;
    std::string roadName = "Curb Road";

public:
    CurbRoad(double height);
    inline std::string getRoadName(){return roadName;}
    std::vector<double> CalcRoad(double simulationTime, double timeStepSize) override;
};

#endif