#ifndef ROAD_H
#define ROAD_H

#include <string>
#include <iostream>
#include <cmath>
#include <vector>

class Road{
protected:
    std::vector<double> y_g;
public:
    virtual std::vector<double> CalcRoad(double t_final, double dt)=0;
};

class SineRoad : public Road{
private:
    double A=0.1;
    double f_sin=1;

public:
    SineRoad()=default;
    SineRoad(double amp, double freq);
    std::vector<double> CalcRoad(double t_finalIn, double dtIn) override;    
};

class SweptSine : public Road{
private:
    double A_swp=0.1;
    double f_start=0.0;
    double f_end=10.0;

public:
    SweptSine()=default;
    SweptSine(double Amp, double freq1, double freq2);
    std::vector<double> CalcRoad(double t_final, double dt) override;
};

class RampRoad : public Road{
private:
    double height=0.1;
    double inclination=0.3;

public:
    RampRoad()=default;
    RampRoad(double heightIn, double inclinationIn);

    std::vector<double> CalcRoad(double t_final, double dt) override;
};

class FileRoad : public Road{
private:
    std::vector<double> file_time; 
    std::vector<double> position;
    std::string filename;
    char filter;
    int windowSize;
    double scaling;
    void FilterMA(); 
public:
    //FileRoad()=delete;
    FileRoad(std::string fileNameIn="signal.csv", char filterIn='y', int windowSizeIn=100, double scalingIn=1.0);
    std::vector<double> CalcRoad(double t_final, double dt) override;      
    
};

#endif