#ifndef SPRINGS_HPP
#define SPRINGS_HPP
#include <string>
#include <iostream>
#include <cmath>
#include <vector>

class Spring {
public:
   // Spring() = default;
    virtual double getStiffness(double displacement = 0) = 0;
    virtual double getFreeLength()=0;
    virtual double getTriggerDistance(){return 0;}
    virtual double getPreload(){return 0;}
    virtual void setPreload(double preload){};
    virtual void setStiffness(double stiffness)=0;
    virtual void setFreeLength(double length)=0;
    virtual void setTriggerDistance(double TriggerDistance){};
};

class LinearSpring : public Spring {
private:
    double stiffness;
    double length;
    double preload;
public:
    LinearSpring() = default;
    LinearSpring(double stiffness);
    double getStiffness(double displacement);
    double getFreeLength(){return length;};
    double getPreload(){return preload;}
    void setPreload(double preload){this->preload = preload;}
    void setStiffness(double stiffness);
    void setFreeLength(double length){this->length = length;}
};

class NonLinearSpring : public Spring {
private:
    double stiffness;
    double length;
    double preload;
public:
    NonLinearSpring() = default;
    NonLinearSpring(double stiffness);
    double getStiffness(double displacement);
    double getFreeLength(){return length;}
    double getPreload(){return preload;}
    void setPreload(double preload){this->preload = preload;}
    void setStiffness(double stiffness);
    void setFreeLength(double length){this->length = length;} 
};

class LinearContactSpring : public Spring{
private:
    double stiffness;
    double freeLength;
    double triggerDistance;
public:
    LinearContactSpring() = default;
    LinearContactSpring(double stiffness);
    double getStiffness(double relativeDisplacement);
    double getFreeLength(){return freeLength;}
    double getTriggerDistance(){return triggerDistance;}
    void setTriggerDistance(double triggerDistance) {this->triggerDistance = triggerDistance;}
    void setStiffness(double stiffness) {this->stiffness = stiffness;}
    void setFreeLength(double length)  {freeLength = length;}
};

#endif