#ifndef SPRINGS_HPP
#define SPRINGS_HPP
#include <string>
#include <iostream>
#include <cmath>
#include <vector>

class Spring {
public:
   // Spring() = default;
    virtual double getStiffness() = 0;
    virtual void setStiffness(double stiffness) = 0;
};

class LinearSpring : public Spring {
private:
    double stiffness;
public:
    LinearSpring() = default;
    LinearSpring(double stiffness);
    double getStiffness();
    void setStiffness(double stiffness);
};

class NonLinearSpring : public Spring {
private:
    double stiffness;
public:
    NonLinearSpring() = default;
    NonLinearSpring(double stiffness);
    double getStiffness();
    void setStiffness(double stiffness);
};

#endif