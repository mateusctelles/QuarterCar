#include "Damper.hpp"

LinearDamper::LinearDamper(double dampingRatio) {
    this->dampingRatio = dampingRatio;
}

double LinearDamper::getDampingRatio(double velocity) {
    return dampingRatio;
}

void LinearDamper::setDampingRatio(double dampingRatio) {
    this->dampingRatio = dampingRatio;
}

NonLinearDamper::NonLinearDamper(double dampingRatio) {
    this->dampingRatio = dampingRatio;
}

double NonLinearDamper::getDampingRatio(double velocity) {
    //if (velocity>-400 & velocity<400){
    //    return dampingRatio * 1.5
   // }
    return dampingRatio;
}

void NonLinearDamper::setDampingRatio(double dampingRatio) {
    this->dampingRatio = dampingRatio;
}


LinearContactDamper::LinearContactDamper(double dampingRatio){
    this->dampingRatio = dampingRatio;
}

double LinearContactDamper::getDampingRatio(double velocity){
    return dampingRatio;
}

void LinearContactDamper::setDampingRatio(double dampingRatio) {
    this->dampingRatio = dampingRatio;
}

void LinearContactDamper::setDampingCoefficient(double dampingCoefficient){
    this-> dampingCoefficient = dampingCoefficient;
}

double LinearContactDamper::getDampingCoefficient(double relativeDisplacement)
{
    if (relativeDisplacement <= triggerDistance)
    {
        return dampingCoefficient;
    }
    else
    {
        return 0;
    }
}

