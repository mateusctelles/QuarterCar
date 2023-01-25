#include "Damper.hpp"

LinearDamper::LinearDamper(double coefficient) {
    this->coefficient = coefficient;
}

double LinearDamper::getDampingCoef() {
    return coefficient;
}

void LinearDamper::setDampingCoef(double coefficient) {
    this->coefficient = coefficient;
}

NonLinearDamper::NonLinearDamper(double coefficient) {
    this->coefficient = coefficient;
}

double NonLinearDamper::getDampingCoef() {
    return coefficient;
}

void NonLinearDamper::setDampingCoef(double coefficient) {
    this->coefficient = coefficient;
}