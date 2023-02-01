#include "Spring.hpp"

LinearSpring::LinearSpring(double stiffness) {
    this->stiffness = stiffness;
}

double LinearSpring::getStiffness(double displacement) {
    return stiffness;
}

void LinearSpring::setStiffness(double stiffness) {
    this->stiffness = stiffness;
}

NonLinearSpring::NonLinearSpring(double stiffness) {
    this->stiffness = stiffness;
}

double NonLinearSpring::getStiffness(double displacement) {
    double staticPosition = 4;
    return stiffness*pow(displacement+staticPosition, 2);
}

void NonLinearSpring::setStiffness(double stiffness) {
    this->stiffness = stiffness;
}


