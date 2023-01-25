#include "Spring.hpp"

LinearSpring::LinearSpring(double stiffness) {
    this->stiffness = stiffness;
}

double LinearSpring::getStiffness() {
    return stiffness;
}

void LinearSpring::setStiffness(double stiffness) {
    this->stiffness = stiffness;
}

NonLinearSpring::NonLinearSpring(double stiffness) {
    this->stiffness = stiffness;
}

double NonLinearSpring::getStiffness() {
    return stiffness;
}

void NonLinearSpring::setStiffness(double stiffness) {
    this->stiffness = stiffness;
}


