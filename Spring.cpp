#include "Spring.hpp"

LinearSpring::LinearSpring(double stiffness)
{
    this->stiffness = stiffness;
}

double LinearSpring::getStiffness(double displacement)
{
    return stiffness;
}

void LinearSpring::setStiffness(double stiffness)
{
    this->stiffness = stiffness;
}

LinearContactSpring::LinearContactSpring(double stiffness) {}

double LinearContactSpring::getStiffness(double relativeDisplacement)
{
    if (relativeDisplacement <= triggerDistance)
    {
        return stiffness;
    }
    else
    {
        return 0;
    }
}

NonLinearSpring::NonLinearSpring(double stiffness) {}

double NonLinearSpring::getStiffness(double displacement)
{
    double staticPosition = 4;
    return stiffness * pow(displacement + staticPosition, 2);
}

void NonLinearSpring::setStiffness(double stiffness)
{
    this->stiffness = stiffness;
}
