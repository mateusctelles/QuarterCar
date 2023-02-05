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

BumpStopSpring::BumpStopSpring(double stiffness) {}
double BumpStopSpring::getStiffness(double relativeDisplacement)
{
    if (relativeDisplacement >= availableBumpTravel)
    {
        // stiffness = 0;
        // std::cout<<"\nStiffness "<<0<<" , because "<<availableBumpTravel<<" bp <"<<relativeDisplacement<<" < rdisp "<<availableReboundTravel<<" rbd";
        // std::cout<<"\nGot no stiffness: "<<stiffness<<" | The relative displacement used is: "<<relativeDisplacement<<" | ";
        return 0;
    }
    else
    {
        // std::cout<<"\nGot rebound stiffness: "<<stiffness<<" | The relative displacement used is: "<<relativeDisplacement<<" | ";
        // std::cout<<"\nRebound Stiffness "<<stiffness<<", because "<<relativeDisplacement<<" rdisp > "<<availableReboundTravel<<" rbd";
        return stiffness;
    }
}

ReboundStopSpring::ReboundStopSpring(double stiffness) {}
double ReboundStopSpring::getStiffness(double relativeDisplacement)
{
    if (relativeDisplacement <= availableReboundTravel)
    {
        // stiffness = 0;
        // std::cout<<"\nStiffness "<<0<<" , because "<<availableBumpTravel<<" bp <"<<relativeDisplacement<<" < rdisp "<<availableReboundTravel<<" rbd";
        // std::cout<<"\nGot no stiffness: "<<stiffness<<" | The relative displacement used is: "<<relativeDisplacement<<" | ";
        return 0;
    }
    else
    {
        // std::cout<<"\nGot rebound stiffness: "<<stiffness<<" | The relative displacement used is: "<<relativeDisplacement<<" | ";
        // std::cout<<"\nRebound Stiffness "<<stiffness<<", because "<<relativeDisplacement<<" rdisp > "<<availableReboundTravel<<" rbd";
        return stiffness;
    }
}

TireSpring::TireSpring(double stiffness) {}

double TireSpring::getStiffness(double relativeTireDisplacement)
{
    if (relativeTireDisplacement <= 0 && relativeTireDisplacement >= -tireHeight)
    {
        return stiffness;
    }
    else if (relativeTireDisplacement < -tireHeight)
    {
        return stiffness + 1000;
        // return stiffness * wheelSpring.getWheelStiffness();
    }
    else
    {
        return 0;
    }
}

void TireSpring::setStiffness(double stiffness)
{
    this->stiffness = stiffness;
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
