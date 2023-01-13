#include "Simulation.hpp"
#include "Car.hpp"
#include "Road.hpp"

#include <iostream>

int main()
{
    try
    {
        
        
        
        Car car;
        FileRoad file;
        SineRoad sine;
        Simulation simul;
        
        std::cout << "Starting simulation..." << std::endl;

        simul.Simulate(car, file);
        std::cout << "Simulation finished!" << std::endl;
        simul.Graph();
    }
    catch (std::exception &e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
}