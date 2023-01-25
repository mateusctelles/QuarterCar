#include "Simulation.hpp"
#include "Car.hpp"
#include "Road.hpp"
#include "Spring.hpp"
#include "Damper.hpp"
#include "ModelBuilder.hpp"

#include <iostream>

int main()
{
char repeat='y';   
    while (repeat=='y'){
        try
        {               
            ModelBuilder model;
            model.getParams();
            Car car = model.getCar();
            car.printAttributes();
       
            Simulation sim;
            sim.getSimParams();
            sim.Simulate(car);
            //sim.Graph();
            repeat = sim.Graph();
        }
        catch (std::exception &e)
        {
            std::cerr << "Exception: " << e.what() << std::endl;
            return 1;
        }
    }
}