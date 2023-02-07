#include "Simulation.hpp"
#include "Car.hpp"
#include "Road.hpp"
#include "Spring.hpp"
#include "Damper.hpp"
#include "ModelBuilder.hpp"
#include <memory>

#include <iostream>

int main()

{
int repeat=3;
ModelBuilder model; 

    while (true){
        try
        {   
            if (repeat == 3){
                model.unitsHandler();
            }
            model.getVehicleParams();
            model.getSimParams();

            Car &car = model.getCar();
            Road* road = model.getRoad();     
            Simulation sim = model.getSim();
            model.printAttributes();

            //sim.StaticEquilibrium(car);
            sim.Simulate(car, *road);
            model.printAttributes();
            
            repeat = sim.Graph();

            if (repeat != 1 && repeat != 3) {
                delete road;
                break;
            }
        }
        catch (std::exception &e)
        {
            std::cerr << "Exception: " << e.what() << std::endl;
            return 1;
        }
    }

    return 0;
}