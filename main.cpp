#include "Simulation.hpp"
#include "Car.hpp"
#include "Road.hpp"
#include "Spring.hpp"
#include "Damper.hpp"
#include "ModelBuilder.hpp"

#include <iostream>

int main()
{
char repeat='u';   
    while (true){
        try
        {               
            ModelBuilder model;
            if (repeat == 'u'){
                model.unitsHandler();
            }
            model.getVehicleParams();
            model.getSimParams();

            Car car = model.getCar();
            Road* road= model.getRoad();
            
       
            Simulation sim = model.getSim();
            //sim.getSimParams();

            sim.Simulate(car, *road);
            model.printAttributes();
            
            //sim.Graph();
            repeat = sim.Graph();

            if (repeat != 'y' && repeat != 'u') {
                break;
            }
        }
        catch (std::exception &e)
        {
            std::cerr << "Exception: " << e.what() << std::endl;
            return 1;
        }
    }
}