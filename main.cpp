#include "Simulation.hpp"
#include "Car.hpp"
#include "Road.hpp"
#include "Spring.hpp"
#include "Damper.hpp"
#include "ModelBuilder.hpp"
#include <memory>
#include <iostream>

void programStarter();

int main()
{
    int repeat = 3;
    programStarter();
    ModelBuilder model;

    while (true)
    {
        try
        {
            std::cout << " -----------------------------------------------------------  START ------------------------------------------------------------ \n\n";

            if (repeat == 3)
            {
                model.unitsHandler();
            }
            model.getVehicleParams();
            model.getSimParams();

            Car &car = model.getCar();
            Road *road = model.getRoad();
            Simulation sim = model.getSim();
            model.printAttributes();

            // sim.StaticEquilibrium(car);
            sim.Simulate(car, *road);
            model.printAttributes();

            repeat = sim.Graph();

            if (repeat != 1 && repeat != 3)
            {
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

void programStarter()
{
    std::cout << " **********************************************  QUARTER CAR SIMULATION SOFTWARE *********************************************** \n";
    std::cout << " *                                                                                                                             * \n";
    std::cout << " * By: Mateus Telles.                                                                                                          * \n";
    std::cout << " * Version 1.0.0: Fev/07/2023.                                                                                                 * \n";
    std::cout << " *                                                                                                                             * \n";
    std::cout << " ******************************************************************************************************************************* \n\n\n";
}