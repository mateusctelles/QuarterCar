#ifndef SIMULATION_H
#define SIMULATION_H

#include <vector>
#include <string>
#include <iostream>
#include "Car.hpp"
#include "Road.hpp"

class Simulation{
private:

   double t_final=2.0;
   double dt = 0.001;

   std::vector<double> a_u;
   std::vector<double> v_u;
   std::vector<double> y_u;
   std::vector<double> a_s;
   std::vector<double> v_s;
   std::vector<double> y_s;
   std::vector<double> y_g;
   std::vector<double> v_g;

   std::vector<double> F_b;
   std::vector<double> F_spring;
   std::vector<double> F_damper;
   std::vector<double> F_tire;
   std::vector<double> F_tire_damping;
   std::vector<double> delta_ys;
   
   std::vector<double> t;

public:
    Simulation()=default;
    Simulation(double t_final_In, double dtIn);

       
    void Simulate(Car &car, Road &road);

    void Graph();

};

#endif