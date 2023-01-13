#include "Simulation.hpp"
#include <matplot/matplot.h>

Simulation::Simulation(double t_final_In, double dtIn) : t_final(t_final_In), dt(dtIn) {}

void Simulation::Simulate(Car &car, Road &road){

    int numSteps=t_final/dt;
    y_g = std::move(road.CalcRoad(t_final, dt));

    double f_ride = car.CalcRideFreq();
    double c_s = car.CalcSuspDamp();

    a_u.resize(numSteps, 0.0);
    v_u.resize(numSteps, 0.0);
    y_u.resize(numSteps, 0.0);
    a_s.resize(numSteps, 0.0);
    v_s.resize(numSteps, 0.0);
    y_s.resize(numSteps, 0.0);
    v_g.resize(numSteps, 0.0);

    F_b.resize(numSteps, 0.0);
    F_spring.resize(numSteps, 0.0);
    F_damper.resize(numSteps, 0.0);
    F_tire.resize(numSteps, 0.0);
    F_tire_damping.resize(numSteps, 0.0);

    double delta_ys = 0.0;

    t.resize(numSteps, 0.0);

    // Calculating Outputs: Forces, Displacements, Velocities and Accelerations
    for (int i=0; i<numSteps-1; i++){ 

      delta_ys = y_u[i]-y_s[i];
           
      if (delta_ys < car.travel_limit){
         F_b[i]=0;
        }
      else {
         F_b[i]=car.k_Bstop*(delta_ys-car.travel_limit);
      } 
      
        F_spring[i] = car.k_w*(y_s[i]-y_u[i]);
        F_damper[i] = c_s*(v_s[i]-v_u[i]);
        F_tire[i] = car.k_t*(y_u[i]-y_g[i]);
        F_tire_damping[i] = car.c_t*(v_u[i]-v_g[i]);

        a_s[i] = (-F_spring[i]-F_damper[i] + F_b[i])/car.m_s; // Acceleration of sprung mass
        a_u[i] = (-F_tire[i]+F_spring[i]+F_damper[i] -F_b[i])/car.m_u; // Acceleration of unsprung mass
        v_s[i+1] = a_s[i]*dt+v_s[i]; // Velocity of sprung mass
        v_u[i+1] = a_u[i]*dt+v_u[i]; // Velocity of unsprung mass
        y_s[i+1] = v_s[i]*dt+y_s[i]; // Position of sprung mass
        y_u[i+1] = v_u[i]*dt+y_u[i]; // Position of unsprung mass
        v_g[i+1] = (y_g[i+1]-y_g[i])/dt; // Velocity of ground displacement

        t[i+1]=(i+1)*dt;

        // std::cout<<y_u[i] << " ; " << i <<  std::endl;
        //std::cout<< i << std::endl;
    }  
}    

void Simulation::Graph(){

    using namespace matplot;
    int wdt = 2; // Line Width
    auto h = figure(true);
    h->name("Quarter Car"); // Figure Name
    h->size(1900,950); // Figure Size
 
    auto ax0 = subplot(3,2,0);
    title("Input: ");
    
    auto p0 = plot(t, y_g);
    p0 -> line_width(wdt);
    p0 -> display_name("2D Road");
    auto lgd0 = legend(on);
    lgd0->font_name("Arial");
    xlabel("Time [s]");
    ylabel("Position [m]");
    hold(on); 

   auto ax1 = subplot(3,2,1);  
   // title("Vertical Position");
   auto p1=plot(t, y_u);
   //->display_name("Unsprung");
   p1 -> line_width(wdt);
   p1 -> display_name("Unsprung");
   hold(on);
   auto p2 = plot(t, y_s);
   p2 -> line_width(wdt);
   p2 -> display_name("Sprung");
   auto lgd = legend(on);
   lgd->font_name("Arial");
   xlabel("Time [s]");
   ylabel("Position [m]");
 
   auto ax2 = subplot(3,2,3);
   //title("Velocity");
   auto p3 = plot(t, v_u);
   p3 -> line_width(wdt);
   p3 -> display_name("Unsprung");
   hold(on);
   auto p4 = plot(t, v_s);
   p4 -> line_width(wdt);
   p4 -> display_name("Sprung");
   auto lgd2 = legend(on);
   lgd2->font_name("Arial");
   xlabel("Time [s]");
   ylabel("Velocity [m/s]");

   auto forces = subplot(3,2,{2,4});
   title("Forces");
   
   auto p7 = plot(t, F_spring);
   p7 -> line_width(wdt);
   p7 -> display_name("Spring Force");
   hold(on);

   auto p8 = plot(t, F_damper);
   p8 -> line_width(wdt);
   p8 -> display_name("Damper Force");
   hold(on);
   
   auto p9 = plot(t, F_b);
   p9 -> line_width(wdt);
   p9 -> display_name("Bumpstop Force");
   hold(on);
   
   auto p10 = plot(t, F_tire);
   p10 -> line_width(wdt);
   p10 -> display_name("Tire Force");
   auto lgd4 = legend(on);
   lgd4->font_name("Arial");
   xlabel("Time [s]");
   ylabel("Force [N]"); 
   
   auto ax3 = subplot(3,2,5);
   //ax3 -> size(500,500);
   //title("Acceleration");
   auto p5 = plot(t, a_u);
   p5 -> line_width(wdt);
   p5 -> display_name("Unsprung");
   hold(on);

   auto p6 = plot(t, a_s);
   p6 -> line_width(wdt);
   p6 -> display_name("Sprung");
   auto lgd3 = legend(on);
   lgd3->font_name("Arial");
   xlabel("Time [s]");
   ylabel("Acceleration [m/s²]"); 

   show();
   //save("img/QuarterCar.jpg");

}


