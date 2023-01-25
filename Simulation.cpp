#include "Simulation.hpp"
#include <matplot/matplot.h>

Simulation::Simulation(double t_final_In, double dtIn) : t_final(t_final_In), dt(dtIn) {}

void Simulation::getSineRoadFromUser(){
    double A;
    double w_sin;                
    std::cout<<"\nSine Wave input method selected.";
    std::cout<<"\nDefine the Sine Wave Amplitude [m]: ";
    std::cin>>A; // meters
    std::cout<<"Define the sine frequency [Hz]: ";
    std::cin>>w_sin;
    road_ = new SineRoad(A,w_sin);
}

void Simulation::getSweptSineRoadFromUser(){
    double f_start;
    double f_end;
    double A_swp;                
    std::cout<<"\nSwept Sine input method selected.";
    std::cout<<"\nDefine the start of frequency range [Hz]: ";
    std::cin>>f_start;
    std::cout<<"\nDefine the end of the frequency range [Hz]: ";
    std::cin>>f_end;
    std::cout<<"Define the Amplitude of the swept sine wave [m]: ";
    std::cin>>A_swp; 
    road_ = new SweptSine(A_swp,f_start,f_end); 
}

void Simulation::getRampRoadFromUser(){
    double height;
    double inclination;
    std::cout<<"\nRamp Sequence input method selected.";
    std::cout<<"\nDefine the height of the ramp [m]: ";
    std::cin>>height;
    std::cout<<"\nDefine the inclination of the ramp [m]: ";
    std::cin>>inclination;
    road_ = new RampRoad(height,inclination);
}

void Simulation::getFileRoadFromUser(){
    std::string filename;
    double scaling;
    char filter;
    double windowSize;                
    std::cout<<"\n\nImport File input method selected. File must have a displacement time signal with a time duration equal or greater than the simulation time.";
    std::cout<<"\nWrite the file name with the file extension. If file is in different folder than executable, provide file path.\n";
    std::cout<<"\nFile: ";
    std::cin>>filename;
    std::cout<<"\n--------------------------------------------\n";
    std::cout<<"\nDefine the scaling of the signal. Type '1' if no scaling is desired.\n";
    std::cout<<"\nScaling: ";
    std::cin>> scaling;
    std::cout<<"\n--------------------------------------------\n";
    std::cout<<"\nDo you wish to filter the signal (Moving Average Method)? Type 'y' or 'n': ";
    std::cin>>filter;
    if (filter=='y'){
        std::cout<<"Define the window size [number of samples]: ";
        std::cin>>windowSize;
    }
    road_ = new FileRoad(filename, filter, windowSize, scaling);
}

void Simulation::getSimParams(){

    int inputsim;
    int inptype;
    int changeinp;

    std::cout<<"\n\n--------------------------------------- SIMULATION PARAMETERS -------------------------------------------- \n";
    //std::cout<<"\nSIMULATION PARAMETERS:\n";
    //std::cout<<"\nType '0' if you want to define simulation parameters in the console, type '1' if want to use the parameters in the code, \ntype '2' if want to keep parameters from last run.\n";
    std::cout<<"\n[1] Define simulation parameters in the console\n";
    std::cout<<"[2] Keep parameters from previous run. \n";
    //std::cout<<"[3] Keep parameters from previous run.\n";
    
    std::cout<<"\nSelection: ";
    std::cin>>inputsim;
    std::cout<<"\n--------------------------------------------\n";
    if(inputsim == 3){

        /* // Simulation Parameters
          t_final= 2; 
          dt = 0.0001; 
          std::string input = "swept"; // Select input between "sine", "ramp", "swept", "custom" and "file". 
          
          // Sine Wave Input Parameters
          A = 0.1; // meters
          w_sin = 10; // Hz
          
          // Ramp Input Parameters
          height = 0.1;
          inclination = 0.3;
          sig = '=';

          // Swept Sine Input Parameters
          f_start = 0;
          f_end = 20; 
          A_swp = 0.1;

          // File import Parameters

          filename  = "signal.csv"; // Enter filename. If file is in other path, provide address.
          filter = 'y'; // Choose between "y" for yes, or "n" for no.
          windowSize = 100; // Number of samples
          scaling = 1; // Scales input signal from file. Default 1.0 for no scaling. */
    }

    else if(inputsim==1) {
          std::cout<<"\nInsert Simulation Time [s]: ";
          std::cin>>t_final;
          std::cout<<"\nInsert Step Size [s]: ";
          std::cin>>dt;
          std::cout<<"\n--------------------------------------------";
          std::cout<<"\n[1] Define signal input type and its parameters.";
          std::cout<<"\n[2] Keep the same settings from last run.\n";
          std::cout<<"\nSelection: ";
          std::cin>>changeinp;
          std::cout<<"\n--------------------------------------------";

          if(changeinp==1){

              std::cout<<"\nDefine Input Type: (1) Sine Wave     (2) Swept Sine      (3) Ramp       (4) Import Displacement Signal from File\n";
              std::cout<<"Selection: ";
              std::cin>>inptype;
              std::cout<<"\n--------------------------------------------";

              if(inptype==1){
                  getSineRoadFromUser();                
              }
              else if(inptype==2){ 
                  getSweptSineRoadFromUser();   
              }
              else if(inptype==3){
                  getRampRoadFromUser();
              }
              else if(inptype==4){ 
                  getFileRoadFromUser();
              }
          }
    }
       
    else if(inputsim==2){

    }       

};

void Simulation::Simulate(Car &car){

    std::cout<<"\nStarting Simulation: Level 1";
    double RideFreq= (1/(2*M_PI))*pow(((car.getSpring()->getStiffness() * car.getKT()) / (car.getSpring()->getStiffness() + car.getKT()) / car.getSprungMass()), 0.5);
    double damping = (2*RideFreq*2*M_PI*car.getSprungMass())*car.getDamper()->getDampingCoef();
    std::cout<<"\n\nCalculated Stiffness and Ride Freq";
    
    int numSteps=t_final/dt;
    std::cout<<"\n\nCalculated number of steps: "<<numSteps;
    y_g = std::move(road_->CalcRoad(t_final, dt));
    std::cout<<"\n\nRoad Calculated and moved to y_g";
    //double f_ride = RideFreq;
    //double c_s = damping;
    
    double f_ride = car.CalcRideFreq();
    double c_s = car.CalcSuspDamp();
    
    std::cout<<"\n\nRide Frequency Calculated. Frequency: "<<f_ride<<std::endl;
    std::cout<<"\n\nDamping Ratio Calculated. Value: "<<c_s<<std::endl;


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

    std::cout<<"\nResponse Vectors Initialized\n";
    double delta_ys = 0.0;

    t.resize(numSteps, 0.0);

    std::cout<<"\nTime Vector Initialized\n";

    std::cout<<"\nStarting Simulation: Level 2 - Calculations";    

    // Calculating Outputs: Forces, Displacements, Velocities and Accelerations
    for (int i=0; i<numSteps-1; i++){ 

      delta_ys = y_u[i]-y_s[i];
           
      if (delta_ys < car.travel_limit){
         F_b[i]=0;
        }
      else {
         F_b[i]=car.k_Bstop*(delta_ys-car.travel_limit);
      } 
      
        F_spring[i] = car.getSpring()->getStiffness()*(y_s[i]-y_u[i]);
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

    std::cout<<"\n\nSimulation completed!";  
}    

char Simulation::Graph(){

    std::cout<<"\n\nBuilding Plots. . .";

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
    ylabel("Displacement [m]");
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
   std::cout<<"Want to run a new simulation (y/n)? ";
   char rep;
   std::cin>>rep;

   return rep;
   //save("img/QuarterCar.jpg");

}


