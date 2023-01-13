#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <matplot/matplot.h>
#include <fstream>
#include <sstream>

int main(){ 

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//

char cont = 'y';
double m_s; 
double k_w;
double DR; 
double m_u; 
double k_t;
double c_t;
double k_stopper;
double travel_limit; 
int inptype; 
char vparamrep;
int inputsim;
int paramtype;
int changeinp;
int next;
char sig;
double t_final= 2; 
double dt = 0.0001;
std::string input; 
double A; 
double w_sin;  
double height;
double inclination;
double f_start;
double f_end; 
double A_swp;
std::string filename;
char filter; 
int windowSize; 
double scaling; 


   std::cout<<" ------------------------------------------------------------------------------------------------------------ \n";
   std::cout<<" |                                        QUARTER CAR SIMULATOR                                             | \n";
   std::cout<<" |                                                                                                          | \n";
   std::cout<<" -- Version 0.2. Release Date: 01/02/2023                                                 By Mateus Telles -- \n";
   std::cout<< " ----------------------------------------------------------------------------------------------------------- \n";
   std::cout<<"                                                                                                          \n";
   std::cout<<"                                                                                                         \n";


while(cont == 'y'){

// ------------------------------------------- USER INPUTS -----------------------------------------------
// Vehicle Parameters
   

   //std::cout<<" ------------------------------------------------------------------------------------------------------------ \n\n";
   std::cout<<" -----------------------------------------------  START ----------------------------------------------- \n\n";
   
   std::cout<<"Do you wish to define vehicle parameters? Type 'y' to yes, or 'n' to keep the previously defined parameters.\n";
   std::cout<<"Selection: ";
   std::cin>>vparamrep;
   if(vparamrep=='y'){
   //std::cout<<"\n------------------------------------------------------------------------------------------------------------\n";
   std::cout<<"---------------------------------------------- VEHICLE PARAMETERS ------------------------------------------ \n";;
   std::cout<<"\nType '1' to define all vehicle parameters in the console.\n" ;
   std::cout<<"Type '2' to change single selected parameters.\n";
   std::cout<<"Type '3' to use all vehicle parameters defined in the code. \n";
   std::cout<<"Type '4' to keep the previous vehicle parameters \n"<< std::endl;
   std::cout<<"Selection: ";
   std::cin>>paramtype; 
   std::cout<<"--------------------------------------------------------\n";
   //std::cout<<"\n----------------------------------------------------------------------------------------------------------\n";
   if(paramtype==1){

        std::cout<<"\n------------------------ OPTION 1 SELECTED: ALL VEHICLE PARAMETERS DEFINITION -----------------------\n";   
        std::cout<< "\nDefine the vehicle quarter sprung mass [Kg]: ";
        std::cin>> m_s;

        std::cout<< "\nDefine the suspension stiffness [K/m]: ";
        std::cin>> k_w;

        std::cout<< "\nDefine the Suspension Damping Ratio: ";
        std::cin>> DR;

        std::cout<< "\nDefine the suspension travel until bumpstop [m]: ";
        std::cin>> travel_limit;
        
        std::cout<< "\nDefine the vehicle quarter unsprung mass [Kg]: ";
        std::cin>> m_u;

        std::cout<< "\nDefine the Tire Vertical Stiffness [N/m]: ";
        std::cin>> k_t;

        std::cout<< "\nDefine the Tire Damping Ns/m: ";
        std::cin>> c_t;

        std::cout<< "\nDefine bumpstop stiffness [N/m]: ";
        std::cin>> k_stopper;
    }

   else if(paramtype==2) {
    char repeat1 = 'y';
    int param;
        while(repeat1=='y'){

            
            std::cout<<"\nWhich parameter you wish to change? \n\n";
            std::cout<<"(1) Sprung Mass [Kg]\n";
            std::cout<<"(2) Suspension Stiffness [N/m]\n";
            std::cout<<"(3) Suspension Damping Ratio\n";
            std::cout<<"(4) Suspension Travel Limit [m]\n";
            std::cout<<"(5) Unsprung Mass [Kg]\n";
            std::cout<<"(6) Tire Vertical Stiffness [N/m]\n";
            std::cout<<"(7) Bumpstop Stiffness [N/m]\n\n";
            std::cout<<"Selection: ";
            std::cin>>param;
            std::cout<<"-----------------------------------------------\n";

            if(param==1){
                    std::cout<< "\nDefine the vehicle quarter sprung mass [Kg]: ";
                    std::cin>>m_s;
                    }

            else if(param==2){
                    std::cout<< "\nDefine the suspension stiffness [K/m]: ";
                    std::cin>>k_w;
                    }

            else if(param==3){
                    std::cout<< "\nDefine the Suspension Damping Ratio: ";
                    std::cin>>DR;
                    }

            else if(param==4){
                    std::cout<< "\nDefine the suspension travel until bumpstop [m]: ";
                    std::cin>>travel_limit;
                    }

            else if(param==5){
                    std::cout<< "\nDefine the vehicle quarter unsprung mass [Kg]: ";
                    std::cin>>m_u;}
                                   
            else if(param==6){
                    std::cout<< "\nDefine the Tire Vertical Stiffness [N/m]: ";
                    std::cin>>k_t;}
                    
            else if(param==7){
                    std::cout<< "\nDefine bumpstop stiffness [N/m]: ";
                    std::cin>>k_stopper;    
                                 
            }
            std::cout<<"\nWant to change other parameter? ('y' or 'n'): ";
            std::cin>> repeat1;
            std::cout<<"--------------------------------------------------\n";
        }
            
    }

   else if(paramtype==3){
        m_s = 120; 
        k_w = 7680; 
        DR = 0.50; 
        m_u = 30; 
        k_t = 90000; 
        c_t = 0; 
        k_stopper = 900000; 
        travel_limit = 0.80; 
         
    }
   }
   std::cout<<"\n\n--------------------------------------- SIMULATION PARAMETERS -------------------------------------------- \n";
   //std::cout<<"\nSIMULATION PARAMETERS:\n";
   //std::cout<<"\nType '0' if you want to define simulation parameters in the console, type '1' if want to use the parameters in the code, \ntype '2' if want to keep parameters from last run.\n";
   std::cout<<"\n[1] Define simulation parameters in the console\n";
   std::cout<<"[2] Use parameters defined in the code. \n";
   std::cout<<"[3] Keep parameters from previous run.\n";

   
   std::cout<<"\nSelection: ";
   std::cin>>inputsim;
   std::cout<<"\n--------------------------------------------\n";
   if(inputsim == 2){

   // Simulation Parameters
   t_final= 2; 
   dt = 0.0001; 
   std::string input = "file"; // Select input between "sine", "ramp", "swept", "custom" and "file". 
   
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
   scaling = 1; // Scales input signal from file. Default 1.0 for no scaling.
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
                std::cout<<"\nSine Wave input method selected.";
                std::cout<<"\nDefine the Sine Wave Amplitude [m]: ";
                std::cin>>A; // meters
                std::cout<<"Define the sine frequency [Hz]: ";
                std::cin>>w_sin;
                
            }
            else if(inptype==2){ 
                std::cout<<"\nSwept Sine input method selected.";
                std::cout<<"\nDefine the start of frequency range [Hz]: ";
                std::cin>>f_start;
                std::cout<<"\nDefine the end of the frequency range [Hz]: ";
                std::cin>>f_end;
                std::cout<<"Define the Amplitude of the swept sine wave [m]: ";
                std::cin>>A_swp;     
            }
            else if(inptype==3){
                std::cout<<"\nRamp Sequence input method selected.";
                std::cout<<"\nDefine the height of the ramp [m]: ";
                std::cin>>height;
                std::cout<<"\nDefine the inclination of the ramp [m]: ";
                std::cin>>inclination;
            }
            else if(inptype==4){ 
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
                    std::cout<<"\n-------------------------------------------------------------------------------------------\n";
                }
            }
        }
    }     
    
    else if(inputsim==3){ 

   }       

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ------------------------------------- SOFTWARE BACKEND -----------------------------------------------

   //Solving Ride Frequency and Damping Ratio
   double w_ride = 1.000/2.000/M_PI*pow(((k_w*k_t)/(k_w+k_t)/m_s),0.5);
   double c_w= DR*(2*w_ride*2*M_PI*m_s);

   //w_ride = 1/2/pi*((k_w*k_t)/(k_w+k_t)/m_s)^0.5;
   //c_w= 1*(2*w_ride*2*pi*m_s);
   std::cout<<"\n-----------------------------------------------------------------------------\n\n";
   std::cout <<"\n                         Natural Frequency: " << w_ride << " Hz"<< std::endl;
   std::cout <<"                         Damping: " << c_w << "\n\n\n"<< std::endl;
   std::cout<<"\n------------------------------------------------------------------------------\n\n";

   // Calculate the number of steps required for the simulation
   int numSteps = t_final/dt;

   // Initializing zero-filled vectors
   std::vector<double> a_u(numSteps, 0.0);
   std::vector<double> v_u(numSteps, 0.0);
   std::vector<double> y_u(numSteps, 0.0);
   std::vector<double> a_s(numSteps, 0.0);
   std::vector<double> v_s(numSteps, 0.0);
   std::vector<double> y_s(numSteps, 0.0);
   std::vector<double> y_g(numSteps, 0.0);
   std::vector<double> v_g(numSteps, 0.0);

   std::vector<double> F_b(numSteps, 0.0);
   std::vector<double> F_spring(numSteps, 0.0);
   std::vector<double> F_damper(numSteps, 0.0);
   std::vector<double> F_tire(numSteps, 0.0);
   std::vector<double> F_tire_damping(numSteps, 0.0);
   std::vector<double> delta_ys(numSteps, 0.0);
   
   std::vector<double> t(numSteps, 0.0);

    // Defining initial conditions
    v_s[0]=0;
    y_s[0]=0;
    v_u[0]=0;
    y_u[0]=0;

    // Building the Input Signal based on the method selected by the user.
    
    if (inptype==4){
        //std::cout<<" Em construção"<<std::endl;

        std::cout<<"File import input method selected. \n\n";
        std::string line;
        //std::vector<double> y_g(numSteps, 0.0);
        std::vector<double> file_time ; //(file_nsamples, 0.0);
        std::vector<double> position;
        std::fstream myFile;
        myFile.open(filename, std::ios::in); //read
        if (myFile.is_open()) {
            std::cout<<"Parsing data from file . . .\n";
            std::string line;
            
            while(std::getline(myFile, line)) {
                    //std::cout<<line<<std::endl;
                    std::stringstream ss(line);
                    std::string field;
                    
                    /* Uses the getline function to read the first field from the ss stringstream, up to the first comma. 
                    This reads the time value from the line string.*/
                    std::getline(ss,field,',');
                    
                    // Converts the time field to a double and store it in the time vector
                    file_time.push_back(std::stod(field));
                    
                    // Reads the position field and store it in the position vector
                    std::getline(ss, field, ',');
                    position.push_back(std::stod(field));
                    }
                myFile.close();	
                }
                else {
                    std::cout << "File could not be opened!\n\n\n" << std::endl;
                    return 1;
                }

                double original_stepsize = file_time[2] - file_time[1];
                std::vector<double> rsmp_time(numSteps, 0.0);
                int file_nsamples = file_time.size();
                
                double last_ftime = file_time.back();
                if (last_ftime>=t_final){
                std::cout<<"Parsing finished.\nThe original signal has "<<last_ftime<<" seconds and " <<file_nsamples<< " samples.\n" << std::endl;
                }
                else {
                    std::cout<<"ERROR: The simulation time is longer than the time signal. This is currently not supported.\n\n\n";
                    return 1;
                }
                for (int i=0;i<file_nsamples;i++){
                    //std::cout <<"Time: "<< file_time[i] << "; Position: "<< position[i] << std::endl;
                }
                        
                //Cutting the imported time signal to fit the used-defined simulation
                std::cout<<"Matching file signal size with used-defined simulation time\n\n";
                double total_ftime = 0;
                while(total_ftime<t_final){
                    total_ftime = total_ftime + original_stepsize;
                    //std::cout<<"total_ftime = "<<total_ftime << "; t_final = "<<t_final<<std::endl;
                }
                double file_nsamples_cut = total_ftime/original_stepsize;

                //Applying moving average filter
                if (filter == 'y'){ 
                    std::cout<<"Performing Moving Average filtering with window size of "<<windowSize<<". . .\n";
                    std::vector<double> filteredSignal(position.size());
                    //Initialize the running sum with the first window
                    double runningSum=0;
                    for (int i=0; i< windowSize; i++){
                        runningSum+=position[i];				
                    }
                    for (int i=0; i<position.size(); i++){
                        //Update the running sum 
                        runningSum+= position[i + windowSize] - position[i];
                        
                        //Store the average value in the filtered signal
                        filteredSignal[i] = runningSum/windowSize;
                    }
                    
                    for(int i = 0; i<position.size(); i++){
                        position[i]=filteredSignal[i];
                    }
                    std::cout<<"Signal Filtered Sucessfully\n\n";
                }

                 //Resample the signal
                std::cout<<"Resampling signal . . ."<<std::endl;
                std::cout<<"The first "<<total_ftime<<" seconds of the imported signal will be used."<<std::endl;
                std::cout<<"The number of samples used before the resample is: "<< file_nsamples_cut<<std::endl;
                
                for (int i = 0; i<numSteps;i++){
                    int factor = std::round(file_nsamples_cut/numSteps);
                    int interval=i*factor;
                    y_g[i] = position[interval];
                    rsmp_time[i] = file_time[i*factor];
                    //std::cout <<"Resampled Time: "<< rsmp_time[i] << "; Resampled Position: "<< y_g[i] <<". Iteration "<< i<< std::endl;
                }
                
                double resampled_stepsize = rsmp_time[2] - rsmp_time[1];
                
                std::cout<<"\n################################################################### \n\n";
                std::cout<<"The input signal was resampled to fit used-defined Step Size of "<<dt<<"."<<std::endl;
                std::cout<<"The original vector size was "<< position.size() <<", and now is " << y_g.size() << std::endl;
                std::cout<<"The original step size was "<< original_stepsize<< ", and the step size is now "<< resampled_stepsize << ".\n\n\n" << std::endl; 
                
                //Exporting resampled signal to a new .csv file
                std::ofstream file("resampled_signal.csv");
                if(!file.is_open()){
                    std::cout<<"Error opening output .csv file"<< std::endl;
                    return 1;
                }
                
                for (int i=0; i<y_g.size(); i++){
                    file <<y_g[i]<< ","<<rsmp_time[i] << std::endl;
                }
                
                file.close();
                std::cout<<"Resampled signal exported to >>resampled_signal.csv<<\n\n\n";
    }
    //--------------------------------------------
    
    else if (inptype == 1){
       for (int i=0; i<numSteps-1; i++){
           y_g[i] = A*std::sin(w_sin*2*M_PI*i*dt);
        } 
    }

    else if (inptype == 3){
        for (int i=0; i<numSteps-1; i++){
            
            if (y_g[i] < height){
                  sig = '<';
                  y_g[i+1] = y_g[i] + inclination*dt;
            }
            else {
                sig = '>';
                y_g[i] = 0;
            }
        //std::cout<<y_g[i]<<sig<<height<<std::endl;
        }
    }

    else if (inptype==2){
        for (int i=0; i<numSteps-1; i++){
            double tswp=i*dt;
            double fcurr= f_start +(f_end - f_start)*tswp/t_final;
            double sample = A_swp*sin(2*M_PI*fcurr*tswp);
            y_g[i] = sample;
        }
    }

    else{
        std::cout<<"Invalid Input Signal Selection. Check the name of the selected input signal.";
        return EXIT_FAILURE;
    }
   
    // Calculating Outputs: Forces, Displacements, Velocities and Accelerations
    for (int i=0; i<numSteps-1; i++){ 

      delta_ys[i] = y_u[i]-y_s[i];
           
      if (delta_ys[i] < travel_limit){
         F_b[i]=0;
        }
      else {
         F_b[i]=k_stopper*(delta_ys[i]-travel_limit);
      } 
      
        F_spring[i] = k_w*(y_s[i]-y_u[i]);
        F_damper[i] = c_w*(v_s[i]-v_u[i]);
        F_tire[i] = k_t*(y_u[i]-y_g[i]);
        F_tire_damping[i] = c_t*(v_u[i]-v_g[i]);

        a_s[i] = (-F_spring[i]-F_damper[i] + F_b[i])/m_s; // Acceleration of sprung mass
        a_u[i] = (-F_tire[i]+F_spring[i]+F_damper[i] -F_b[i])/m_u; // Acceleration of unsprung mass
        v_s[i+1] = a_s[i]*dt+v_s[i]; // Velocity of sprung mass
        v_u[i+1] = a_u[i]*dt+v_u[i]; // Velocity of unsprung mass
        y_s[i+1] = v_s[i]*dt+y_s[i]; // Position of sprung mass
        y_u[i+1] = v_u[i]*dt+y_u[i]; // Position of unsprung mass
        v_g[i+1] = (y_g[i+1]-y_g[i])/dt; // Velocity of ground displacement

        t[i+1]=(i+1)*dt;

        // std::cout<<y_u[i] << " ; " << i <<  std::endl;
        //std::cout<< i << std::endl;
    }    

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//---------------------------------------------- Plot Builder ---------------------------------------------------------------
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

  std::cout<<"Do you wish to run a new simulation? (y/n): "<<std::endl;  
  std::cin>>cont; 
  next = next+1;
}
      
   return 0;

}
