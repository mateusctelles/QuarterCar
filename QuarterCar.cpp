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
/*
char exit = 'n';

while(exit == 'n'){
*/


// ------------------------------------------- USER INPUTS -----------------------------------------------
   // Vehicle Parameters

   /*char inptype
   std::cout<< "Type 1 to define vehicle parameters in the console, type 0 to define vehicle parameters in the console"<< std::endl;
   std::cin>>inptype; */
   double m_s = 120; 
   double k_w = 7680;
   double DR = 0.50; 
   double m_u = 30; 
   double k_t = 90000;
   double c_t = 0 ;
   double k_stopper = 900000;
   double travel_limit = 0.80;

   // Simulation Parameters
   double t_final= 2;
   double dt = 0.0001;
   std::string input = "file"; // Select input between "sine", "ramp", "swept", "custom" and "file".
   
   // Sine Wave Input Parameters
   double A = 0.1; // meters
   double w_sin = 10; // Hz
   
   // Ramp Input Parameters
   double height = 0.1;
   double inclination = 0.3;
   char sig = '=';

   // Swept Sine Input Parameters
   double f_start = 0;
   double f_end = 20; 
   double A_swp = 0.1;

   // File import Parameters

   std::string filename  = "signal.csv"; // Enter filename. If file is in other path, provide address.
   char filter = 'y'; // Choose between "y" for yes, or "n" for no.
   int windowSize = 100; // Number of samples
   double scaling = 1; // Scales input signal from file. Default 1.0 for no scaling.

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ------------------------------------- SOFTWARE BACKEND -----------------------------------------------

   //Solving Ride Frequency and Damping Ratio
   double w_ride = 1.000/2.000/M_PI*pow(((k_w*k_t)/(k_w+k_t)/m_s),0.5);
   double c_w= DR*(2*w_ride*2*M_PI*m_s);

   //w_ride = 1/2/pi*((k_w*k_t)/(k_w+k_t)/m_s)^0.5;
   //c_w= 1*(2*w_ride*2*pi*m_s);

   std::cout <<"\nNatural Frequency: " << w_ride << " Hz"<< std::endl;
   std::cout <<"Damping: " << c_w << "\n"<< std::endl;

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
    
    if (input =="file"){
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
    
    else if (input == "sine"){
       for (int i=0; i<numSteps-1; i++){
           y_g[i] = A*std::sin(w_sin*2*M_PI*i*dt);
        } 
    }

    else if (input == "ramp"){
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

    else if (input=="swept"){
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
         F_b[i]=k_stopper*delta_ys[i];
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

    //}
      
   return 0;

}
