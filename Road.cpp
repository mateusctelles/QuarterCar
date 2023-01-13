#include "Road.hpp"
#include <cmath>
#include <fstream>
#include <sstream>


SineRoad::SineRoad(double amp, double freq): A(amp), f_sin(freq){}

std::vector<double> SineRoad::CalcRoad(double t_final, double dt){
    int numSteps=t_final/dt;
    y_g.resize(numSteps,0.0);
    for (int i=0; i<numSteps; i++){
        y_g[i] = A*std::sin(f_sin*2*M_PI*i*dt);
    }
    
    return y_g;
}


SweptSine::SweptSine(double Amp, double freq1, double freq2): A_swp(Amp), f_start(freq1),f_end(freq2){}

std::vector<double>SweptSine::CalcRoad(double t_final, double dt){
    int numSteps=t_final/dt;
    y_g.resize(numSteps,0.0);
    for (int i=0; i<numSteps-1; i++){
        double tswp=i*dt;
        double fcurr= f_start +(f_end - f_start)*tswp/t_final;
        double sample = A_swp*sin(2*M_PI*fcurr*tswp);
        y_g[i] = sample;
        }

    return y_g;
}


RampRoad::RampRoad(double heightIn, double inclinationIn): height(heightIn), inclination(inclinationIn){}

std::vector<double>RampRoad::CalcRoad(double t_final, double dt){
    
    int numSteps=t_final/dt;
    y_g.resize(numSteps,0.0);
    for (int i=0; i<numSteps-1; i++){
            
        if (y_g[i] < height){
            y_g[i+1] = y_g[i] + inclination*dt;
        }
        else {
            y_g[i] = 0;
        }
    }

    return y_g;
}

FileRoad::FileRoad(std::string fileNameIn, char filterIn, int windowSizeIn, double scalingIn) : filename(fileNameIn), filter(filterIn), scaling(scalingIn), windowSize(windowSizeIn)
{
    std::cout<<"File import input method selected. \n\n";
    std::string line;
    //std::vector<double> y_g(numSteps, 0.0);

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
        throw std::runtime_error("ERROR: File could not be opened!\n\n\n");
    }
}


void FileRoad::FilterMA(){
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

std::vector<double>FileRoad::CalcRoad(double t_final, double dt){
    int numSteps=t_final/dt;
    y_g.resize(numSteps,0.0);
    double original_stepsize = file_time[2] - file_time[1];
    std::vector<double> rsmp_time(numSteps, 0.0);
    int file_nsamples = file_time.size();

    double last_ftime = file_time.back();
    if (last_ftime>=t_final){
        std::cout<<"Parsing finished.\nThe original signal has "<<last_ftime<<" seconds and " <<file_nsamples<< " samples.\n" << std::endl;
    }
    else {
        throw std::runtime_error("ERROR: The simulation time is longer than the time signal. This is currently not supported.\n\n\n");
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
        FilterMA();
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
        throw std::runtime_error("Could not open .csv file");
    }
    
    for (int i=0; i<y_g.size(); i++){
        file <<y_g[i]<< ","<<rsmp_time[i] << std::endl;
    }
    
    file.close();
    std::cout<<"Resampled signal exported to >>resampled_signal.csv<<\n\n\n";
    
    return y_g;

}

