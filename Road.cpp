#include "Road.hpp"
#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>

CurbRoad::CurbRoad(double height) : height(height) {}

std::vector<double> CurbRoad::CalcRoad(double simulationTime, double timeStepSize)
{
    std::cout << "\nCreating CurbRoad: Level 1";
    std::cout << "\nHeight defined: " << height;
    int numSteps = simulationTime / timeStepSize;
    double simPart;
    std::vector<double> roadLine(numSteps, 0.0);
    for (int i = 0; i < numSteps; i++)
    {
        simPart = (double)i / numSteps;
        if (simPart <= 0.25)
        {
            roadLine[i] = 0;
            // std::cout<<"\nCondition 1 met. <0.25 "<<simPart;
        }
        else if (simPart > 0.25 && simPart <= 0.50)
        {
            roadLine[i] = height;
            // std::cout<<"\nCondition 2 met. 0.25> i <0.5 "<<simPart;
        }
        else if (simPart > 0.5 && simPart <= 0.75)
        {
            roadLine[i] = 0;
            // std::cout<<"\nCondition 3 met. 0.5> i <0.75 "<<simPart;
        }
        else if (simPart > 0.75 && simPart <= 1.00)
        {
            roadLine[i] = height;
            // std::cout<<"\nCondition 4 met. 0.75> i <1.0 "<<simPart;
        }
    }
    std::cout << "\nRoad: Curb Road Calculated!";
    return roadLine;
}

SineRoad::SineRoad(double amplitude, double frequency) : amplitude_(amplitude), frequency_(frequency) {}

std::vector<double> SineRoad::CalcRoad(double simulationTime, double timeStepSize)
{
    std::cout << "\nCreating Road: Level 1";
    int numSteps = simulationTime / timeStepSize;
    std::cout << "\nGot NumSteps:" << numSteps;

    std::vector<double> roadLine(numSteps, 0.0);
    for (int i = 0; i < numSteps; i++)
    {
        roadLine[i] = amplitude_ * std::sin(frequency_ * 2 * M_PI * i * timeStepSize);
    }

    std::cout << "\nroadLine sucessfully resized to the number: " << roadLine.size();
    return roadLine;
}

SweptSine::SweptSine(double amplitude, double freq1, double freq2) : amplitude_(amplitude), frequencyStart(freq1), frequencyEnd(freq2) {}

std::vector<double> SweptSine::CalcRoad(double simulationTime, double timeStepSize)
{
    std::cout << "\nCalculating SweptSine Road with the frequency range between " << frequencyStart << " - " << frequencyEnd << std::endl;
    int numSteps = simulationTime / timeStepSize;
    std::vector<double> roadLine(numSteps, 0.0);
    for (int i = 0; i < numSteps - 1; i++)
    {
        double time = i * timeStepSize;
        double fcurr = frequencyStart + (frequencyEnd - frequencyStart) * time / simulationTime;
        double sample = amplitude_ * sin(M_PI * fcurr * time);
        frequencyVector.push_back(fcurr);
        roadLine[i] = sample;
    }

    return roadLine;
}

RampRoad::RampRoad(double heightIn, double inclinationIn, int type, double inclinationRate) : height_(heightIn), inclination_(inclinationIn), type_(type), inclinationRate_(inclinationRate) {}
RampRoad::RampRoad(double heightIn, double inclinationIn, int type) : height_(heightIn), inclination_(inclinationIn), type_(type){};
std::vector<double> RampRoad::CalcRoad(double simulationTime, double timeStepSize)
{

    int numSteps = simulationTime / timeStepSize;
    std::vector<double> roadLine(numSteps, 0.0);

    switch (type_)
    {
    case 1:
        for (int i = 0; i < numSteps - 1; i++)
        {

            if (roadLine[i] < height_)
            {
                roadLine[i + 1] = roadLine[i] + inclination_ * height_ * timeStepSize;
                // std::cout<<"\nRoad Line: "<<roadLine[i]<<", height: "<< height_;
            }
            else
            {
                roadLine[i + 1] = 0;
                // std::cout<<"\nRoad Line: "<<roadLine[i]<<", height: "<< height_;
            }
        }
    case 2:
        for (int i = 0; i < numSteps - 1; i++)
        {
            if (roadLine[i] < height_)
            {
                roadLine[i + 1] = roadLine[i] + inclination_ * height_ * timeStepSize;
                // std::cout<<"\nRoad Line: "<<roadLine[i]<<", height: "<< height_;
            }
            else
            {
                roadLine[i + 1] = height_;
                // std::cout<<"\nRoad Line: "<<roadLine[i]<<", height: "<< height_;
            }
        }

    case 3:
        for (int i = 0; i < numSteps - 1; i++)
        {
            if (roadLine[i] < height_)
            {
                roadLine[i + 1] = roadLine[i] + inclination_ * height_ * timeStepSize;
                // std::cout<<"\nInclination (RISE): "<< inclination_;
                //  std::cout<<"\nRoad Line: "<<roadLine[i]<<", height: "<< height_;
            }
            else
            {
                roadLine[i + 1] = 0;
                inclination_ = inclination_ + inclinationRate_ * timeStepSize * i * inclinationRate_;
                // std::cout<<"\nInclination (FALL): "<< inclination_;

                // std::cout<<"\nRoad Line: "<<roadLine[i]<<", height: "<< height_;
            }
        }

    case 4:
        int j;
        for (int i = 0; i < numSteps - 1; i++)
        {
            if (roadLine[i] < height_)
            {
                j = j + 1;
                inclination_ = j * j * timeStepSize * inclinationRate_;
                roadLine[i + 1] = roadLine[i] + inclination_ * inclination_ * height_ * timeStepSize;
                // std::cout<<"\nInclination (RISE): "<< inclination_;
                //  std::cout<<"\nRoad Line: "<<roadLine[i]<<", height: "<< height_;
            }
            else
            {
                j = 0;
                roadLine[i + 1] = 0;
                // inclination_ = inclination_ + inclinationRate_ * timeStepSize * i * inclinationRate_;
                //  std::cout<<"\nInclination (FALL): "<< inclination_;

                // std::cout<<"\nRoad Line: "<<roadLine[i]<<", height: "<< height_;
            }
        }
    }

    return roadLine;
}

FileRoad::FileRoad(std::string fileNameIn, char filterOptionIn, int windowSizeIn, double scalingIn) : filename(fileNameIn), filterOption(filterOptionIn), scaling(scalingIn), windowSize(windowSizeIn)
{
    std::cout << "File import input method selected. \n\n";
    std::string line;
    // std::vector<double> roadLine(numSteps, 0.0);

    std::fstream myFile;
    myFile.open(filename, std::ios::in); // read
    if (myFile.is_open())
    {
        std::cout << "Parsing data from file . . .\n";
        std::string line;

        while (std::getline(myFile, line))
        {
            // std::cout<<line<<std::endl;
            std::stringstream ss(line);
            std::string field;

            /* Uses the getline function to read the first field from the ss stringstream, up to the first comma.
            This reads the time value from the line string.*/
            // std::cout << "\nExtracting time vector";
            std::getline(ss, field, ',');
            // std::cout << "Time Extracted: " << field;

            // Converts the time field to a double and store it in the time vector
            try
            {
                file_time.push_back(std::stod(field));
            }

            catch (const std::invalid_argument &e)
            {
                std::cout << "Error: invalid argument - " << field << std::endl;
            }
            catch (const std::out_of_range &e)
            {
                std::cout << "Error: out of range - " << field << std::endl;
            }

            // Reads the position field and store it in the position vector
            // std::cout << "\nExtracting Displacement Vector";
            std::getline(ss, field, ',');

            try
            {
                position.push_back(std::stod(field));
            }
            catch (const std::invalid_argument &e)
            {
                std::cout << "Error: invalid argument - " << field << std::endl;
            }
            catch (const std::out_of_range &e)
            {
                std::cout << "Error: out of range - " << field << std::endl;
            }
        }
        myFile.close();
    }
    else
    {
        std::cout << "File could not be opened!\n\n\n"
                  << std::endl;
        throw std::runtime_error("ERROR: File could not be opened!\n\n\n");
        std::system("pause");
    }
}

void FileRoad::FilterMA()
{
    std::cout << "Performing Moving Average filtering with window size of " << windowSize << ". . .\n";
    std::vector<double> filteredSignal(position.size());
    // Initialize the running sum with the first window
    double runningSum = 0;
    for (int i = 0; i < windowSize; i++)
    {
        runningSum += position[i];
    }
    for (int i = 0; i < position.size(); i++)
    {
        // Update the running sum
        runningSum += position[i + windowSize] - position[i];

        // Store the average value in the filtered signal
        filteredSignal[i] = runningSum / windowSize;
    }

    for (int i = 0; i < position.size(); i++)
    {
        position[i] = filteredSignal[i];
    }
    std::cout << "Signal Filtered Sucessfully\n\n";
}

std::vector<double> FileRoad::SignalResample(const std::vector<double> &signal, double targetFrequency, double originalFrequency)
{
    int n = signal.size();
    std::vector<double> resampledSignal;
    int frequencyRatio = originalFrequency / targetFrequency;

    for (int i = 0; i < n * frequencyRatio; i++)
    {
        double t = i / targetFrequency;
        int index = (int)t;
        double fraction = t - index;
        if (index + 1 < n)
        {
            double sample = (1.0 - fraction) * signal[index] + fraction * signal[index + 1];
            resampledSignal.push_back(sample);
        }
    }
    return resampledSignal;
}

std::vector<double> FileRoad::CalcRoad(double simulationTime, double timeStepSize)
{

    /*for (int i = 0; i < file_time.size(); i++)
    {
        std::cout << "\nFileTime: " << file_time[i];
    }*/

    int numSteps;
    double original_stepsize = file_time[2] - file_time[1];
    double originalFrequency = 1 / original_stepsize;
    std::cout << "\nOriginal step size: " << original_stepsize << ", Time Step Size:" << timeStepSize;
    if (original_stepsize > timeStepSize)
    {
        timeStepSize = original_stepsize;
        std::cout << "\nInput signal Step Size is greater than user defined Simulation Step Size. This is not allowed.";
        std::cout << "\nThe simulation will use the smallest possible step size, which is the same as the Input Signal Step Size: " << original_stepsize << " [s]" << std::endl;
    }

    numSteps = simulationTime / timeStepSize;

    std::vector<double> roadLine(numSteps, 0.0);
    std::vector<double> rsmp_time(numSteps, 0.0);
    int file_nsamples = file_time.size();

    double last_ftime = file_time.back();
    if (last_ftime >= simulationTime)
    {
        std::cout << "\nParsing finished.\nThe original signal has " << last_ftime << " seconds and " << file_nsamples << " samples.\n"
                  << std::endl;
    }
    else
    {
        throw std::runtime_error("ERROR: The simulation time is longer than the time signal. This is currently not supported.");
        std::cout<<"\nThe input signal duration is: "<<last_ftime<<"which is smaller than "<<simulationTime<<".\n\n";
        std::system("pause");
    }
    for (int i = 0; i < file_nsamples; i++)
    {
        // std::cout <<"Time: "<< file_time[i] << "; Position: "<< position[i] << std::endl;
    }

    // Cutting the imported time signal to fit the used-defined simulation
    std::cout << "Matching file signal size with used-defined simulation time\n\n";
    double total_ftime = 0;
    while (total_ftime < simulationTime)
    {
        total_ftime = total_ftime + original_stepsize;
        // std::cout<<"total_ftime = "<<total_ftime << "; simulationTime = "<<simulationTime<<std::endl;
    }
    double file_nsamples_cut = total_ftime / original_stepsize;
    // Applying moving average filter

    if (filterOption == 'y')
    {
        FilterMA();
    }

    // Resampling and scaling the signal
    std::cout << "Resampling signal . . ." << std::endl;
    std::cout << "The first " << total_ftime << " seconds of the imported signal will be used." << std::endl;
    std::cout << "The number of samples used before the resample is: " << file_nsamples_cut << std::endl;

    // roadLine = SignalResample(position, numSteps);

    for (int i = 0; i < numSteps; i++)
    {
        int factor = std::round(file_nsamples_cut / numSteps);
        int interval = i * factor;
        roadLine[i] = position[interval] * scaling;
        rsmp_time[i] = file_time[i * factor];
        // std::cout <<"Resampled Time: "<< rsmp_time[i] << "; Resampled Position: "<< roadLine[i] <<". Iteration "<< i<< std::endl;
    }

    double resampled_stepsize = rsmp_time[2] - rsmp_time[1];

    std::cout << "\n################################################################### \n\n";
    std::cout << "The input signal was resampled to fit used-defined Step Size of " << timeStepSize << "." << std::endl;
    std::cout << "The original vector size was " << position.size() << ", and now is " << roadLine.size() << std::endl;
    std::cout << "The original step size was " << original_stepsize << ", and the step size is now " << resampled_stepsize << ".\n\n\n"
              << std::endl;

    // Exporting resampled signal to a new .csv file
    std::ofstream file("resampled_signal.csv");
    if (!file.is_open())
    {
        std::cout << "Error opening output .csv file" << std::endl;
        throw std::runtime_error("Could not open .csv file");
    }

    for (int i = 0; i < roadLine.size(); i++)
    {
        file << roadLine[i] << "," << rsmp_time[i] << std::endl;
    }

    file.close();
    std::cout << "Resampled signal exported to >>resampled_signal.csv<<\n\n\n";

    return roadLine;
}
