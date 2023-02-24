#include "Simulation.hpp"
#include <matplot/matplot.h>
// #include <armadillo/armadillo.h>

Simulation::Simulation(double t_final_In, double dtIn) : simulationTime_(t_final_In), timeStepSize_(dtIn) {}

void Simulation::ExportResults(int numSteps, Car &car)
{
  std::ofstream file("SimulationOutput.csv");
  if (!file.is_open())
  {
    std::cout << "Error opening output .csv file" << std::endl;
    throw std::runtime_error("Could not open .csv file");
  }
  file << "Time, RoadLine, SprungMassDisp, Sprung Mass Acc,Unsprung Mass Acc,BumpStopForce,ReboundStop Force,StopperForce,Tire Force,Spring Force,Damper Force, Tire Damper Force, BumpStop Stiffness, ReboundStop Stiffness, availableBumpTravel, Relative Displacement, availableReboundTravel, ";
  // file << "roadDispRMS, sprungDispRMS, roadAccRMS, sprungAccRMS, dispRMSRatio, accRMSRatio, PSDFrequencies" << std::endl;
  for (int i = 0; i < numSteps; i++)
  {
    file << time[i] << "," << roadLine[i] << "," << sprungMassDisplacement[i] << "," << sprungMassAccG[i] << "," << unsprungMassAccG[i] << "," << bumpStopForce[i] << "," << reboundStopForce[i] << "," << stopperForce[i] << "," << tireElasticForce[i] << ","
         << springForce[i] << "," << damperForce[i] << "," << tireDamperForce[i] << "," << bumpStopStiffness[i] << "," << reboundStopStiffness[i] << "," << car.getBumpStopSpring()->getTriggerDistance() << ","
         << relativeDisplacementVector[i] << "," << car.getReboundStopSpring()->getTriggerDistance() << std::endl;
  }

  file.close();
}

void Simulation::InitializeVectors(int numSteps, Car &car)
{
  numSteps_ = numSteps;
  unsprungMassAcc.resize(numSteps, 0.0);
  unsprungMassAccG.resize(numSteps, 0.0);
  unsprungMassVelocity.resize(numSteps, 0.0);
  unsprungMassDisplacement.resize(numSteps, 0.0);
  sprungMassAcc.resize(numSteps, 0.0);
  sprungMassAccG.resize(numSteps, 0.0);
  sprungMassVelocity.resize(numSteps, 0.0);
  sprungMassDisplacement.resize(numSteps, 0.0);
  roadLineVelocity.resize(numSteps, 0.0);
  roadLineAcceleration.resize(numSteps, 0.0);
  relativeDisplacementVector.resize(numSteps, 0.0);
  relativeTireDisplacementVector.resize(numSteps, 0.0);
  sprungMassPosition.resize(numSteps, car.getStaticHeight());
  unsprungMassPosition.resize(numSteps, car.getTireStaticHeight());
  sprungMassNetForce.resize(numSteps, 0.0);
  unsprungMassNetForce.resize(numSteps, 0);
  stopperForce.resize(numSteps, 0.0);
  tireDefLimit.resize(numSteps, 0.0);
  springBumpLimit.resize(numSteps, 0.0);
  springReboundLimit.resize(numSteps, 0.0);
  bumpStopStiffness.resize(numSteps, 0.0);
  reboundStopStiffness.resize(numSteps, 0.0);
  displacementAttenuation.resize(numSteps, 0.0);
  accelerationAttenuation.resize(numSteps, 0.0);

  sprungMassSnap.resize(numSteps, 0.0);
  sprungMassJerk.resize(numSteps, 0.0);

  bumpStopForce.resize(numSteps, 0.0);
  reboundStopForce.resize(numSteps, 0.0);
  springForce.resize(numSteps, -gravity * car.getSprungMass());
  damperForce.resize(numSteps, 0.0);
  tireElasticForce.resize(numSteps, -gravity * (car.getSprungMass() + car.getUnsprungMass()));
  tireDamperForce.resize(numSteps, 0.0);

  std::cout << "\nResponse Vectors Initialized\n";
}

double Simulation::AirBorneTime()
{
  double currentTime = 0;
  double totalTime = 0;
  for (int i = 0; i < tireElasticForce.size(); i++)
  {
    if (tireElasticForce[i] > -0.001)
    {
      totalTime = totalTime + timeStepSize_;
      // std::cout << "\nTotal Airborne Time: " << totalTime<<" | Tire Force: "<<tireElasticForce[i];
    }
  }
  return totalTime;
}

std::vector<double> Simulation::CalculateSignalRatio(std::vector<double> &input, std::vector<double> &output)
{
  std::cout << "\nCalculating RMS Gains";
  int signalSize = input.size();
  std::vector<double> vectorRatio(signalSize, 0);
  for (int i = 0; i < signalSize; i++)
  {
    vectorRatio[i] = (output[i] / input[i]);
    // std::cout<<"Ratio: "<<vectorRatio[i]<<", inputSignal: "<<inputSignal[i]<<", outputSignal: "<<outputSignal[i]<<std::endl;
  }
  // std::cout << "\nCalculated RMS Gains";

  return vectorRatio;
}

std::vector<double> Simulation::CalculateRMS(const std::vector<double> &psd)
{
  std::cout << "\nCalculating RMS outputs. . .";
  std::vector<double> rms(psd.size());
  for (int i = 0; i < psd.size(); i++)
  {
    rms[i] = std::sqrt(psd[i]);
  }
  return rms;
}

double Simulation::CalcTimeAccRMS(const std::vector<double> &data)
{
  double sum = 0.0;
  for (size_t i = 0; i < data.size(); i++)
  {
    sum += pow(data[i], 2);
  }
  double mean = sum / data.size();
  double rms = sqrt(mean);
  return rms;
}

void Simulation::StaticEquilibrium(Car &car)
{
  std::cout << "\nCalculating static equilibrium configuration. . .";
  double tireStiffness = car.getTireSpring()->getStiffness(0);
  double suspStiffness = car.getSpring()->getStiffness(0);
  // double dampingRatio = car.getDamper()->getDampingRatio(0);

  suspSpringDeflection_ = (car.getSprungMass() * 9810 / suspStiffness);
  tireSpringDeflection_ = (car.getSprungMass() + car.getUnsprungMass()) * 9810 / tireStiffness;

  sprungMassDeflection_ = suspSpringDeflection_ + tireSpringDeflection_;

  sprungMassInitialPosition = car.getSpring()->getFreeLength() + car.getTireSpring()->getFreeLength() - sprungMassDeflection_;
  unsprungMassInitialPosition = car.getTireSpring()->getFreeLength() - tireSpringDeflection_;
  availableBumpTravel_ = car.getSpring()->getFreeLength() - suspSpringDeflection_;
  availableReboundTravel_ = suspSpringDeflection_;

  // Setting Contact Spring parameters for tire and stoppers
  // springPreload = car.getRideHeight()

  car.getBumpStopSpring()->setTriggerDistance(-availableBumpTravel_);
  car.getReboundStopSpring()->setTriggerDistance(-availableReboundTravel_);
  car.getTireSpring()->setTriggerDistance(tireSpringDeflection_);
  // std::cout << "\n(Inside StaticEquilibrium Method) Available Bump Travel: " << car.getBumpStopSpring()->getTriggerDistance() << ", set by suspSpringDeflection: " << suspSpringDeflection_ << ", and getfreelenth: " << car.getSpring()->getFreeLength();
  // std::cout << "\n(Inside StaticEquilibrium Method) Available Rebound Travel: " << car.getReboundStopSpring()->getTriggerDistance();
  car.setStaticHeight(sprungMassInitialPosition);
  car.setTireStaticHeight(unsprungMassInitialPosition);
 // car.getSpring()->setPreload(suspSpringDeflection_ * car.getSpring()->getStiffness(0));
  //car.getTireSpring()->setPreload(tireSpringDeflection_ * car.getTireSpring()->getStiffness(0));
  int selection;
  printStaticEquilibriumResults();
}

void Simulation::printStaticEquilibriumResults()
{
  std::cout << "\n---------------------------------------- STATIC EQUILIBRIUM -----------------------------------------";
  std::cout << "\n\nCalculated static equilibrium configuration: ";
  std::cout << "\nThe unsprung mass deflection was " << tireSpringDeflection_ << " " << getDisplacementUnit() << ", and the remaining tire travel is: " << unsprungMassInitialPosition << " " << getDisplacementUnit() << ".";
  std::cout << "\nThe sprung mass deflection was " << sprungMassDeflection_ << " " << getDisplacementUnit() << ", and the current Ride Height is: " << sprungMassInitialPosition << " " << getDisplacementUnit() << "."
            << ".\n\nThe available bump travel is : " << availableBumpTravel_ << " " << getDisplacementUnit() << ".";
  std::cout << "\nThe available rebound travel is: " << availableReboundTravel_ << " " << getDisplacementUnit() << ".";
  // std::cout << "\nSelect:\n[1] Proceed with the displayed configuration.\n[2] Set a Spring Preload change ride height.\nSelection: ";
  // std::cin >> selection;
  // std::cout << "\n---------------------------------------------------------------------------------------------------";
}

void Simulation::printMetrics()
{
  std::cout << "\n\n----------------------------------------- CALCULATED METRICS --------------------------------------------";
  std::cout << "\n\nAbruptness: " << abruptness;
  std::cout << "\nRMS Accelerometer Chassis: " << sprungMassAccRMS;
  std::cout << "\nAir Borne Time [s]: " << airBorneTime;
  std::cout << "\n\n---------------------------------------------------------------------------------------------------------\n\n";
}
std::vector<double> Simulation::ComputePSD(const std::vector<double> &signal, int N, double sample_rate)
{
  std::cout << "\nCalculating PSD. . .";
  std::vector<double> PSD;

  // Allocate memory for the FFT input and output arrays
  fftw_complex *in, *out;
  in = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * N);
  out = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * N);

  /*// Copy the time signal into the input array
  for (int i = 0; i < N; i++) {
      in[i][0] = signal[i];
      in[i][1] = 0.0;
  }*/

  // Apply windowing to the input time signal
  for (int i = 0; i < N; i++)
  {
    in[i][0] = signal[i] * (0.5 - 0.5 * cos(2 * M_PI * i / (N - 1)));
    in[i][1] = 0.0;
  }

  // Create the FFT plan
  fftw_plan plan = fftw_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

  // Execute the FFT
  fftw_execute(plan);

  // Calculate the power spectral density
  PSD.resize(N / 2 + 1);
  frequencies.resize(N / 2 + 1);
  for (int i = 0; i < N / 2 + 1; i++)
  {
    PSD[i] = std::sqrt(out[i][0] * out[i][0] + out[i][1] * out[i][1]) / N;
    frequencies[i] = i * sample_rate / N;
  }

  // Free the memory
  fftw_free(in);
  fftw_free(out);
  fftw_destroy_plan(plan);

  return PSD;
}

void Simulation::Simulate(Car &car, Road &road)
{
  StaticEquilibrium(car);

  std::cout << "\nStarting Simulation: Stage 1";

  // int numSteps = simulationTime_ / timeStepSize_;

  roadLine = std::move(road.CalcRoad(simulationTime_, timeStepSize_));
  // roadLine = road.CalcRoad(simulationTime_, timeStepSize_);

  std::cout << "\n\nRoad Calculated and moved to roadLine";

  int numSteps = roadLine.size();
  int simNumSteps = simulationTime_ / timeStepSize_;

  std::cout << "\n\nBefore: time Step size is: " << timeStepSize_;
  if (simNumSteps > numSteps)
  {
    timeStepSize_ = simulationTime_ / (double)numSteps;
  }
  std::cout << "\nAfter: time Step size is: " << timeStepSize_;

  std::cout << "\n\nCalculated number of steps: " << numSteps;
  // Getting and calculating additional vehicle parameters.
  double tireStiffness = car.getSpring()->getStiffness(0);
  double suspStiffness = car.getSpring()->getStiffness(0);
  double dampingRatio = car.getDamper()->getDampingRatio(0);
  double f_ride = car.CalcRideFreq();
  double dampingCoefficient = car.CalcSuspDamp(0);
  std::cout << "\nSprung Mass Natural Frequency Calculated: " << car.CalcSprungNatFreq();
  std::cout << "\n\nRide Frequency Calculated. Frequency: " << f_ride;
  std::cout << "\nDamping Coefficient Calculated. Value: " << car.CalcSuspDamp(0) << std::endl;

  // Initializing vectors
  InitializeVectors(numSteps, car);

  // std::cout << "\nInitial Ride Height: " << sprungMassPosition[0] << ". Initial Tire Height: " << unsprungMassPosition[0];
  double relativeDisplacement = 0.0;
  double springDisplacement = 0.0;
  double relativePosition = 0.0;
  double relativeVelocity = 0.0;
  double staticHeight = 100;
  double relativeTireDisplacement;
  double relativeTirePosition;

  unsprungMassWeight = (car.getUnsprungMass() / massUnitScaling_) * gravity;
  sprungMassWeight = (car.getSprungMass() / massUnitScaling_) * gravity;

  time.resize(numSteps, 0.0);

  std::cout << "\nTime Vector Initialized\n";

  std::cout << "\nStarting Simulation: Level 2 - Calculations";

  // Calculating Outputs: Forces, Displacements, Velocities and Accelerations
  for (int i = 1; i < numSteps - 1; i++)
  {
    // Calculating relative displacements and velocities.
    springDisplacement = sprungMassDisplacement[i] - unsprungMassDisplacement[i];
    relativeDisplacement = springDisplacement - suspSpringDeflection_;
    relativeTireDisplacement = unsprungMassDisplacement[i] - roadLine[i];
    relativeVelocity = sprungMassVelocity[i] - unsprungMassVelocity[i];
    relativeDisplacementVector[i] = springDisplacement;
    relativeTireDisplacementVector[i] = relativeTireDisplacement;

    // Generating contact spring limit vectors for plotting and data exporting.
    tireDefLimit[i] = car.getTireSpring()->getTriggerDistance();
    springBumpLimit[i] = car.getBumpStopSpring()->getTriggerDistance();
    springReboundLimit[i] = -car.getReboundStopSpring()->getTriggerDistance();
    bumpStopStiffness[i] = car.getBumpStopSpring()->getStiffness(springDisplacement);
    reboundStopStiffness[i] = car.getReboundStopSpring()->getStiffness(-springDisplacement);

    // Zeroing to static position at first and last positions.
    // tireElasticForce[0] = -gravity*(car.getSprungMass() +car.getUnsprungMass());
    // tireElasticForce[numSteps-1]=tireElasticForce[0];
    // springForce[0]= -gravity*car.getSprungMass();
    // springForce[numSteps-1] = springForce[0];

    // Calculating all forces acting on system components in each iteration.
    bumpStopForce[i] = car.getBumpStopSpring()->getStiffness(springDisplacement) * (springDisplacement - car.getBumpStopSpring()->getTriggerDistance());
    reboundStopForce[i] = car.getReboundStopSpring()->getStiffness(-springDisplacement) * (springDisplacement + car.getReboundStopSpring()->getTriggerDistance());
    stopperForce[i] = bumpStopForce[i] + reboundStopForce[i];
    springForce[i] = car.getSpring()->getStiffness(relativeDisplacement) * (relativeDisplacement);
    damperForce[i] = car.CalcSuspDamp(relativeVelocity) * (relativeVelocity);
    tireDamperForce[i] = car.getTireDamper()->getDampingCoefficient() * (unsprungMassVelocity[i] - roadLineVelocity[i]);
    tireElasticForce[i] = car.getTireSpring()->getStiffness(relativeTireDisplacement) * (relativeTireDisplacement - tireSpringDeflection_);

    // Calculating equations of motion using the forces in each iteration.
    sprungMassAcc[i] = ((-springForce[i] - damperForce[i] - stopperForce[i] - sprungMassWeight) / (car.getSprungMass()));
    unsprungMassAcc[i] = (-tireDamperForce[i] - tireElasticForce[i] + springForce[i] + damperForce[i] + stopperForce[i] - unsprungMassWeight) / car.getUnsprungMass();
    sprungMassVelocity[i + 1] = sprungMassAcc[i] * timeStepSize_ + sprungMassVelocity[i];
    unsprungMassVelocity[i + 1] = unsprungMassAcc[i] * timeStepSize_ + unsprungMassVelocity[i];
    sprungMassDisplacement[i + 1] = sprungMassVelocity[i] * timeStepSize_ + sprungMassDisplacement[i];
    unsprungMassDisplacement[i + 1] = unsprungMassVelocity[i] * timeStepSize_ + unsprungMassDisplacement[i];

    roadLineVelocity[i] = (roadLine[i] - roadLine[i - 1]) / timeStepSize_;
    roadLineAcceleration[i] = (roadLineVelocity[i] - roadLineVelocity[i - 1]) / timeStepSize_; // Velocity of ground displacement

    sprungMassAccG[i] = sprungMassAcc[i] / (9.81 * gUnitScaling_);
    unsprungMassAccG[i] = unsprungMassAcc[i] / (9.81 * gUnitScaling_);

    // Calculating actual position considering static constants to be visualized on the position plots.
    sprungMassPosition[i] = sprungMassDisplacement[i] + car.getStaticHeight();
    unsprungMassPosition[i] = unsprungMassDisplacement[i] + car.getTireStaticHeight();

    sprungMassNetForce[i] = sprungMassAcc[i] * car.getSprungMass();
    unsprungMassNetForce[i] = unsprungMassAcc[i] * car.getUnsprungMass();

    // Calculating Sprung Mass Jerk and Snap (3rd and 4th order displacement derivatives)
    // std::cout << "\nCalculating Jerk Metric. . . | i: "<<i;
    sprungMassJerk[i] = (sprungMassAccG[i] - sprungMassAccG[i - 1]) / timeStepSize_;

    // std::cout << "\nCalculating Snap Metric. . .| i: "<< i;
    sprungMassSnap[i] = sprungMassJerk[i] * sprungMassDisplacement[i];

    // Generating the time vector
    time[i + 1] = (i + 1) * timeStepSize_;
  }

  // Calculating the PSD

  std::cout << "\nCalculating Frequency Domain Metrics. . .";

  // std::cout << "\nCalculating PSD outputs. . .";
  double sampleRate = 1 / timeStepSize_;
  accelerationPSD = ComputePSD(sprungMassAccG, numSteps, sampleRate);
  roadAccPSD = ComputePSD(roadLineAcceleration, numSteps, sampleRate);
  sprungDisplacementPSD = ComputePSD(sprungMassDisplacement, numSteps, sampleRate);
  roadDisplacementPSD = ComputePSD(roadLine, numSteps, sampleRate);

  // Initializing Frequency Domain Metrics Vectors
  sprungMassDisplacementRMS.resize(frequencies.size(), 0.0);
  sprungMassAccelerationRMS.resize(frequencies.size(), 0.0);
  roadLineDisplacementRMS.resize(frequencies.size(), 0.0);
  roadLineAccelerationRMS.resize(frequencies.size(), 0.0);
  sprungDispDelay.resize(frequencies.size(), 0.0);
  accelerationDelay.resize(frequencies.size(), 0.0);

  // Calculating each RMS curve to be used as arguments of CalculateSignalRatio
  // std::cout << "\nCalculating RMS outputs. . .";
  sprungMassDisplacementRMS = CalculateRMS(sprungDisplacementPSD);
  roadLineDisplacementRMS = CalculateRMS(roadDisplacementPSD);
  sprungMassAccelerationRMS = CalculateRMS(accelerationPSD);
  roadLineAccelerationRMS = CalculateRMS(roadAccPSD);
  // int RMSSize = roadLineDisplacementRMS.size();

  // Calculating the ratio between the RMS of the response and roadInput.
  // std::cout << "\nCalculating RMS Gains. . .";
  displacementRMSRatio.resize(frequencies.size(), 0.0);
  accelerationRMSRatio.resize(frequencies.size(), 0.0);
  displacementRMSRatio = CalculateSignalRatio(roadLineDisplacementRMS, sprungMassDisplacementRMS);
  accelerationRMSRatio = CalculateSignalRatio(roadLineAccelerationRMS, sprungMassAccelerationRMS);

  // Calculating Delay Metrics
  accelerationDelay = ComputeDelay(accelerationPSD, frequencies);
  sprungDispDelay = ComputeDelay(sprungDisplacementPSD, frequencies);

  // Exporting resampled signal to a new .csv file
  ExportResults(numSteps, car);
  abruptness = CalculateAbruptness(sprungMassAccG, simulationTime_);
  sprungMassAccRMS = CalcTimeAccRMS(sprungMassAcc);
  airBorneTime = AirBorneTime();

  std::cout << "\n\nSimulation completed!";
}

double Simulation::CalculateAbruptness(const std::vector<double> &sprungMassAcceleration, double duration)
{
  double abruptness = 0.0;
  for (int i = 1; i < sprungMassAcceleration.size(); i++)
  {
    double deltaAcceleration = sprungMassAcceleration[i] - sprungMassAcceleration[i - 1];
    abruptness += abs(deltaAcceleration);
  }
  abruptness /= duration;
  return abruptness;
}

std::vector<double> Simulation::ComputeDelay(const std::vector<double> &PSD, const std::vector<double> &frequencies)
{
  //std::cout << "\nCalculating Delays. . .";
  //std::cout << "\n PSD Size: " << PSD.size() << ", frequencies size: " << frequencies.size();
  int N = PSD.size();
  std::vector<double> delayVector(N, 0);
  for (int i = 1; i < N; i++)
  {
    delayVector[i] = PSD[i] / frequencies[i];
  }

  return delayVector;
}

void Simulation::ClearCSV(const std::string &filename)
{
  std::ofstream ofs(filename, std::ios::out | std::ios::trunc);
  ofs.close();
}

int Simulation::Graph()
{
  int graphCount = 0;
  int graphType = 2;
  int graphOpt;
  int rep;
  int togglePlot;
  int showSprungRel;
  int showUnsprungRel;
  int showRoadProfile = 1;
  int showUnsprungAcc;
  int showSprungAcc = 1;
  int showUnsprungDisp;
  int showUnsprungPos;
  int attenuation = 0;
  int PSDPlot = 1;
  int metrics = 1;
  double xFreqLim = 25;
  char keepPlot = 'y';
  char deleteCSV = 'n';
  int config;
  int wdt = 2;

  std::cout << "\nSelect plotting config:";
  std::cout << "\n[1] Profile 1: Unsprung Mass Acceleration [OFF]; Unsprung Mass Displacement [OFF]; Spring Deformation [OFF]; Tire Deformation [OFF];";
  std::cout << "\n[2] Profile 2: Unsprung Mass Acceleration [ON]; Unsprung Mass Displacement [ON]; Spring Deformation [OFF]; Tire Deformation [OFF];";
  std::cout << "\n[3] Custom Profile. Select configuration. ";
  std::cout << "\nSelection: ";
  std::cin >> config;

  switch (config)
  {

  case 1:
    showUnsprungDisp = 0;
    showUnsprungAcc = 0;
    showSprungRel = 0;
    showUnsprungRel = 0;
    break;

  case 2:
    showUnsprungDisp = 1;
    showUnsprungAcc = 1;
    showSprungRel = 0;
    showUnsprungRel = 0;
    break;

  case 3:
    std::cout << "\nDisplay Unsprung Mass Acceleration?  [0] OFF     [1] ON\nSelection: ";
    std::cin >> showUnsprungAcc;
    std::cout << "\nDisplay the Unsprung Mass Displacement?  [0] OFF     [1] ON\nSelection: ";
    std::cin >> showUnsprungDisp;
    std::cout << "\nDisplay the Suspension Spring Deformation?  [0] OFF     [1] ON\nSelection: ";
    std::cin >> showSprungRel;
    std::cout << "\nDisplay Tire Deformation?  [0] OFF     [1] ON\nSelection: ";
    std::cin >> showUnsprungRel;
    break;
  }

  if (keepPlot == 'y')
  {
    std::cout << "\nDo you want to use the last saved results for comparison? (y/n). \n\nIf this is a different event, it is recommended to select 'n'.\nSelection: ";
    std::cin >> deleteCSV;
    if (deleteCSV == 'n')
    {
      ClearCSV("MetricsOutput.csv");
    }
  }

  // std::cout << "";

  while (true)
  {
    graphCount = 1;
    if (keepPlot == 'y')
      CopyPlots();

    if (metrics == 1)
    {
      std::cout << "\nBuilding Metric Plots. . .";
      using namespace matplot;
      int wdt = 2; // Line Width
      auto h = figure(true);
      h->name("Quarter Car Metrics"); // Figure Name
      h->size(1900, 950);             // Figure Size

      auto ax0 = subplot(2, 2, 0);
      title("RMS Gain: Acceleration");
      auto p0 = plot(frequencies, accelerationRMSRatio);
      p0->line_width(wdt);
      p0->display_name("Current Metrics");
      hold(on);
      auto p01 = plot(frequencies, accelerationRMSRatioCompare);
      p01->line_width(wdt);
      p01->display_name("Previous Metrics");
      auto lgd0 = legend(on);
      lgd0->font_name("Arial");
      xlabel("Frequency [Hz]");
      ylabel("RMS Gain - Sprung Mass Acc");
      xlim({0, xFreqLim});

      auto ax1 = subplot(2, 2, 1);
      // title("Vertical Position");
      auto p1 = plot(frequencies, accelerationDelay);
      //->display_name("Unsprung");
      p1->line_width(wdt);
      p1->display_name("Current Metrics (PSD/freq)");
      hold(on);
      auto p10 = plot(frequencies, accelerationDelayCompare);
      p10->line_width(wdt);
      p10->display_name("Previous Metrics (PSD/freq)");
      auto lgd10 = legend(on);
      lgd10->font_name("Arial");
      xlim({0, xFreqLim});
      xlabel("Frequency [Hz]");
      ylabel("Sprung Mass Delay [g²/Hz²]");

      auto ax2 = subplot(2, 2, 3);
      // title("Velocity");
      auto p33 = plot(time, sprungMassJerk);
      p33->line_width(wdt);
      p33->display_name("Current Metrics");
      hold(on);
      auto p30 = plot(time, sprungMassJerkCompare);
      p30->line_width(wdt);
      p30->display_name("Previous Metrics");
      auto lgd14 = legend(on);
      lgd14->font_name("Arial");
      xlabel("Time [s]");
      ylabel("Sprung Mass Jerk [g/s]");

      auto forces = subplot(2, 2, 2);
      title("RMS Gain: Displacement");
      auto p7 = plot(frequencies, displacementRMSRatio);
      p7->line_width(wdt);
      p7->display_name("Current Metrics ");
      hold(on);
      auto p70 = plot(frequencies, displacementRMSRatioCompare);
      p70->line_width(wdt);
      p70->display_name("Previous Metrics");
      auto lgd4 = legend(on);
      lgd4->font_name("Arial");
      xlabel("Frequency [hz]");
      ylabel("RMS Gain - Sprung Mass Disp");
      xlim({0, xFreqLim});
      show();
    }

    if (graphType == 2)
    {
      {
        std::cout << "\nBuilding General Output Plots. . .";
        using namespace matplot;
        int wdt = 2; // Line Width
        auto h = figure(true);
        h->name("Quarter Car"); // Figure Name
        h->size(1900, 950);     // Figure Size

        auto ax0 = subplot(3, 2, 0);
        title("Input: ");

        auto p0 = plot(time, roadLine);
        p0->line_width(wdt);
        p0->display_name("2D Road");
        auto lgd0 = legend(on);
        lgd0->font_name("Arial");
        xlabel("Time [s]");
        ylabel("Displacement " + displacementUnit_);
        hold(on);

        auto ax1 = subplot(3, 2, 1);
        // title("Vertical Position");
        auto p2 = plot(time, sprungMassDisplacement);
        p2->line_width(wdt);
        p2->display_name("Sprung: Current");
        hold(on);
        auto p21 = plot(time, sprungMassDisplacementCompare);
        p21->line_width(wdt);
        p21->display_name("Sprung: Previous");
        hold(on);
        // std::cout << "showUnsprungDisp: " << showUnsprungDisp;
        if (showUnsprungDisp == 1)
        {
          // std::cout << "Showing unsprung mass disp.";
          auto p41 = plot(time, unsprungMassDisplacement);
          //->display_name("Unsprung");
          p41->line_width(wdt);
          p41->display_name("Unsprung");
          hold(on);
        }

        if (showSprungRel == 1)
        {
          auto p20 = plot(time, relativeDisplacementVector);
          p20->line_width(wdt);
          p20->display_name("Difference");
        }

        auto lgd = legend(on);
        lgd->font_name("Arial");
        xlabel("Time [s]");
        ylabel("Position " + displacementUnit_);
        auto ax2 = subplot(3, 2, 3);
        // title("Velocity");
        auto p3 = plot(time, unsprungMassVelocity);
        p3->line_width(wdt);
        p3->display_name("Unsprung");
        hold(on);
        auto p4 = plot(time, sprungMassVelocity);
        p4->line_width(wdt);
        p4->display_name("Sprung");
        auto lgd2 = legend(on);
        lgd2->font_name("Arial");
        xlabel("Time [s]");
        ylabel("Velocity " + velocityUnit_);

        auto forces = subplot(3, 2, {2, 4});
        title("Forces");

        auto p7 = plot(time, springForce);
        p7->line_width(wdt);
        p7->display_name("Spring Force");
        hold(on);

        auto p8 = plot(time, damperForce);
        p8->line_width(wdt);
        p8->display_name("Damper Force");
        hold(on);

        auto p9 = plot(time, stopperForce);
        p9->line_width(wdt);
        p9->display_name("Stopper Force");
        hold(on);

        auto p10 = plot(time, tireElasticForce);
        p10->line_width(wdt);
        p10->display_name("Tire Force");
        auto lgd4 = legend(on);
        lgd4->font_name("Arial");
        xlabel("Time [s]");
        ylabel("Force " + forceUnit_);

        auto ax3 = subplot(3, 2, 5);
        // ax3 -> size(500,500);
        // title("Acceleration");
        if (showUnsprungAcc == 1)
        {
          // ax3 -> size(500,500);
          // title("Acceleration");
          auto p51 = plot(time, unsprungMassAccG);
          p51->line_width(wdt);
          p51->display_name("Unsprung");
          hold(on);
        }

        if (showSprungAcc == 1)
        {
          auto p6 = plot(time, sprungMassAccG);
          p6->line_width(wdt);
          p6->display_name("Sprung");
          auto lgd3 = legend(on);
          lgd3->font_name("Arial");
          xlabel("Time [s]");
          ylabel("Acceleration " + accelerationUnit_);
          hold(on);
          auto p242 = plot(time, sprungMassAccGCompare);
          p242->line_width(wdt);
          p242->display_name("Sprung: Previous");
          show();
        }
      }
    }

    else if (graphType == 1)
    {

      std::cout << "\nBuilding Plots. . .";

      using namespace matplot;
      int wdt = 2; // Line Width
      auto h = figure(true);
      h->name("Quarter Car"); // Figure Name
      h->size(1900, 950);     // Figure Size

      if (showRoadProfile != 1)
      {
        auto ax0 = subplot(3, 2, 0);
        title("Input: ");
        auto p0 = plot(time, roadLine);
        p0->line_width(wdt);
        p0->display_name("2D Road");
        auto lgd0 = legend(on);
        lgd0->font_name("Arial");
      }

      auto ax1 = subplot(3, 2, {2, 4});
      // title("Vertical Position");
      auto p2 = plot(time, sprungMassPosition);
      p2->line_width(wdt);
      p2->display_name("Sprung: Current");
      xlabel("Time [s]");
      ylabel("Displacement " + displacementUnit_);
      hold(on);
      auto p27 = plot(time, sprungMassPositionCompare);
      p27->line_width(wdt);
      p27->display_name("Sprung: Previous");
      hold(on);
      if (showUnsprungDisp == 1)
      {
        auto p188 = plot(time, unsprungMassPosition);
        //->display_name("Unsprung");
        p188->line_width(wdt);
        p188->display_name("Unsprung");
      }

      char showDiff = 'y';

      if (showUnsprungRel == 1)
      {
        auto p20 = plot(time, relativeTireDisplacementVector);
        p20->line_width(wdt);
        p20->display_name("Tire Deformation");
        auto p31 = plot(time, tireDefLimit, ":");
        p31->line_width(1);
        p31->display_name("Tire Def. Limit: Bump");
      }

      if (showSprungRel == 1)
      {
        auto p21 = plot(time, relativeDisplacementVector);
        p21->line_width(wdt);
        p21->display_name("Spring Deformation");
        auto p31 = plot(time, springBumpLimit, ":");
        p31->line_width(1);
        p31->display_name("Spring Def. Limit: Bump");
        auto p41 = plot(time, springReboundLimit, ":");
        p41->line_width(1);
        p41->display_name("Spring Def. Limit: Rebound");
      }

      if (showRoadProfile == 1)
      {
        auto p22 = plot(time, roadLine);
        p22->line_width(wdt - 1);
        p22->display_name("Road Profile");
        auto lgd = legend(on);
        lgd->font_name("Arial");
        xlabel("Time [s]");
        ylabel("Position " + displacementUnit_);

        auto ax0 = subplot(3, 2, 0);

        if (PSDPlot == 0)
        {

          if (attenuation == 1)
          {
            title("Attenuation ");
            auto p0 = plot(time, displacementRMSRatio);
            p0->line_width(wdt);
            p0->display_name("Displacement Attenuation");
            auto lgd0 = legend(on);
            lgd0->font_name("Arial");
            xlabel("Time [s]");
            ylabel("Displacement RMS Ratio [%]");
            hold(on);
          }
          else
          {
            title("Attenuation ");
            auto p01 = plot(time, accelerationRMSRatio);
            p01->line_width(wdt);
            p01->display_name("Acceleration Attenuation");
            auto lgd01 = legend(on);
            lgd01->font_name("Arial");
            xlabel("Time [s]");
            ylabel("Acceleration RMS Ratio [%]");
            hold(on);
          }
        }
        else
        {
          title("PSD");
          auto p01 = plot(frequencies, accelerationPSD);
          p01->line_width(wdt);
          p01->display_name("Sprung Mass Acceleration PSD: Current");
          hold(on);
          auto p012 = plot(frequencies, accelerationPSDCompare);
          p012->line_width(wdt);
          p012->display_name("Sprung Mass Acceleration PSD: Previous");
          auto lgd01 = legend(on);
          lgd01->font_name("Arial");
          xlabel("Frequency [Hz]");
          ylabel("g²/Hz");
          xlim({0, 25});
        }
      }

      auto ax2 = subplot(3, 2, 3);
      // title("Velocity");
      auto p3 = plot(time, unsprungMassVelocity);
      p3->line_width(wdt);
      p3->display_name("Unsprung");
      hold(on);
      auto p4 = plot(time, sprungMassVelocity);
      p4->line_width(wdt);
      p4->display_name("Sprung");
      auto lgd2 = legend(on);
      lgd2->font_name("Arial");
      xlabel("Time [s]");
      ylabel("Velocity " + velocityUnit_);

      auto forces = subplot(3, 2, 1);
      title("Forces");

      auto p7 = plot(time, springForce);
      p7->line_width(wdt);
      p7->display_name("Spring Force");
      hold(on);

      auto p8 = plot(time, damperForce);
      p8->line_width(wdt);
      p8->display_name("Damper Force");
      hold(on);

      auto p9 = plot(time, stopperForce);
      p9->line_width(wdt);
      p9->display_name("Bumpstop Force");
      hold(on);

      auto p10 = plot(time, tireElasticForce);
      p10->line_width(wdt);
      p10->display_name("Tire Force");
      auto lgd4 = legend(on);
      lgd4->font_name("Arial");
      xlabel("Time [s]");
      ylabel("Force " + forceUnit_);

      auto ax3 = subplot(3, 2, 5);
      // ax3 -> size(500,500);
      // title("Acceleration");
      if (showUnsprungAcc == 1)
      {
        // ax3 -> size(500,500);
        // title("Acceleration");
        auto p51 = plot(time, unsprungMassAccG);
        p51->line_width(wdt);
        p51->display_name("Unsprung");
        hold(on);
      }

      if (showSprungAcc == 1)
      {
        auto p6 = plot(time, sprungMassAccG);
        p6->line_width(wdt);
        p6->display_name("Sprung");
        auto lgd3 = legend(on);
        lgd3->font_name("Arial");
        hold(on);
        auto p242 = plot(time, sprungMassAccGCompare);
        p242->line_width(wdt);
        p242->display_name("Sprung: Previous");
        xlabel("Time [s]");
        ylabel("Acceleration " + accelerationUnit_);
        show();
      }
    }

    //int rep;
    while (true)
    {
      std::cout << "\nNext Action:";
      std::cout << "\n[1] - Setup and run new simulation with current unit system.";
      std::cout << "\n[2] - Graph Display Settings.";
      std::cout << "\n[3] - Setup and run new simulation with different unit system.";
      std::cout << "\n[4] - Rebuild Plots";
      std::cout << "\n[5] - Close program";
      std::cout << "\nSelection: ";

      if (std::cin >> rep && (rep >= 1 && rep <= 5))
      {
        break;
      }
      else
      {
        std::cout << "Invalid input. Please enter a valid option." << std::endl;
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      }
    }

    //graphOpt = rep;
    // if (rep != 2 && rep != 4)
    //{
    if (rep == 1 || rep == 3)
    {
      std::cout << "\nDo you wish to keep the current calculated outputs in the plot, to be compared with the next graph? (y or n)\nSelection: ";
      std::cin >> keepPlot;
      if (keepPlot == 'y')
      {
        ExportMetrics();
      }
      break;
    }
    // }
    else if (rep == 5)
    {
      //std::exit(0);
      //graphOpt = rep;
      break;
    }
    else if (rep == 2)
    {
      while (true)
      {
        std::cout << "\n-----------------------------PLOT SETTINGS------------------------------";
        std::cout << "\n\n[1] Toggle on/off Suspension Spring Travel visualization";
        std::cout << "\n[2] Toggle on/off Tire Spring Travel visualization";
        std::cout << "\n[3] Toggle on/off Road Profile Displacement within Position plot.";
        std::cout << "\n[4] Toggle between Displacement RMS Ratio or Acceleration RMS Ratio";
        std::cout << " \n[5] Toggle Unsprung Mass Acceleration Visualization";
        std::cout << "\n[6] Toggle Sprung Mass Acceleration Visualization";
        std::cout << "\n[7] Toggle PSDPlot Visualization";
        std::cout << "\n[8] Toggle Unsprung Mass Displacement/Position Visualization";
        std::cout << "\n[9] Set Limit of the X Axis for frequency domain metrics";
        std::cout << "\n[10] Toggle Plot Layout (Displacement-Centered / Forces-Centered)";

        std::cout << "\nSelection: ";

        if (std::cin >> togglePlot && (togglePlot >= 1 && togglePlot <= 10))
        {
          break;
        }
        else
        {
          std::cout << "Invalid input. Please enter a valid option." << std::endl;
          std::cin.clear();
          std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
      }
      std::cout << " \n------------------------------------------------------------------------";
      switch (togglePlot)
      {
      case 1:
        showSprungRel = showSprungRel + 1;
        if (showSprungRel != 1)
          showSprungRel = 0;
        break;

      case 2:
        showUnsprungRel = showUnsprungRel + 1;
        if (showUnsprungRel != 1)
          showUnsprungRel = 0;
        break;

      case 3:
        showRoadProfile = showRoadProfile + 1;
        if (showRoadProfile != 1)
          showRoadProfile = 0;
        break;

      case 4:
        attenuation = attenuation + 1;
        if (attenuation != 1)
          attenuation = 0;
        break;

      case 5:
        showUnsprungAcc = showUnsprungAcc + 1;
        if (showUnsprungAcc != 1)
          showUnsprungAcc = 0;
        break;

      case 6:
        showSprungAcc = showSprungAcc + 1;
        if (showSprungAcc != 1)
          showSprungAcc = 0;
        break;

      case 7:
        PSDPlot = PSDPlot + 1;
        if (PSDPlot != 1)
          PSDPlot = 0;
        break;

      case 8:
        showUnsprungDisp = showUnsprungDisp + 1;
        if (showUnsprungDisp != 1)
          showUnsprungDisp = 0;
        showUnsprungPos = 0;
        break;

      case 9:
        std::cout << "\nSelect the end frequency [Hz] of the X Axis: ";
        std::cin >> xFreqLim;
        break;

      case 10:
        graphType = graphType + 1;
        if (graphType != 2)
          graphType = 1;
        break;
      }
    }
    else if (rep == 4)
      rep = 2;
    //graphOpt = rep;
  }

  return rep;
}

void Simulation::CopyPlots()
{
  std::cout << "\nCopying plots";
  int N = accelerationRMSRatio.size();
  int i = 0;

  displacementRMSRatioCompare.resize(0, 0);
  accelerationRMSRatioCompare.resize(0, 0);
  accelerationDelayCompare.resize(0, 0);
  displacementDelayCompare.resize(0, 0);
  sprungDispDelayCompare.resize(0, 0);
  sprungDisplacementPSDCompare.resize(0, 0);
  accelerationPSDCompare.resize(0, 0);
  sprungMassJerkCompare.resize(0, 0);
  sprungMassSnapCompare.resize(0, 0);
  sprungMassDisplacementCompare.resize(0, 0);
  sprungMassPositionCompare.resize(0, 0);

  std::string line;
  std::string filename = "MetricsOutput.csv";
  std::string field;
  std::stringstream ss;

  std::fstream myFile;
  myFile.open(filename, std::ios::in); // read
  if (myFile.is_open())
  {
    std::cout << "\nParsing data from file . . .\n";
    std::string line;

    // read and discard the first line
    std::getline(myFile, line);

    while (std::getline(myFile, line))
    {
      ss.str(line); // set the stringstream to the current line
      i = i + 1;
      if (i <= N)
      {
        // skip the first column
        std::getline(ss, field, ',');

        // read the values of the remaining columns and store them in the corresponding vectors
        std::getline(ss, field, ',');
        accelerationRMSRatioCompare.push_back(std::stod(field));

        std::getline(ss, field, ',');
        displacementRMSRatioCompare.push_back(std::stod(field));

        std::getline(ss, field, ',');
        accelerationDelayCompare.push_back(std::stod(field));

        std::getline(ss, field, ',');
        displacementDelayCompare.push_back(std::stod(field));

        std::getline(ss, field, ',');
        accelerationPSDCompare.push_back(std::stod(field));
      }

      else
      {
        std::getline(ss, field, ',');
        std::getline(ss, field, ',');
        std::getline(ss, field, ',');
        std::getline(ss, field, ',');
        std::getline(ss, field, ',');
        std::getline(ss, field, ',');
      }

      std::getline(ss, field, ',');
      // time.push_back(std::stod(field));

      std::getline(ss, field, ',');
      sprungMassJerkCompare.push_back(std::stod(field));

      std::getline(ss, field, ',');
      sprungMassSnapCompare.push_back(std::stod(field));

      std::getline(ss, field, ',');
      sprungMassAccGCompare.push_back(std::stod(field));

      std::getline(ss, field, ',');
      sprungMassDisplacementCompare.push_back(std::stod(field));

      std::getline(ss, field, ',');
      sprungMassPositionCompare.push_back(std::stod(field));
      // std::cout << "\nAcc: " << field;

      ss.clear();
    }
    myFile.close();
  }
  else
  {
    std::cout << "File could not be opened!\n\n\n"
              << std::endl;
    throw std::runtime_error("ERROR: File could not be opened!\n\n\n");
  }

  std::cout << "Frequency Domain Metrics copied sucessfully.";

  std::cout << "Plots copied sucessfully.";
}

void Simulation::ExportMetrics()
{
  int N = frequencies.size();
  std::ofstream fileMetrics("MetricsOutput.csv");
  if (!fileMetrics.is_open())
  {
    std::cout << "Error opening output .csv file" << std::endl;
    throw std::runtime_error("Could not open .csv file");
  }

  fileMetrics << "Frequencies, AccelerationRMSRatio, DisplacementRMSRatio, AccelerationDelay,  DisplacementDelay, Acceleration PSD, Time, Sprung Mass Jerk, Sprung Mass Snap, Sprung Mass Acc G, Sprung Mass Displacement, Sprung Mass Position" << std::endl;

  for (int i = 0; i < numSteps_; i++)
  {
    if (i <= N)
    {
      fileMetrics << frequencies[i] << "," << accelerationRMSRatio[i] << "," << displacementRMSRatio[i] << "," << accelerationDelay[i] << "," << sprungDispDelay[i] << "," << accelerationPSD[i] << ",";
    }
    else
    {
      fileMetrics << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << "," << 0 << ",";
    }
    fileMetrics << time[i] << "," << sprungMassJerk[i] << "," << sprungMassSnap[i] << "," << sprungMassAccG[i] << "," << sprungMassDisplacement[i] << "," << sprungMassPosition[i] << std::endl;
  }

  fileMetrics.close();
}