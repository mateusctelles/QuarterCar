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
  file << "roadDispRMS, sprungDispRMS, roadAccRMS, sprungAccRMS, dispRMSRatio, accRMSRatio, PSD Amplitude, PSDFrequencies" << std::endl;
  for (int i = 0; i < numSteps; i++)
  {
    file << time[i] << "," << roadLine[i] << "," << sprungMassDisplacement[i] << "," << sprungMassAccG[i] << "," << unsprungMassAccG[i] << "," << bumpStopForce[i] << "," << reboundStopForce[i] << "," << stopperForce[i] << "," << tireElasticForce[i] << ","
         << springForce[i] << "," << damperForce[i] << "," << tireDamperForce[i] << "," << bumpStopStiffness[i] << "," << reboundStopStiffness[i] << "," << car.getBumpStopSpring()->getTriggerDistance() << ","
         << relativeDisplacementVector[i] << "," << car.getReboundStopSpring()->getTriggerDistance() << "," << roadLineDisplacementRMS[i] << "," << sprungMassDisplacementRMS[i] << "," << roadLineAccelerationRMS[i] << ","
         << sprungMassDisplacementRMS[i] << "," << displacementRMSRatio[i] << "," << accelerationRMSRatio[i]<<std::endl;
  }

  file.close();
}

void Simulation::InitializeVectors(int numSteps, Car &car)
{
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

  bumpStopForce.resize(numSteps, 0.0);
  reboundStopForce.resize(numSteps, 0.0);
  springForce.resize(numSteps, 0.0);
  damperForce.resize(numSteps, 0.0);
  tireElasticForce.resize(numSteps, 0.0);
  tireDamperForce.resize(numSteps, 0.0);

  sprungMassDisplacementRMS.resize(numSteps, 0.0);
  sprungMassAccelerationRMS.resize(numSteps, 0.0);
  roadLineDisplacementRMS.resize(numSteps, 0.0);
  roadLineAccelerationRMS.resize(numSteps, 0.0);
  displacementRMSRatio.resize(numSteps, 0.0);
  accelerationRMSRatio.resize(numSteps, 0.0);

  frequencies.resize(numSteps, 0.0);
  accelerationPSD.resize(numSteps, 0.0);

  std::cout << "\nResponse Vectors Initialized\n";
}

std::vector<double> Simulation::CalculateSignalRatio(std::vector<double> &inputSignal, std::vector<double> &outputSignal)
{
  int signalSize = inputSignal.size();
  std::vector<double> vectorRatio(signalSize, 0);
  for (int i = 0; i < signalSize; i++)
  {
    vectorRatio[i] = (1 - (outputSignal[i] / inputSignal[i])) * 100;
    // std::cout<<"Ratio: "<<vectorRatio[i]<<", inputSignal: "<<inputSignal[i]<<", outputSignal: "<<outputSignal[i]<<std::endl;
  }

  return vectorRatio;
}

std::vector<double> Simulation::CalculateRMS(std::vector<double> &signal)
{
  int n = signal.size();
  std::vector<double> rms(n, 0);

  for (int i = 0; i < n; i++)
  {
    double sum = 0;
    for (int j = 0; j <= i; j++)
    {
      sum += pow(signal[j], 2);
    }
    rms[i] = sqrt(sum / (i + 1));
  }
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
  double availableBumpTravel = car.getSpring()->getFreeLength() - suspSpringDeflection_;
  double availableReboundTravel = suspSpringDeflection_;

  // Setting Contact Spring parameters for tire and stoppers

  car.getBumpStopSpring()->setTriggerDistance(-availableBumpTravel);
  car.getReboundStopSpring()->setTriggerDistance(-availableReboundTravel);
  car.getTireSpring()->setTriggerDistance(tireSpringDeflection_);
  std::cout << "\n(Inside StaticEquilibrium Method) Available Bump Travel: " << car.getBumpStopSpring()->getTriggerDistance() << ", set by suspSpringDeflection: " << suspSpringDeflection_ << ", and getfreelenth: " << car.getSpring()->getFreeLength();
  std::cout << "\n(Inside StaticEquilibrium Method) Available Rebound Travel: " << car.getReboundStopSpring()->getTriggerDistance();
  car.setStaticHeight(sprungMassInitialPosition);
  car.setTireStaticHeight(unsprungMassInitialPosition);
  car.getSpring()->setPreload(suspSpringDeflection_ * car.getSpring()->getStiffness(0));
  car.getTireSpring()->setPreload(tireSpringDeflection_ * car.getTireSpring()->getStiffness(0));
  int selection;

  std::cout << "\n\n------------------------------------ STATIC EQUILIBRIUM -----------------------------------------";
  std::cout << "\n\nCalculated static equilibrium configuration.";
  std::cout << "\nThe unsprung mass deflection was " << tireSpringDeflection_ << " " << getDisplacementUnit() << ", and the remaining tire travel is: " << unsprungMassInitialPosition << " " << getDisplacementUnit() << ".";
  std::cout << "\nThe sprung mass deflection was " << sprungMassDeflection_ << " " << getDisplacementUnit() << ", and the current Ride Height is: " << sprungMassInitialPosition << getDisplacementUnit() << "."
            << ".\n\nThe available bump travel is : " << availableBumpTravel << " " << getDisplacementUnit() << ".";
  std::cout << "\nThe available rebound travel is: " << availableReboundTravel << " " << getDisplacementUnit() << ".";
  // std::cout << "\nSelect:\n[1] Proceed with the displayed configuration.\n[2] Set a Spring Preload change ride height.\nSelection: ";
  // std::cin >> selection;
  std::cout << "\n---------------------------------------------------------------------------------------------------";
}

std::vector<double> Simulation::computePSD(const std::vector<double> &signal, int N, double sample_rate)
{
    std::cout<<"Calculating PSD. . .";
    
    // Allocate memory for the FFT input and output arrays
    fftw_complex *in, *out;
    in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
    out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);

    // Copy the time signal into the input array
    for (int i = 0; i < N; i++) {
        in[i][0] = signal[i];
        in[i][1] = 0.0;
    }

    // Create the FFT plan
    fftw_plan plan = fftw_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_ESTIMATE);

    // Execute the FFT
    fftw_execute(plan);

    // Calculate the power spectral density
    accelerationPSD.resize(N/2 + 1);
    frequencies.resize(N/2 + 1);
    for (int i = 0; i < N/2 + 1; i++) {
        accelerationPSD[i] = std::sqrt(out[i][0] * out[i][0] + out[i][1] * out[i][1]) / N;
        frequencies[i] = i * sample_rate / N;
    }

    // Free the memory
    fftw_free(in);
    fftw_free(out);
    fftw_destroy_plan(plan);

    return accelerationPSD;
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
  for (int i = 0; i < numSteps - 1; i++)
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
    roadLineVelocity[i + 1] = (roadLine[i + 1] - roadLine[i]) / timeStepSize_;
    roadLineAcceleration[i + 2] = (roadLineVelocity[i + 2] - roadLineVelocity[i + 1]) / timeStepSize_; // Velocity of ground displacement
    sprungMassAccG[i] = sprungMassAcc[i] / (9.81 * gUnitScaling_);
    unsprungMassAccG[i] = unsprungMassAcc[i] / (9.81 * gUnitScaling_);

    // Calculating actual position considering static constants to be visualized on the position plots.
    sprungMassPosition[i + 1] = sprungMassDisplacement[i + 1] + car.getStaticHeight();
    unsprungMassPosition[i + 1] = unsprungMassDisplacement[i + 1] + car.getTireStaticHeight();

    sprungMassNetForce[i] = sprungMassAcc[i] * car.getSprungMass();
    unsprungMassNetForce[i] = unsprungMassAcc[i] * car.getUnsprungMass();

    // Generating the time vector
    time[i + 1] = (i + 1) * timeStepSize_;
    /*if (road.getRoadName() == "Swept Sine")
    {
      frequencyVector[i] = time[i] * (frequencyEnd_ - frequencyStart_) / simulationTime_;
      // velocityVector[i] = frequencyVector[i]*
    }*/
  }

  std::cout << "\nCalculating RMS outputs. . .";

  // Calculating each RMS curve to be used as arguments of CalculateSignalRatio
  sprungMassDisplacementRMS = CalculateRMS(sprungMassDisplacement);
  roadLineDisplacementRMS = CalculateRMS(roadLine);
  sprungMassAccelerationRMS = CalculateRMS(sprungMassAcc);
  roadLineAccelerationRMS = CalculateRMS(roadLineAcceleration);
  int RMSSize = roadLineDisplacementRMS.size();

  // Calculating the ratio between the RMS of the response and roadInput.
  displacementRMSRatio = CalculateSignalRatio(roadLineDisplacementRMS, sprungMassDisplacementRMS);
  accelerationRMSRatio = CalculateSignalRatio(roadLineAccelerationRMS, sprungMassAccelerationRMS);

  // Calculating the PSD
  double sampleRate = 1/timeStepSize_;
  accelerationPSD = computePSD(sprungMassAccG, numSteps, sampleRate);

  // Exporting resampled signal to a new .csv file
  ExportResults(numSteps, car);

  std::cout << "\n\nSimulation completed!";
}

/*std::string Simulation::SettingsBool(int setting)
{
  if (setting == 1)
    return "YES";
  else
  {
    return "NO";
  }
}

int Simulation::GraphSetter(int &setting)
{
  int setting = setting+1;
  if (setting!=1){
    setting = 0;
  }
  return setting;
}
*/
int Simulation::Graph()
{
  int graphCount = 0;
  int graphType = 1;
  int graphOpt;
  int togglePlot;
  int showSprungRel = 0;
  int showUnsprungRel = 0;
  int showRoadProfile = 1;
  int showUnsprungAcc = 0;
  int showSprungAcc = 1;
  int attenuation = 0;
  int PSDPlot = 1;

  // std::cout << "";

  while (true)
  {
    graphCount = 1;
    std::cout << "\nBuilding Plots. . .";
    if (graphType == 2)
    {
      

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
      auto p1 = plot(time, unsprungMassDisplacement);
      //->display_name("Unsprung");
      p1->line_width(wdt);
      p1->display_name("Unsprung");
      hold(on);
      auto p2 = plot(time, sprungMassDisplacement);
      p2->line_width(wdt);
      p2->display_name("Sprung");

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
        auto p5 = plot(time, unsprungMassAccG);
        p5->line_width(wdt);
        p5->display_name("Unsprung");
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
        show();
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
        xlabel("Time [s]");
        ylabel("Displacement " + displacementUnit_);
        hold(on);
      }

      auto ax1 = subplot(3, 2, {2, 4});
      // title("Vertical Position");
      auto p1 = plot(time, unsprungMassPosition);
      //->display_name("Unsprung");
      p1->line_width(wdt);
      p1->display_name("Unsprung");
      hold(on);
      auto p2 = plot(time, sprungMassPosition);
      p2->line_width(wdt);
      p2->display_name("Sprung");

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
          p01->display_name("Sprung Mass Acceleration PSD");
          auto lgd01 = legend(on);
          lgd01->font_name("Arial");
          xlabel("Frequency [Hz]");
          ylabel("g²/Hz");
          xlim({0, 25});
          hold(on);
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
      if (showUnsprungAcc == 1)
      {

        // ax3 -> size(500,500);
        // title("Acceleration");
        auto p5 = plot(time, unsprungMassAccG);
        p5->line_width(wdt);
        p5->display_name("Unsprung");
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
      }
      show();
    }

    int rep;
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

    graphOpt = rep;
    if (graphOpt != 2 && graphOpt != 4)
    {
      break;
    }
    else if (graphOpt == 2)
    {
      while (true)
      {
        std::cout << "\n-----------------------------PLOT SETTINGS------------------------------";
        std::cout << "\n\n[1] Toggle on/off Suspension Spring Travel visualization";
        std::cout << "\n[2] Toggle on/off Tire Spring Travel visualization";
        std::cout << "\n[3] Toggle on/off Road Profile Displacement within Position plot.";
        std::cout << "\n[4] Toggle between Displacement RMS Ratio or Acceleration RMS Ratio";
        std::cout<< " \n[5] Toggle between PSD/RMS";
        std::cout << "\n[6] Toggle Unsprung Mass Acceleration Visualization";
        std::cout << "\n[7] Toggle Sprung Mass Acceleration Visualization";
        std::cout << "\n[8] Toggle Plot Layout (Displacement-Centered / Forces-Centered)";

        std::cout << "\nSelection: ";

        if (std::cin >> togglePlot && (togglePlot >= 1 && togglePlot <= 8))
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
        PSDPlot = PSDPlot+1;
        if (PSDPlot!=1)
          PSDPlot = 0;
        break;

      case 8:
        graphType = graphType + 1;
        if (graphType != 2)
          graphType = 1;
        break;
      }
    }
    else if (graphOpt == 4)
      graphOpt = 2;
  }
  return graphOpt;
}
