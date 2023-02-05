#include "Simulation.hpp"
#include <matplot/matplot.h>

Simulation::Simulation(double t_final_In, double dtIn) : simulationTime_(t_final_In), timeStepSize_(dtIn) {}

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
  double dampingRatio = car.getDamper()->getDampingRatio(0);

  suspSpringDeflection_ = (car.getSprungMass() * 9810 / suspStiffness);
  tireSpringDeflection_ = (car.getSprungMass() + car.getUnsprungMass()) * 9810 / tireStiffness;

  sprungMassDeflection_ = suspSpringDeflection_ + tireSpringDeflection_;
  // unsprungMassDeflection = tireSpringDeflection;

  sprungMassInitialPosition = car.getSpring()->getFreeLength() + car.getTireSpring()->getFreeLength() - sprungMassDeflection_;
  unsprungMassInitialPosition = car.getTireSpring()->getFreeLength() - tireSpringDeflection_;
  double availableBumpTravel = car.getSpring()->getFreeLength() - suspSpringDeflection_;
  double availableReboundTravel = sprungMassDeflection_ - tireSpringDeflection_;

  car.getBumpStopSpring()->setAvailableBumpTravel(-car.getSpring()->getFreeLength());
  car.getReboundStopSpring()->setAvailableReboundTravel(0);
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
  //std::cout << "\nSelect:\n[1] Proceed with the displayed configuration.\n[2] Set a Spring Preload change ride height.\nSelection: ";
  //std::cin >> selection;
  std::cout << "\n---------------------------------------------------------------------------------------------------";
}

void Simulation::Simulate(Car &car, Road &road)
{
  StaticEquilibrium(car);

  double tireStiffness = car.getSpring()->getStiffness(0);
  double suspStiffness = car.getSpring()->getStiffness(0);
  double dampingRatio = car.getDamper()->getDampingRatio(0);

  std::cout << "\nStarting Simulation: Level 1";
  double RideFreq = (1 / (2 * M_PI)) * pow(((suspStiffness * tireStiffness) / (suspStiffness + tireStiffness) / car.getSprungMass()), 0.5);
  double damping = (2 * RideFreq * 2 * M_PI * car.getSprungMass()) * dampingRatio;
  std::cout << "\n\nCalculated Stiffness and Ride Freq";

  int numSteps = simulationTime_ / timeStepSize_;
  std::cout << "\n\nCalculated number of steps: " << numSteps;

  // roadLine = std::move(road.CalcRoad(simulationTime_, timeStepSize_));
  roadLine = road.CalcRoad(simulationTime_, timeStepSize_);

  std::cout << "\n\nRoad Calculated and moved to roadLine";

  double f_ride = car.CalcRideFreq();
  double dampingCoefficient = car.CalcSuspDamp(0);

  std::cout << "\nSprung Mass Natural Frequency Calculated: " << car.CalcSprungNatFreq();
  std::cout << "\n\nRide Frequency Calculated. Frequency: " << f_ride;
  std::cout << "\nDamping Coefficient Calculated. Value: " << car.CalcSuspDamp(0) << std::endl;

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

  std::cout << "\nResponse Vectors Initialized\n";
  std::cout << "Initial Ride Height: " << sprungMassPosition[0] << ". Initial Tire Height: " << unsprungMassPosition[0];
  double relativeDisplacement = 0.0;
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
    // std::cout<<"\nCalculating relative displacements and velocities, iteration: "<<i;
    relativeDisplacement = sprungMassDisplacement[i] - unsprungMassDisplacement[i] - suspSpringDeflection_;
    // relativePosition = sprungMassDisplacement[i] + car.getStaticHeight() - unsprungMassDisplacement[i] - car.getTireStaticHeight();

    relativeTireDisplacement = unsprungMassDisplacement[i] - roadLine[i] - tireSpringDeflection_;
    // relativeTirePosition = relativeTireDisplacement + car.getTireStaticHeight();

    relativeVelocity = sprungMassVelocity[i] - unsprungMassVelocity[i];

    relativeDisplacementVector[i] = relativeDisplacement;
    relativeTireDisplacementVector[i] = relativeTireDisplacement;
    tireDefLimit[i] = -car.getTireSpring()->getFreeLength();
    springBumpLimit[i] = car.getBumpStopSpring()->getAvailableBumpTravel();
    springReboundLimit[i] = car.getReboundStopSpring()->getAvailableReboundTravel();
    bumpStopStiffness[i] = car.getBumpStopSpring()->getStiffness(relativeDisplacement);

    // std::cout<<"\nCalculating isolated Bumpforce, iteration: "<<i;
    bumpStopForce[i] = car.getBumpStopSpring()->getStiffness(relativeDisplacement) * (relativeDisplacement - car.getBumpStopSpring()->getAvailableBumpTravel());
    // std::cout<<"\nCalculating isolated Rebound force, iteration: "<<i;
    reboundStopForce[i] = car.getReboundStopSpring()->getStiffness(relativeDisplacement) * (relativeDisplacement - car.getReboundStopSpring()->getAvailableReboundTravel());
    // std::cout<<"\nCalculating combined Stopper Force, iteration: "<<i;
    stopperForce[i] = bumpStopForce[i] + reboundStopForce[i];
    springForce[i] = car.getSpring()->getStiffness(relativeDisplacement) * (relativeDisplacement);
    damperForce[i] = car.CalcSuspDamp(relativeVelocity) * (relativeVelocity);
    tireDamperForce[i] = car.getTireDamper()->getDampingCoefficient() * (unsprungMassVelocity[i] - roadLineVelocity[i]);
    tireElasticForce[i] = car.getTireSpring()->getStiffness(relativeTireDisplacement) * relativeTireDisplacement;

    // std::cout<<"\nCalculating equations of motion, iteration: "<<i;
    sprungMassAcc[i] = ((-springForce[i] - damperForce[i] - stopperForce[i] - sprungMassWeight) / (car.getSprungMass()));                                              // Acceleration of sprung mass
    unsprungMassAcc[i] = (-tireDamperForce[i] - tireElasticForce[i] + springForce[i] + damperForce[i] + stopperForce[i] - unsprungMassWeight) / car.getUnsprungMass(); // Acceleration of unsprung mass
    sprungMassVelocity[i + 1] = sprungMassAcc[i] * timeStepSize_ + sprungMassVelocity[i];                                                                              // Velocity of sprung mass
    unsprungMassVelocity[i + 1] = unsprungMassAcc[i] * timeStepSize_ + unsprungMassVelocity[i];                                                                        // Velocity of unsprung mass
    sprungMassDisplacement[i + 1] = sprungMassVelocity[i] * timeStepSize_ + sprungMassDisplacement[i];                                                                 // Position of sprung mass
    unsprungMassDisplacement[i + 1] = unsprungMassVelocity[i] * timeStepSize_ + unsprungMassDisplacement[i];                                                           // Position of unsprung mass
    roadLineVelocity[i + 1] = (roadLine[i + 1] - roadLine[i]) / timeStepSize_;
    roadLineAcceleration[i + 2] = (roadLineVelocity[i + 2] - roadLineVelocity[i + 1]) / timeStepSize_; // Velocity of ground displacement
    sprungMassAccG[i] = sprungMassAcc[i] / (9.81 * gUnitScaling_);
    unsprungMassAccG[i] = unsprungMassAcc[i] / (9.81 * gUnitScaling_);

    sprungMassPosition[i + 1] = sprungMassDisplacement[i + 1] + car.getStaticHeight();
    unsprungMassPosition[i + 1] = unsprungMassDisplacement[i + 1] + car.getTireStaticHeight();

    sprungMassNetForce[i] = sprungMassAcc[i] * car.getSprungMass();
    unsprungMassNetForce[i] = unsprungMassAcc[i] * car.getUnsprungMass();

    time[i + 1] = (i + 1) * timeStepSize_;

    // std::cout << "\n Sprung Mass Weight: " << sprungMassWeight;
    // std::cout << " | Unsprung Mass Weight: " << unsprungMassWeight;
    // std::cout << " | Unsprung Mass Force: " << unsprungMassNetForce[i];
    // std::cout << " | Sprung Mass Force: " << sprungMassNetForce[i];

    // std::cout<<unsprungMassDisplacement[i] << " ; " << i <<  std::endl;

    // std::cout<< i << std::endl;
  }

  std::cout << "\nCalculating RMS outputs. . .";
  sprungMassDisplacementRMS = CalculateRMS(sprungMassDisplacement);
  roadLineDisplacementRMS = CalculateRMS(roadLine);
  sprungMassAccelerationRMS = CalculateRMS(sprungMassAcc);
  roadLineAccelerationRMS = CalculateRMS(roadLineAcceleration);
  int RMSSize = roadLineDisplacementRMS.size();

  displacementRMSRatio = CalculateSignalRatio(roadLineDisplacementRMS, sprungMassDisplacementRMS);
  accelerationRMSRatio = CalculateSignalRatio(roadLineAccelerationRMS, sprungMassAccelerationRMS);

  // Exporting resampled signal to a new .csv file
  std::ofstream file("SimulationOutput.csv");
  if (!file.is_open())
  {
    std::cout << "Error opening output .csv file" << std::endl;
    throw std::runtime_error("Could not open .csv file");
  }
  file << "Time, RoadLine, SprungMassDisp, Sprung Mass Acc,Unsprung Mass Acc,BumpStopForce,ReboundStop Force,StopperForce,Tire Force,Spring Force,Damper Force, Tire Damper Force, BumpStop Stiffness, ReboundStop Stiffness, availableBumpTravel, Relative Displacement, availableReboundTravel, ";
  file<< "roadDispRMS, sprungDispRMS, roadAccRMS, sprungAccRMS, dispRMSRatio, accRMSRatio"<<std::endl;
  for (int i = 0; i < numSteps; i++)
  {
    file << time[i] << "," << roadLine[i] << "," << sprungMassDisplacement[i] << "," << sprungMassAccG[i] << "," << unsprungMassAccG[i] << "," << bumpStopForce[i] << "," << reboundStopForce[i] << "," << stopperForce[i] << "," << tireElasticForce[i] << ","
         << springForce[i] << "," << damperForce[i] << "," << tireDamperForce[i]<<","<< bumpStopStiffness[i] << "," << reboundStopStiffness[i] << "," << car.getBumpStopSpring()->getAvailableBumpTravel() << ","
         << relativeDisplacementVector[i] << "," << car.getReboundStopSpring()->getAvailableReboundTravel() << "," << roadLineDisplacementRMS[i] << "," << sprungMassDisplacementRMS[i] << "," << roadLineAccelerationRMS[i] << ","
         << sprungMassDisplacementRMS[i] << "," << displacementRMSRatio[i] << "," << accelerationRMSRatio[i] << std::endl;
  }

  file.close();

  std::ofstream fileRMS("RMSOutput.csv");
  if (!fileRMS.is_open())
  {
    std::cout << "Error opening output .csv file" << std::endl;
    throw std::runtime_error("Could not open .csv file");
  }
  std::cout << "\nExporting RMS Calculations. . . ";
  fileRMS << "time, roadDispRMS, sprungDispRMS, roadAccRMS, sprungAccRMS, dispRMSRatio, accRMSRatio" << std::endl;
  for (int i = 0; i < RMSSize; i++)
  {
    fileRMS << time[i] << "," << roadLineDisplacementRMS[i] << "," << sprungMassDisplacementRMS[i] << "," << roadLineAccelerationRMS[i] << "," << sprungMassDisplacementRMS[i] << "," << displacementRMSRatio[i] << "," << accelerationRMSRatio[i] << std::endl;
  }
  fileRMS.close();

  std::cout << "\n\nSimulation completed!";
}

int Simulation::Graph()
{

  int graphType = 1;
  int graphOpt;
  int togglePlot;
  int showSprungRel = 0;
  int showUnsprungRel = 0;
  int showRoadProfile = 1;
  int attenuation = 1;
  while (true)
  {
    if (graphType == 2)
    {
      std::cout << "\nBuilding Plots. . .";

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

      auto p20 = plot(time, relativeDisplacementVector);
      p20->line_width(wdt);
      p20->display_name("Difference");
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
      auto p5 = plot(time, unsprungMassAccG);
      p5->line_width(wdt);
      p5->display_name("Unsprung");
      hold(on);

      auto p6 = plot(time, sprungMassAccG);
      p6->line_width(wdt);
      p6->display_name("Sprung");
      auto lgd3 = legend(on);
      lgd3->font_name("Arial");
      xlabel("Time [s]");
      ylabel("Acceleration " + accelerationUnit_);
      show();
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
        title("Attenuation ");
        if (attenuation == 1)
        {
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
          auto p01 = plot(time, accelerationRMSRatio);
          p01->line_width(wdt);
          p01->display_name("Displacement Attenuation");
          auto lgd01 = legend(on);
          lgd01->font_name("Arial");
          xlabel("Time [s]");
          ylabel("Acceleration RMS Ratio [%]");
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
      // ax3 -> size(500,500);
      // title("Acceleration");
      auto p5 = plot(time, unsprungMassAccG);
      p5->line_width(wdt);
      p5->display_name("Unsprung");
      hold(on);

      auto p6 = plot(time, sprungMassAccG);
      p6->line_width(wdt);
      p6->display_name("Sprung");
      auto lgd3 = legend(on);
      lgd3->font_name("Arial");
      xlabel("Time [s]");
      ylabel("Acceleration " + accelerationUnit_);

      show();
    }

    std::cout << "\nNext Action:";
    std::cout << "\n[1] - Setup and run new simulation with current unit system.";
    std::cout << "\n[2] - Graph Display Settings.";
    std::cout << "\n[3] - Setup and run new simulation with different unit system.";
    std::cout << "\n[4] - Close program";
    std::cout << "\nSelection: ";
    int rep;
    std::cin >> rep;
    graphOpt = rep;
    if (graphOpt != 2)
    {
      break;
    }
    else if (graphOpt == 2)
    {
      std::cout << "\n[1] Toggle on/off Suspension Spring Travel";
      std::cout << "\n[2] Toggle on/off Tire Spring Travel";
      std::cout << "\n[3] Toggle on/off Road Profile Displacement within Position plot.";
      std::cout << "\n[4] Toggle between Displacement RMS Ratio or Acceleration RMS Ratio";
      std::cout << "\n[5] Show displacement plots (instead of position plots).";
      std::cout << "\nSelection: ";
      std::cin >> togglePlot;

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
        graphType = 2;
      }
    }
  }
  return graphOpt;

  // save("img/QuarterCar.jpg");
}
