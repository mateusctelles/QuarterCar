#include "Simulation.hpp"
#include <matplot/matplot.h>

Simulation::Simulation(double t_final_In, double dtIn) : simulationTime_(t_final_In), timeStepSize_(dtIn) {}

void Simulation::Simulate(Car &car, Road &road)
{

  std::cout << "\nStarting Simulation: Level 1";
  double RideFreq = (1 / (2 * M_PI)) * pow(((car.getSpring()->getStiffness(0) * car.getTireStiffness()) / (car.getSpring()->getStiffness(0) + car.getTireStiffness()) / car.getSprungMass()), 0.5);
  double damping = (2 * RideFreq * 2 * M_PI * car.getSprungMass()) * car.getDamper()->getDampingRatio(0);
  std::cout << "\n\nCalculated Stiffness and Ride Freq";

  int numSteps = simulationTime_ / timeStepSize_;
  std::cout << "\n\nCalculated number of steps: " << numSteps;

  // roadLine = std::move(road.CalcRoad(simulationTime_, timeStepSize_));
  roadLine = road.CalcRoad(simulationTime_, timeStepSize_);

  std::cout << "\n\nRoad Calculated and moved to roadLine";
  // double f_ride = RideFreq;
  // double c_s = damping;

  double f_ride = car.CalcRideFreq();
  double dampingCoefficient = car.CalcSuspDamp(0);

  std::cout << "\n\nRide Frequency Calculated. Frequency: " << f_ride << std::endl;
  std::cout << "\n\nDamping Ratio Calculated. Value: " << dampingCoefficient << std::endl;

  unsprungMassAcc.resize(numSteps, 0.0);
  unsprungMassAccG.resize(numSteps, 0.0);
  unsprungMassVelocity.resize(numSteps, 0.0);
  unsprungMassDisplacement.resize(numSteps, 0.0);
  sprungMassAcc.resize(numSteps, 0.0);
  sprungMassAccG.resize(numSteps, 0.0);
  sprungMassVelocity.resize(numSteps, 0.0);
  sprungMassDisplacement.resize(numSteps, 0.0);
  roadLineVelocity.resize(numSteps, 0.0);
  relativeDisplacementVector.resize(numSteps, 0.0);
  sprungMassPosition.resize(numSteps, car.getStaticHeight());
  unsprungMassPosition.resize(numSteps, 0.0);

  bumpStopForce.resize(numSteps, 0.0);
  springForce.resize(numSteps, 0.0);
  damperForce.resize(numSteps, 0.0);
  tireElasticForce.resize(numSteps, 0.0);
  tireDamperForce.resize(numSteps, 0.0);

  std::cout << "\nResponse Vectors Initialized\n";
  double relativeDisplacement = 0.0;
  double relativeVelocity = 0.0;
  double staticHeight = 100;

  time.resize(numSteps, 0.0);

  std::cout << "\nTime Vector Initialized\n";

  std::cout << "\nStarting Simulation: Level 2 - Calculations";

  // Calculating Outputs: Forces, Displacements, Velocities and Accelerations
  for (int i = 0; i < numSteps - 1; i++)
  {

    relativeDisplacement = sprungMassDisplacement[i] - unsprungMassDisplacement[i];
    relativeVelocity = sprungMassVelocity[i] - unsprungMassVelocity[i];

    relativeDisplacementVector[i] = sprungMassDisplacement[i] + car.getStaticHeight() - unsprungMassDisplacement[i];

    if (relativeDisplacementVector[i] < car.getMaxBumpTravel() && relativeDisplacementVector[i] > 0)
    {
      bumpStopForce[i] = 0;
    }

    else if (relativeDisplacementVector[i] > car.getMaxBumpTravel() + car.getStaticHeight())
    {
      bumpStopForce[i] = car.getBumpStopStiffness() * (relativeDisplacementVector[i] - car.getMaxBumpTravel() - car.getStaticHeight());
    }

    else if (relativeDisplacementVector[i] < 0)
    {
      bumpStopForce[i] = car.getBumpStopStiffness() * relativeDisplacementVector[i];
    }
    // time[i] = i * timeStepSize_;
    // std::cout << "Time: "<<time[i]
    //         << " | BumpstopForce: " << bumpStopForce[i] << " | bumpStopStiffness: " << car.getBumpStopStiffness() << " | relativeDisplacement: " << relativeDisplacement << " | car.MaxBumpTravel: " << car.getMaxBumpTravel() << std::endl;

    // springForce[i] = car.getSpring()->getStiffness((unsprungMassDisplacement[i])) * (sprungMassDisplacement[i] - unsprungMassDisplacement[i]); //*stiffnessUnitScaling_;
    springForce[i] = car.getSpring()->getStiffness(relativeDisplacement) * (relativeDisplacement);
    damperForce[i] = car.CalcSuspDamp(relativeVelocity) * (relativeVelocity);
    tireDamperForce[i] = car.getTireDamping() * (unsprungMassVelocity[i] - roadLineVelocity[i]);
    tireElasticForce[i] = car.getTireStiffness() * (unsprungMassDisplacement[i] - roadLine[i]);

    /*if (unsprungMassDisplacement[i] - roadLine[i]  0)
    {
      tireElasticForce[i] = car.getTireStiffness() * (unsprungMassDisplacement[i] - roadLine[i]);
    }
    else
    {
      tireElasticForce[i] = 0;
    }*/

    sprungMassAcc[i] = ((-springForce[i] - damperForce[i] - bumpStopForce[i]) / (car.getSprungMass()));                       // Acceleration of sprung mass
    unsprungMassAcc[i] = (-tireElasticForce[i] + springForce[i] + damperForce[i] + bumpStopForce[i]) / car.getUnsprungMass(); // Acceleration of unsprung mass
    sprungMassVelocity[i + 1] = sprungMassAcc[i] * timeStepSize_ + sprungMassVelocity[i];                                     // Velocity of sprung mass
    unsprungMassVelocity[i + 1] = unsprungMassAcc[i] * timeStepSize_ + unsprungMassVelocity[i];                               // Velocity of unsprung mass
    sprungMassDisplacement[i + 1] = sprungMassVelocity[i] * timeStepSize_ + sprungMassDisplacement[i];                        // Position of sprung mass
    unsprungMassDisplacement[i + 1] = unsprungMassVelocity[i] * timeStepSize_ + unsprungMassDisplacement[i];                  // Position of unsprung mass
    roadLineVelocity[i + 1] = (roadLine[i + 1] - roadLine[i]) / timeStepSize_;                                                // Velocity of ground displacement
    sprungMassAccG[i] = sprungMassAcc[i] / (9.81 * gUnitScaling_);
    unsprungMassAccG[i] = unsprungMassAcc[i] / (9.81 * gUnitScaling_);

    sprungMassPosition[i + 1] = sprungMassDisplacement[i + 1] + car.getStaticHeight();

    time[i + 1] = (i + 1) * timeStepSize_;

    // std::cout<<unsprungMassDisplacement[i] << " ; " << i <<  std::endl;
    // std::cout<< i << std::endl;
  }

  std::cout << "\n\nSimulation completed!";
}

char Simulation::Graph()
{

  int graphType = 2;

  if (graphType == 1)
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
    /*auto p20 = plot(time, relativeDisplacementVector);
    p20->line_width(wdt);
    p20->display_name("Difference");*/
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

    auto p9 = plot(time, bumpStopForce);
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

  else if (graphType == 2)
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

    auto ax1 = subplot(3, 2, {2, 4});
    // title("Vertical Position");
    auto p1 = plot(time, unsprungMassDisplacement);
    //->display_name("Unsprung");
    p1->line_width(wdt);
    p1->display_name("Unsprung");
    hold(on);
    auto p2 = plot(time, sprungMassPosition);
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

    auto p9 = plot(time, bumpStopForce);
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

  std::cout << "\nWant to run a new simulation (y - Yes, u - Yes, and change unit system before new simulation, n = No.)? ";
  char rep;
  std::cin >> rep;

  return rep;
  // save("img/QuarterCar.jpg");
}
