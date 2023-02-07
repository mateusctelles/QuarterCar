#include "ModelBuilder.hpp"
void ModelBuilder::unitsHandler()
{
    int unitType;
    double dispUnitScaling;
    double massUnitScaling;
    double forceUnitScaling;
    double stiffnessUnitScaling;
    double dampingUnitScaling;
    double gUnitScaling = 9810;

    std::string displacementUnit;
    std::string stiffnessUnit;
    std::string accelerationUnitSI;
    std::string accelerationUnit;
    std::string massUnit;
    std::string dampingUnit;
    std::string forceUnit;
    std::string velocityUnit;
    std::string gUnit;

    std::cout << "\nChoose the unit type for displacement: \n[1] Milimiters [mm], \n[2] Meters [m], \n[3] Consistent Units Convention\nSelection: ";
    std::cin >> unitType;

    switch (unitType)
    {
    case 1:
        dispUnitScaling = 1;
        stiffnessUnitScaling = 1 / dispUnitScaling;
        massUnitScaling = 0.001;
        forceUnitScaling = 1;
        gUnitScaling = 1000;
        dampingUnitScaling = stiffnessUnitScaling;

        velocityUnit = "[mm/s]";
        displacementUnit = "[mm]";
        stiffnessUnit = "[N/mm]";
        accelerationUnitSI = "[mm/s²]";
        massUnit = "[Kg]";
        dampingUnit = "[N.s/mm]";
        forceUnit = "[N]";
        gUnit = "[g]";
        sim.setStiffnessUnitScaling(stiffnessUnitScaling);
        sim.setDampingUnitScaling(dampingUnitScaling);
        sim.setMassUnitScaling(massUnitScaling);
        sim.setgUnitScaling(gUnitScaling);
        sim.setDisplacementUnitScaling(dispUnitScaling);

        sim.setDisplacementUnit(displacementUnit);
        sim.setStiffnessUnit(stiffnessUnit);
        sim.setAccelerationUnit(gUnit);
        sim.setForceUnit(forceUnit);
        sim.setVelocityUnit(velocityUnit);
        sim.setDampingUnit(dampingUnit);
        break;

    case 2:
        dispUnitScaling = 1;
        massUnitScaling = 1;
        forceUnitScaling = 1;
        dampingUnitScaling = 1;
        stiffnessUnitScaling = 1;
        gUnitScaling = 1;
        displacementUnit = "[m]";
        velocityUnit = "[m/s]";
        stiffnessUnit = "[N/m]";
        accelerationUnitSI = "[m/s²]";
        massUnit = "[Kg]";
        dampingUnit = "[N.s/m]";
        sim.setStiffnessUnitScaling(stiffnessUnitScaling);
        sim.setMassUnitScaling(massUnitScaling);
        sim.setDisplacementUnit(displacementUnit);
        sim.setgUnitScaling(gUnitScaling);
        sim.setStiffnessUnit(stiffnessUnit);
        sim.setAccelerationUnit(gUnit);
        sim.setForceUnit(forceUnit);
        sim.setVelocityUnit(velocityUnit);
        sim.setDampingUnit(dampingUnit);
        break;

    case 3:
        dispUnitScaling = 1;
        massUnitScaling = 1;
        forceUnitScaling = 1;
        dampingUnitScaling = 1;
        displacementUnit = "";
        stiffnessUnit = "";
        accelerationUnitSI = "";
        massUnit = "";
        dampingUnit = "";
        break;
    }
}

void ModelBuilder::printAttributes()
{

    std::cout << "\n\n===================================== DEFINED MODEL PARAMETERS =====================================\n\n";
    std::cout << "Suspension Stiffness: " << car_.getSpring()->getStiffness(0);
    std::cout << "\nSuspension Damping Ratio: " << car_.getDamper()->getDampingRatio(0) << " | Damping Coefficient: " << car_.CalcSuspDamp(0) / sim.getDampingUnitScaling() << " " << sim.getDampingUnit();
    std::cout << "\nBumpstop Stiffness: " << car_.getBumpStopSpring()->getStiffness(0) / sim.getStiffnessUnitScaling() << " " << sim.getStiffnessUnit();
    std::cout << "\nAvailable Bump Travel: " << car_.getBumpStopSpring()->getTriggerDistance() / sim.getDisplacementUnitScaling() << " " << sim.getDisplacementUnit();
    std::cout << "\nTire Vertical Stiffness: " << car_.getTireSpring()->getStiffness(0) / sim.getStiffnessUnitScaling() << " " << sim.getStiffnessUnit();
    std::cout << "\nTire Damping: " << car_.getTireDamper()->getDampingCoefficient() / sim.getDampingUnitScaling() << " " << sim.getDampingUnit();
    std::cout << "\nSprung Mass: " << car_.getSprungMass() / sim.getMassUnitScaling() << " " << sim.getMassUnit();
    std::cout << "\nUnsprung Mass: " << car_.getUnsprungMass() / sim.getMassUnitScaling() << " " << sim.getMassUnit();
    std::cout << "\nRide Frequency: " << car_.CalcRideFreq() << " [Hz]";
    std::cout << "\n====================================================================================================\n";
    // std::cout << "Simulation Time: "<<
}

void ModelBuilder::getSprungMassFromUser()
{
    double sprungMass;
    std::cout << "\nEnter the sprung mass " << sim.getMassUnit() << ": ";
    std::cin >> sprungMass;
    car_.setSprungMass(sprungMass * sim.getMassUnitScaling());
}

void ModelBuilder::getUnsprungMassFromUser()
{
    double unsprungMass;
    std::cout << "\nEnter the unsprung mass " << sim.getMassUnit() << ": ";
    std::cin >> unsprungMass;
    car_.setUnsprungMass(unsprungMass * sim.getMassUnitScaling());
}

void ModelBuilder::getTireStiffnessFromUser()
{
    /*Spring *oldTireSpring = car_.getTireSpring();
    if (oldTireSpring != nullptr)
    {
        std::cout << "oldTireSpring  is not nullptr";
        delete oldTireSpring;
        oldTireSpring = nullptr;
    }*/

    double stiffness;
    double height;
    //~delete TireSpring
    std::unique_ptr<Spring>tireSpring = std::make_unique<LinearContactSpring>();
    std::cout << "\nEnter the Tire Stiffness " << sim.getStiffnessUnit() << ": ";
    std::cin >> stiffness;
    tireSpring->setStiffness(stiffness * sim.getStiffnessUnitScaling());
    std::cout << "\nEnter the Tire Sidewall Height " << sim.getDisplacementUnit() << ": ";
    std::cin >> height;
    tireSpring->setFreeLength(height * sim.getStiffnessUnitScaling());
    car_.setTireSpring(std::move(tireSpring));
    std::cout << "Car received Tire Stiffness: \n"
              << car_.getTireSpring()->getStiffness(0) << std::endl;
    std::cout << "Car received Tire Sidewall Height: \n"
              << car_.getTireSpring()->getFreeLength() << std::endl;
    // return tireSpring_;
}

void ModelBuilder::getBumpStopStiffnessFromUser()
{
    /*Spring *oldBumpStopSpring = car_.getBumpStopSpring();
    if (oldBumpStopSpring != nullptr)
    {
        std::cout << "oldBumpStopSpring is not nullptr";
        delete oldBumpStopSpring;
        oldBumpStopSpring = nullptr;
    }*/
    double kStopper;
    std::unique_ptr<Spring> bumpStopSpring = std::make_unique<LinearContactSpring>();
    std::cout << "\nEnter the Bumpstop Stiffness " << sim.getStiffnessUnit() << ": ";
    std::cin >> kStopper;
    bumpStopSpring->setStiffness(kStopper * sim.getStiffnessUnitScaling());
    car_.setBumpStopSpring(std::move(bumpStopSpring));
    std::cout << "\nCar received Bump stop Stiffness: " << car_.getBumpStopSpring()->getStiffness(0);
}

void ModelBuilder::getReboundStopStiffnessFromUser()
{
    double kStopper; 
    std::unique_ptr<Spring> reboundStopSpring = std::make_unique<LinearContactSpring>();
    std::cout << "\nEnter the Rebound Stop Stiffness " << sim.getStiffnessUnit() << ": ";
    std::cin >> kStopper;
    reboundStopSpring->setStiffness(kStopper * sim.getStiffnessUnitScaling());
    car_.setReboundStopSpring(std::move(reboundStopSpring));
    std::cout << "\nCar received Rebound stop Stiffness: " << car_.getReboundStopSpring()->getStiffness(0);
    // delete reboundStopSpring;
}

void ModelBuilder::getStiffnessFromUser()
{
    //Spring *spring;
    std::unique_ptr<Spring> spring;
    char springType;

    std::cout << "\nSelect the spring type: (L for linear, N for nonlinear): ";
    std::cin >> springType;
    if (springType == 'L')
    {
        double stiffness;
        double length;
        spring=std::make_unique<LinearSpring>(); // Creates instance of LinearSpring at Runtime.
        std::cout << "Enter the suspension spring stiffness value " << sim.getStiffnessUnit() << ": ";
        std::cin >> stiffness;
        spring->setStiffness(stiffness * sim.getStiffnessUnitScaling());
        std::cout << "Enter the spring free length value: " << sim.getDisplacementUnit() << ": ";
        std::cin >> length;
        spring->setFreeLength(length * sim.getDisplacementUnitScaling());
        car_.setSpring(std::move(spring));
        std::cout << "\nCar received spring stiffness: " << car_.getSpring()->getStiffness(0);
        std::cout << "\nCar received spring free length: " << car_.getSpring()->getFreeLength();
        // return spring_;
    }
    else if (springType == 'N')
    {
        double stiffness;
        spring = std::make_unique<NonLinearSpring>(); // Creates instance of NonLinearSpring at Runtime.
        std::cout << "Enter the suspension nonlinear stiffness value " << sim.getStiffnessUnit() << ": ";
        std::cin >> stiffness;
        spring->setStiffness(stiffness * sim.getStiffnessUnitScaling());
        std::cout << "\nCar received stiffness: " << spring->getStiffness(0);
        car_.setSpring(std::move(spring));
        // return spring_;
    }
    else
    {
        std::cout << "\nInvalid Option";
        // return nullptr;
    }
}


void ModelBuilder::getTireDampingFromUser()
{
    std::unique_ptr<Damper>tireDamper;
    tireDamper = std::make_unique<LinearDamper>();
    double cTire;
    std::cout << "\nEnter the Tire Damping " << sim.getDampingUnit() << ": ";
    std::cin >> cTire;

    tireDamper->setDampingCoefficient(cTire * sim.getDampingUnitScaling());
    car_.setTireDamper(std::move(tireDamper));
}

void ModelBuilder::getDampingRatioFromUser()
{
    /*Damper* oldDamper = car_.getDamper();
    if(oldDamper != nullptr){
        std::cout<<"oldDamper is not nullptr";
        delete oldDamper;
        oldDamper = nullptr;
    }*/

    std::unique_ptr<Damper>damper;
    char damperType;
    if (userParam == 3)
    {
        damper->setDampingRatio(0.5);
        // return damper_;
    }
    else
    {
        std::cout << "\nSelect the damper type: (L for linear, N for nonlinear): ";
        std::cin >> damperType;
        if (damperType == 'L')
        {
            double dampingRatio;
            damper = std::make_unique<LinearDamper>(); // Creates instance of LinearDamper at Runtime.
            std::cout << "Enter the Damping Ratio: ";
            std::cin >> dampingRatio;
            damper->setDampingRatio(dampingRatio);
            car_.setDamper(std::move(damper));
            // return damper_;
        }

        else if (damperType == 'N')
        {
            double dampingRatio;
            damper = std::make_unique<NonLinearDamper>(); // Creates instance of NonLinearDamper at Runtime.
            std::cout << "Enter the NonLinear Damping Ratio: ";
            std::cin >> dampingRatio;
            damper->setDampingRatio(dampingRatio);
            car_.setDamper(std::move(damper));
            // return damper_;
        }

        else
        {
            std::cout << "\nInvalid Option";
            // return nullptr;
        }
    }
}

void ModelBuilder::getStaticHeightFromUser()
{
    double staticHeight;
    std::cout << "\nEnter the distance between wheel center and sprung mass" << sim.getDisplacementUnit() << ": ";
    std::cin >> staticHeight;
    car_.setStaticHeight(staticHeight * sim.getDisplacementUnitScaling());
}

void ModelBuilder::getVehicleParams()
{

    std::cout << " -----------------------------------------------  START ----------------------------------------------- \n\n";
    std::cout << "Do you wish to define vehicle parameters? Type 'y' to yes, or 'n' to keep the previously defined parameters.\n";
    std::cout << "Selection: ";
    std::cin >> repeatparam;
    if (repeatparam == 'y')
    {
        // std::cout<<"\n------------------------------------------------------------------------------------------------------------\n";
        std::cout << "---------------------------------------------- VEHICLE PARAMETERS ------------------------------------------ \n";
        ;
        std::cout << "\nType '1' to define all vehicle parameters in the console.\n";
        std::cout << "Type '2' to change single selected parameters.\n";
        // std::cout<<"Type '3' to use all vehicle parameters defined in the code. \n";
        std::cout << "Type '3' to use model template parameters \n"
                  << std::endl;
        std::cout << "Selection: ";
        std::cin >> paramtype;
        std::cout << "--------------------------------------------------------\n";

        if (paramtype == 1)
        {

            std::cout << "\n------------------------ OPTION 1 SELECTED: ALL VEHICLE PARAMETERS DEFINITION -----------------------\n";

            // Define the vehicle quarter sprung mass
            getSprungMassFromUser();

            // Define the suspension stiffness and the spring free length
            getStiffnessFromUser();

            // Define the Suspension Damping Ratio;
            getDampingRatioFromUser();

            // Define bumpstop stiffness
            getBumpStopStiffnessFromUser();

            // Define bumpstop stiffness
            getReboundStopStiffnessFromUser();

            // Define the vehicle quarter sprung mass
            getUnsprungMassFromUser();

            // Define the Tire Vertical Stiffness and SideWall Height
            getTireStiffnessFromUser();

            // Define the Tire Damping
            getTireDampingFromUser();
        }

        else if (paramtype == 2)
        {
            char repeatParam = 'y';
            int param;
            while (repeatParam == 'y')
            {

                std::cout << "\nWhich parameter you wish to change? \n\n";
                std::cout << "(1) Sprung Mass " << sim.getMassUnit();
                std::cout << "\n(2) Suspension Stiffness and Spring Free Length " << sim.getStiffnessUnit();
                std::cout << "\n(3) Suspension Damping Ratio " << sim.getDampingUnit();
                // std::cout << "\n(4) Suspension Travel Limit " << sim.getDisplacementUnit();
                // std::cout << "\n(5) Sprung Mass Static Height to the Tire " << sim.getDisplacementUnit();
                std::cout << "\n(4) Bumpstop Stiffness " << sim.getStiffnessUnit();
                std::cout << "\n(5) Rebound Stop Stiffness " << sim.getStiffnessUnit();
                std::cout << "\n(6) Unsprung Mass " << sim.getMassUnit();
                std::cout << "\n(7) Tire Vertical Stiffness and Sidewall Height" << sim.getStiffnessUnit();
                std::cout << "\n(8) Tire Vertical Damping: " << sim.getDampingUnit();

                std::cout << "\nSelection: ";
                std::cin >> param;
                std::cout << "-----------------------------------------------\n";

                switch (param)
                {
                case 1:
                    getSprungMassFromUser();
                    break;

                case 2:
                    getStiffnessFromUser();
                    break;

                case 3:
                    getDampingRatioFromUser();
                    break;

                case 4:
                    getBumpStopStiffnessFromUser();
                    break;

                case 5:
                    getReboundStopStiffnessFromUser();
                    break;

                case 6:
                    getUnsprungMassFromUser();
                    break;

                case 7:
                    getTireStiffnessFromUser();
                    break;
                case 8:
                    getTireDampingFromUser();
                    break;
                }

                std::cout << "\nWant to change other parameter? ('y' or 'n'): ";
                std::cin >> repeatParam;
                std::cout << "--------------------------------------------------\n";
            }
        }

        else if (paramtype == 3)
        {
            /*userParam = paramtype;
            car_.setSprungMass(120 * sim.getMassUnitScaling());
            car_.setSpring(getStiffnessFromUser());
            car_.setDamper(getDampingRatioFromUser());
            car_.setUnsprungMass(20 * sim.getMassUnitScaling());
            car_.setTireDamping(0);*/
            // car_.setKBumpstop(1000 * sim.getStiffnessUnitScaling());
            //  car_.setMaxTravel(100 * sim.getDisplacementUnitScaling());
            // car_.setTireStiffness(90 * sim.getStiffnessUnitScaling());
        }

        // car_.setSpring(spring_);
        // car_.setDamper(damper_);
    }
}

//************************************************ Functions to Build Road******************************************************
void ModelBuilder::getSineRoadFromUser()
{

    Road *oldSineRoad = road_;
    if (oldSineRoad != nullptr)
    {
        std::cout << "oldSineRoad is not nullptr";
        delete oldSineRoad;
        oldSineRoad = nullptr;
    }

    double amplitude;
    double frequency;
    roadName_ = "Sine";
    std::cout << "\nSine Wave input method selected.";
    std::cout << "\nDefine the Sine Wave Amplitude " << sim.getDisplacementUnit() << ": ";
    std::cin >> amplitude; // meters
    std::cout << "Define the sine frequency [Hz]: ";
    std::cin >> frequency;

    road_ = new SineRoad(amplitude, frequency);
}

void ModelBuilder::getCurbRoadFromUser()
{
    double height;
    std::cout << "\nCurb road input method selected.";
    std::cout << "\nDefine Curb Height " << sim.getDisplacementUnit() << ": ";
    std::cin >> height;
    road_ = new CurbRoad(height);
}

void ModelBuilder::getSweptSineRoadFromUser()
{
    double startingFrequency;
    double endingFrequency;
    double amplitude;
    roadName_ = "Swept Sine";
    std::cout << "\nSwept Sine input method selected.";
    std::cout << "\nDefine the start of frequency range [Hz]: ";
    std::cin >> startingFrequency;
    std::cout << "\nDefine the end of the frequency range [Hz]: ";
    std::cin >> endingFrequency;
    std::cout << "Define the Amplitude of the swept sine wave " << sim.getDisplacementUnit() << ": ";
    std::cin >> amplitude;
    road_ = new SweptSine(amplitude, startingFrequency, endingFrequency);
}

void ModelBuilder::getRampRoadFromUser()
{
    double height;
    double inclination;
    roadName_ = "Ramp";
    std::cout << "\nRamp Sequence input method selected.";
    std::cout << "\nDefine the height of the ramp " << sim.getDisplacementUnit() << ": ";
    std::cin >> height;
    std::cout << "\nDefine the inclination of the ramp ";
    std::cin >> inclination;
    road_ = new RampRoad(height, inclination);
}

void ModelBuilder::getFileRoadFromUser()
{
    std::string filename;
    double scaling;
    char filter;
    double windowSize;
    roadName_ = "File Import";
    std::cout << "\n\nImport File input method selected. File must have a displacement time signal with a time duration equal or greater than the simulation time.";
    std::cout << "\nWrite the file name with the file extension. If file is in different folder than executable, provide file path.\n";
    std::cout << "\nFile: ";
    std::cin >> filename;
    std::cout << "\n--------------------------------------------\n";
    std::cout << "\nDefine the scaling of the signal. Type '1' if no scaling is desired.\n";
    std::cout << "\nScaling: ";
    std::cin >> scaling;
    std::cout << "\n--------------------------------------------\n";
    std::cout << "\nDo you wish to filter the signal (Moving Average Method)? Type 'y' or 'n': ";
    std::cin >> filter;
    if (filter == 'y')
    {
        std::cout << "Define the window size [number of samples]: ";
        std::cin >> windowSize;
    }
    road_ = new FileRoad(filename, filter, windowSize, scaling);
}

// ************************************************* Function to build Simulation *************************************************
void ModelBuilder::getSimParams()
{

    int inputsim;
    int inptype;
    int changeinp;

    std::cout << "\n\n--------------------------------------- SIMULATION PARAMETERS -------------------------------------------- \n";
    // std::cout<<"\nSIMULATION PARAMETERS:\n";
    // std::cout<<"\nType '0' if you want to define simulation parameters in the console, type '1' if want to use the parameters in the code, \ntype '2' if want to keep parameters from last run.\n";
    std::cout << "\n[1] Define simulation parameters in the console\n";
    std::cout << "[2] Keep parameters from previous run. \n";
    // std::cout<<"[3] Keep parameters from previous run.\n";

    std::cout << "\nSelection: ";
    std::cin >> inputsim;
    std::cout << "\n--------------------------------------------\n";
    if (inputsim == 3)
    {

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

    else if (inputsim == 1)
    {
        double simulationTime;
        double timeStepSize;
        std::cout << "\nInsert Simulation Time [s]: ";
        std::cin >> simulationTime;
        sim.setSimTotalTime(simulationTime);
        std::cout << "\nInsert Step Size [s]: ";
        std::cin >> timeStepSize;
        sim.setSimStepSize(timeStepSize);
        std::cout << "\n--------------------------------------------";
        std::cout << "\n[1] Define signal input type and its parameters.";
        std::cout << "\n[2] Keep the same settings from last run.\n";
        std::cout << "\nSelection: ";
        std::cin >> changeinp;
        std::cout << "\n--------------------------------------------";

        if (changeinp == 1)
        {

            std::cout << "\nDefine Input Type: (1) Sine Wave     (2) Swept Sine      (3) Ramp       (4) Import Displacement Signal from File     (5) Curb Road\n";
            std::cout << "Selection: ";
            std::cin >> inptype;
            std::cout << "\n--------------------------------------------";

            if (inptype == 1)
            {
                getSineRoadFromUser();
            }
            else if (inptype == 2)
            {
                getSweptSineRoadFromUser();
            }
            else if (inptype == 3)
            {
                getRampRoadFromUser();
            }
            else if (inptype == 4)
            {
                getFileRoadFromUser();
            }
            else if (inptype == 5)
            {
                getCurbRoadFromUser();
            }
        }
    }

    else if (inputsim == 2)
    {
    }
};