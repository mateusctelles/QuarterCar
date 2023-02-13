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

    while (true)
    {
        std::cout << "\nChoose the unit type for displacement: \n[1] Milimiters [mm], \n[2] Meters [m], \n[3] Consistent Units Convention\nSelection: ";
        if (std::cin >> unitType && (unitType >= 1 && unitType <= 3))
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
        sim.setMassUnit(massUnit);
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
        sim.setMassUnit(massUnit);
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
}

void ModelBuilder::getSprungMassFromUser()
{
    double sprungMass;
    while (true)
    {
        std::cout << "\nEnter the Sprung Mass " << sim.getMassUnit() << ": ";
        if (std::cin >> sprungMass)
        {
            break;
        }
        else
        {
            std::cout << "Error: Invalid input. Please enter a valid number." << std::endl;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }

    car_.setSprungMass(sprungMass * sim.getMassUnitScaling());
}
void ModelBuilder::getUnsprungMassFromUser()
{
    double unsprungMass;
    while (true)
    {
        std::cout << "\nEnter the Unsprung Mass " << sim.getMassUnit() << ": ";
        if (std::cin >> unsprungMass)
        {
            break;
        }
        else
        {
            std::cout << "Invalid input. Please enter a valid number." << std::endl;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }
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
    std::unique_ptr<Spring> tireSpring = std::make_unique<LinearContactSpring>();

    while (true)
    {
        std::cout << "\nEnter the Tire Stiffness " << sim.getStiffnessUnit() << ": ";
        if (std::cin >> stiffness)
        {
            break;
        }
        else
        {
            std::cout << "Invalid input. Please enter a valid number." << std::endl;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }

    tireSpring->setStiffness(stiffness * sim.getStiffnessUnitScaling());

    while (true)
    {
        std::cout << "\nEnter the Tire Sidewall Height " << sim.getDisplacementUnit() << ": ";
        if (std::cin >> height)
        {
            break;
        }
        else
        {
            std::cout << "Invalid input. Please enter a valid number." << std::endl;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }

    tireSpring->setFreeLength(height * sim.getStiffnessUnitScaling());
    car_.setTireSpring(std::move(tireSpring));
    // std::cout << "Car received Tire Stiffness: \n"              << car_.getTireSpring()->getStiffness(0) << std::endl;
    // std::cout << "Car received Tire Sidewall Height: \n"              << car_.getTireSpring()->getFreeLength() << std::endl;
    //  return tireSpring_;
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

    while (true)
    {
        std::cout << "\nEnter the Bumpstop Stiffness " << sim.getStiffnessUnit() << ": ";
        if (std::cin >> kStopper)
        {
            break;
        }
        else
        {
            std::cout << "Invalid input. Please enter a valid number." << std::endl;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }

    bumpStopSpring->setStiffness(kStopper * sim.getStiffnessUnitScaling());
    car_.setBumpStopSpring(std::move(bumpStopSpring));
    // std::cout << "\nCar received Bump stop Stiffness: " << car_.getBumpStopSpring()->getStiffness(0);
}

void ModelBuilder::getReboundStopStiffnessFromUser()
{
    double kStopper;
    std::unique_ptr<Spring> reboundStopSpring = std::make_unique<LinearContactSpring>();
    while (true)
    {
        std::cout << "\nEnter the Rebound Stop Stiffness " << sim.getStiffnessUnit() << ": ";
        if (std::cin >> kStopper)
        {
            break;
        }
        else
        {
            std::cout << "Invalid input. Please enter a number value." << std::endl;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }
    reboundStopSpring->setStiffness(kStopper * sim.getStiffnessUnitScaling());
    car_.setReboundStopSpring(std::move(reboundStopSpring));
    // std::cout << "\nCar received Rebound stop Stiffness: " << car_.getReboundStopSpring()->getStiffness(0);
}

void ModelBuilder::getStiffnessFromUser()
{
    std::unique_ptr<Spring> spring;
    int springType;

    while (true)
    {
        std::cout << "\nSelect the Spring type: Enter [1] for Linear or [2] for Nonlinear: ";
        std::cout << "\nSelection: ";
        if (std::cin >> springType && (springType == 1 || springType == 2))
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

    if (springType == 1)
    {
        double stiffness;
        double length;
        spring = std::make_unique<LinearSpring>(); // Creates instance of LinearSpring at Runtime.
        while (true)
        {
            std::cout << "\nEnter the Suspension Spring Stiffness " << sim.getStiffnessUnit() << ": ";
            if (std::cin >> stiffness)
            {
                break;
            }
            else
            {
                std::cout << "Invalid input. Please enter a valid value." << std::endl;
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
        }
        spring->setStiffness(stiffness * sim.getStiffnessUnitScaling());

        while (true)
        {
            std::cout << "\nEnter the spring Free Length: " << sim.getDisplacementUnit() << ": ";
            if (std::cin >> length)
            {
                break;
            }
            else
            {
                std::cout << "Invalid input. Please enter a valid value." << std::endl;
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
        }

        spring->setFreeLength(length * sim.getDisplacementUnitScaling());
        car_.setSpring(std::move(spring));
        // std::cout << "\nCar received spring stiffness: " << car_.getSpring()->getStiffness(0);
        // std::cout << "\nCar received spring free length: " << car_.getSpring()->getFreeLength();
    }

    else if (springType == 2)
    {
        double stiffness;
        spring = std::make_unique<NonLinearSpring>(); // Creates instance of NonLinearSpring at Runtime.
        while (true)
        {
            std::cout << "\nEnter the Suspension Nonlinear Stiffness " << sim.getStiffnessUnit() << ": ";
            if (std::cin >> stiffness)
            {
                break;
            }
            else
            {
                std::cout << "Invalid input. Please enter a valid value." << std::endl;
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
        }
        spring->setStiffness(stiffness * sim.getStiffnessUnitScaling());
        // std::cout << "\nCar received stiffness: " << spring->getStiffness(0);
        car_.setSpring(std::move(spring));
    }
}

void ModelBuilder::getTireDampingFromUser()
{
    std::unique_ptr<Damper> tireDamper;
    tireDamper = std::make_unique<LinearDamper>();
    double cTire;
    while (true)
    {
        std::cout << "\nEnter the Tire Damping " << sim.getDampingUnit() << ": ";
        if (std::cin >> cTire)
        {
            break;
        }
        else
        {
            std::cout << "Invalid input. Please enter a valid value." << std::endl;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }

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

    std::unique_ptr<Damper> damper;
    int damperType;

    while (true)
    {
        std::cout << "\nSelect the Damper Type: Enter [1] for Linear, [2] for Nonlinear: ";
        if (std::cin >> damperType && (damperType == 1 || damperType == 2))
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
    if (damperType == 1)
    {
        double dampingRatio;
        damper = std::make_unique<LinearDamper>(); // Creates instance of LinearDamper at Runtime.
        while (true)
        {
            std::cout << "Enter the Damping Ratio: ";
            if (std::cin >> dampingRatio)
            {
                break;
            }
            else
            {
                std::cout << "\nInvalid input. Please enter a valid value." << std::endl;
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
        }
        damper->setDampingRatio(dampingRatio);
        car_.setDamper(std::move(damper));
    }

    else if (damperType == 'N')
    {
        double dampingRatio;
        damper = std::make_unique<NonLinearDamper>(); // Creates instance of NonLinearDamper at Runtime.
        while (true)
        {
            std::cout << "Enter the NonLinear Damping Ratio: ";
            if (std::cin >> dampingRatio)
            {
                break;
            }
            else
            {
                std::cout << "\nInvalid input. Please enter a valid ." << std::endl;
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
        }
        damper->setDampingRatio(dampingRatio);
        car_.setDamper(std::move(damper));
    }

    else
    {
        std::cout << "\nInvalid Option";
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
    if (usage_ == 0)
    {
        // std::cout << "\nfirstUsage: " << usage_;
        repeatparam = 'y';
    }
    else
    {
        // std::cout << "\nelsefirstUsage: " << usage_;
        while (true)
        {
            std::cout << "Do you wish to define vehicle parameters? Type 'y' to yes, or 'n' to keep the previously defined parameters.\n";
            std::cout << "Selection: ";
            if (std::cin >> repeatparam && (repeatparam == 'y' || repeatparam == 'n'))
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
    }

    if (repeatparam == 'y')
    {
        // std::cout << "\nrepeatparam Usage: " << usage_;
        // std::cout << "\nrepeatparam: " << repeatparam;
        std::cout << "---------------------------------------------- VEHICLE PARAMETERS ------------------------------------------ \n";

        if (usage_ == 0)
        {
            paramtype = 1;
        }
        else
        {
            // std::cout << usage_;
            while (true)
            {

                std::cout << "\nType [1] to define all vehicle parameters in the console.\n";
                std::cout << "Type [2] to change only selected parameters.\n";
                std::cout << "Selection: ";
                if (std::cin >> paramtype && (paramtype == 1 || paramtype == 2))
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
        }
    }

    else
    {
        paramtype = 0;
    }

    if (paramtype == 1)
    {
        // std::cout << "\nparamtype Usage: " << usage_;

        std::cout << "\n------------------------------------  ALL VEHICLE PARAMETERS DEFINITION ------------------------------------\n";

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
        std::cout << "\n------------------------ OPTION 2 SELECTED: SELECT SINGLE PARAMETER DEFINITION -----------------------\n";
        char repeatParam = 'y';
        int param;
        while (repeatParam == 'y')
        {

            std::cout << "\nWhich parameter you wish to change? \n";
            std::cout << "\n(1) Sprung Mass " << sim.getMassUnit();
            std::cout << "\n(2) Suspension Stiffness and Spring Free Length " << sim.getStiffnessUnit();
            std::cout << "\n(3) Suspension Damping Ratio " << sim.getDampingUnit();
            std::cout << "\n(4) Bumpstop Stiffness " << sim.getStiffnessUnit();
            std::cout << "\n(5) Rebound Stop Stiffness " << sim.getStiffnessUnit();
            std::cout << "\n(6) Unsprung Mass " << sim.getMassUnit();
            std::cout << "\n(7) Tire Vertical Stiffness and Sidewall Height" << sim.getStiffnessUnit();
            std::cout << "\n(8) Tire Vertical Damping: " << sim.getDampingUnit();

            while (true)
            {
                std::cout << "\nSelection: ";
                if (std::cin >> param && (param >= 1 && param <= 8))
                {
                    break;
                }
                else
                {
                    std::cout << "Invalid input. Please enter a valid option." << std::endl;
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                }
                std::cout << "-----------------------------------------------\n";
            }
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

            while (true)
            {
                std::cout << "\nWant to change other parameter? ('y' or 'n'): ";
                if (std::cin >> repeatParam && (repeatParam == 'y' || repeatParam == 'n'))
                {
                    break;
                }
                else
                {
                    std::cout << "Invalid input. Please enter a valid option." << std::endl;
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                }
                std::cout << "--------------------------------------------------\n";
            }
        }
    }
}

//************************************************ Functions to Build Road******************************************************
void ModelBuilder::getSineRoadFromUser()
{

    double amplitude;
    double frequency;
    roadName_ = "Sine";
    std::cout << "\nSine Wave input method selected.";
    while (true)
    {
        std::cout << "\nDefine the Sine Wave Amplitude " << sim.getDisplacementUnit() << ": ";
        if (std::cin >> amplitude)
        {
            break;
        }
        else
        {
            std::cout << "Invalid input. Please enter a valid value." << std::endl;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }

    while (true)
    {
        std::cout << "\nDefine the sine frequency [Hz]: ";
        if (std::cin >> frequency)
        {
            break;
        }
        else
        {
            std::cout << "Invalid input. Please enter a valid value." << std::endl;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }
    road_ = std::make_unique<SineRoad>(amplitude, frequency);
}

void ModelBuilder::getCurbRoadFromUser()
{
    double height;
    std::cout << "\nCurb road input method selected.";
    while (true)
    {
        std::cout << "\nDefine Curb Height " << sim.getDisplacementUnit() << ": ";
        if (std::cin >> height)
        {
            break;
        }
        else
        {
            std::cout << "Invalid input. Please enter a valid value." << std::endl;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }
    road_ = std::make_unique<CurbRoad>(height);
}

void ModelBuilder::getSweptSineRoadFromUser()
{
    double startingFrequency;
    double endingFrequency;
    double amplitude;
    roadName_ = "Swept Sine";
    std::cout << "\nSwept Sine input method selected.";
    while (true)
    {
        std::cout << "\nDefine the start of frequency range [Hz]: ";
        if (std::cin >> startingFrequency)
        {
            break;
        }
        else
        {
            std::cout << "Invalid input. Please enter a valid value." << std::endl;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }

    sim.setFrequencyStart(startingFrequency);
    while (true)
    {
        std::cout << "\nDefine the end of the frequency range [Hz]: ";
        if (std::cin >> endingFrequency)
        {
            break;
        }
        else
        {
            std::cout << "Invalid input. Please enter a valid value." << std::endl;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }

    sim.setFrequencyEnd(endingFrequency);

    while (true)
    {
        std::cout << "\nDefine the Amplitude of the swept sine wave " << sim.getDisplacementUnit() << ": ";
        if (std::cin >> amplitude)
        {
            break;
        }
        else
        {
            std::cout << "Invalid input. Please enter a valid value." << std::endl;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }

    road_ = std::make_unique<SweptSine>(amplitude, startingFrequency, endingFrequency);
}

void ModelBuilder::getRampRoadFromUser()
{
    double height;
    double inclination;
    int type;
    double inclinationRate;
    std::cout << "\nRamp Sequence input method selected.";
    std::cout << "\nSelect desired type of Ramp event: \n[1] Equal ramps sequence.\n[2] One ramp to a constant height.  \n[3] Varying Inclination Ramps Sequence.";
    std::cout<<"\n[4] Parabolic Ramps";
    std::cout << "\nSelection: ";
    std::cin >> type;

    while (true)
    {
        std::cout << "\nDefine the height of the ramp " << sim.getDisplacementUnit() << ": ";
        if (std::cin >> height)
        {
            break;
        }
        else
        {
            std::cout << "Invalid input. Please enter a valid value." << std::endl;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }
    switch (type)
    {
    case 1:
    case 2:
        while (true)
        {
            std::cout << "\nDefine the inclination of the ramp: ";
            if (std::cin >> inclination)
            {
                break;
            }
            else
            {
                std::cout << "Invalid input. Please enter a valid value." << std::endl;
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
        }

        road_ = std::make_unique<RampRoad>(height, inclination, type);
        break;

    case 3:
        while (true)
        {
            std::cout << "\nDefine starting inclination of the ramp: ";
            if (std::cin >> inclination)
            {
                break;
            }
            else
            {
                std::cout << "Invalid input. Please enter a valid value." << std::endl;
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
        }

        while (true)
        {
            std::cout << "\nDefine inclination rate of change (height/time)/time: ";
            if (std::cin >> inclinationRate)
            {
                break;
            }
            else
            {
                std::cout << "Invalid input. Please enter a valid value." << std::endl;
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
        }
        road_ = std::make_unique<RampRoad>(height, inclination, type, inclinationRate);
        break;

    case 4:
        while (true)
        {
            std::cout << "\nDefine starting inclination of the ramp: ";
            if (std::cin >> inclination)
            {
                break;
            }
            else
            {
                std::cout << "Invalid input. Please enter a valid value." << std::endl;
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
        }

        while (true)
        {
            std::cout << "\nDefine inclination rate of change (height/time)/time: ";
            if (std::cin >> inclinationRate)
            {
                break;
            }
            else
            {
                std::cout << "Invalid input. Please enter a valid value." << std::endl;
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
        }
        road_ = std::make_unique<RampRoad>(height, inclination, type, inclinationRate);
        break;
    }
}

void ModelBuilder::getFileRoadFromUser()
{
    std::string filename;
    double scaling;
    char filter;
    double windowSize;
    roadName_ = "File Import";
    std::cout << "\n\nImport File input method selected. File must have a displacement time signal with a time duration equal or greater than the simulation time.";

    while (true)
    {
        std::cout << "\nWrite the file name with the file extension. If file is in different folder than executable, provide file path.\n";
        std::cout << "\nFile: ";
        if (std::cin >> filename)
        {
            break;
        }
        else
        {
            std::cout << "Invalid input. Please enter a valid file name or file path." << std::endl;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }
    std::cout << "\n--------------------------------------------\n";
    while (true)
    {
        std::cout << "\nDefine the scaling of the signal. Type '1' if no scaling is desired.\n";
        std::cout << "\nScaling: ";
        if (std::cin >> scaling)
        {
            break;
        }
        else
        {
            std::cout << "Invalid input. Please enter a valid value." << std::endl;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }
    std::cout << "\n--------------------------------------------\n";
    while (true)
    {
        std::cout << "\nDo you wish to filter the signal (Moving Average Method)? Type 'y' or 'n': ";
        if (std::cin >> filter && (filter == 'y' || filter == 'n'))
        {
            break;
        }
        else
        {
            std::cout << "\nInvalid input. Please enter a valid option." << std::endl;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }
    if (filter == 'y')
    {
        while (true)
        {
            std::cout << "Define the window size [number of samples]: ";
            if (std::cin >> windowSize)
            {
                break;
            }
            else
            {
                std::cout << "Invalid input. Please enter a valid value." << std::endl;
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
        }
    }

    road_ = std::make_unique<FileRoad>(filename, filter, windowSize, scaling);
}

// ************************************************* Function to build Simulation *************************************************
void ModelBuilder::getSimParams()
{

    int inputsim;
    int inptype;
    int changeinp;

    std::cout << "\n\n --------------------------------------------------- SIMULATION PARAMETERS ---------------------------------------------------- \n";

    if (usage_ == 0)
    {
        inputsim = 1;
    }
    else
    {
        while (true)
        {
            std::cout << "\n[1] Define simulation parameters in the console.\n";
            std::cout << "[2] Keep parameters from previous run. \n";
            std::cout << "\nSelection: ";
            if (std::cin >> inputsim && (inputsim == 1 || inputsim == 2))
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
        std::cout << "\n--------------------------------------------\n";
    }

    if (inputsim == 1)
    {
        double simulationTime;
        double timeStepSize;
        while (true)
        {
            std::cout << "\nInsert Simulation Time [s]: ";
            if (std::cin >> simulationTime)
            {
                break;
            }
            else
            {
                std::cout << "Invalid input. Please enter a valid value." << std::endl;
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
        }
        sim.setSimTotalTime(simulationTime);
        while (true)
        {
            std::cout << "\nInsert Step Size [s]: ";
            if (std::cin >> timeStepSize && (timeStepSize <= simulationTime))
            {
                break;
            }
            else
            {
                std::cout << "Invalid input. Please enter a valid option. Time Step Size must be a number smaller than simulation time." << std::endl;
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
        }
        sim.setSimStepSize(timeStepSize);

        if (usage_ == 0)
        {
            changeinp = 1;
        }
        else
        {
            while (true)
            {
                std::cout << "\n--------------------------------------------";
                std::cout << "\n[1] Define signal input type and its parameters.";
                std::cout << "\n[2] Keep the same settings from last run.\n";
                std::cout << "\nSelection: ";
                if (std::cin >> changeinp && (changeinp == 1 || changeinp == 2))
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
        }
        std::cout << "\n--------------------------------------------";

        if (changeinp == 1)
        {
            while (true)
            {
                std::cout << "\nDefine Input Type: (1) Sine Wave     (2) Swept Sine      (3) Ramp       (4) Import Displacement Signal from File     (5) Curb Road\n";
                std::cout << "Selection: ";
                if (std::cin >> inptype && (inptype >= 1 && inptype <= 5))
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
    usage_ = usage_ + 1;
};