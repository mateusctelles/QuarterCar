#include "ModelBuilder.hpp"

double ModelBuilder::getSprungMassFromUser(double unit) {
    double sprungMass;
    std::cout << "\nEnter the sprung mass [Kg]: ";
    std::cin >> sprungMass;
    return sprungMass;
}
 
double ModelBuilder::getUnsprungMassFromUser(double unit) {
    double unsprungMass;
    std::cout << "\nEnter the unsprung mass [Kg]: ";
    std::cin >> unsprungMass;
    return unsprungMass;
}

double ModelBuilder::getStiffnessFromUser(double unit) {
    char springType;
    std::cout<<"\nSelect the spring type: (L for linear, N for nonlinear): ";
    std::cin>> springType; 
    if (springType == 'L'){
        double stiffness;
        spring_ = new LinearSpring; // Creates instance of LinearSpring at Runtime.
        std::cout << "Enter the suspension stiffness value [N/mm]: ";
        std::cin >> stiffness;
        return stiffness*unit;
    }

    else if (springType == 'N'){
        double stiffness;
        spring_ = new NonLinearSpring; // Creates instance of NonLinearSpring at Runtime.
        std::cout << "Enter the suspension nonlinear stiffness value [N/m]: ";
        std::cin >> stiffness;
        return stiffness*unit;
    }

    else {
        std::cout<<"\nInvalid Option";
        return 1;
    }
}

double ModelBuilder::getDampingRatioFromUser() {
    char damperType;
    std::cout<<"\nSelect the damper type: (L for linear, N for nonlinear): ";
    std::cin>> damperType; 
    if (damperType == 'L'){
        double dampingRatio;
        damper_ = new LinearDamper; // Creates instance of LinearDamper at Runtime.
        std::cout << "Enter the Damping Ratio: ";
        std::cin >> dampingRatio;
        return dampingRatio;
    }

    else if(damperType=='N'){
        double dampingRatio;
        damper_ = new NonLinearDamper; // Creates instance of NonLinearDamper at Runtime.
        std::cout << "Enter the NonLinear Damping Ratio: ";
        std::cin >> dampingRatio;
        return dampingRatio;
    }

    else {
        std::cout<<"\nInvalid Option";
        return 1;
    }
}

double ModelBuilder::getTravelLimitFromUser(double unit){
    double travelLimit;
    std::cout<<"\nEnter the travel limit [mm]: ";
    std::cin >> travelLimit;
    return travelLimit/unit;
}

double ModelBuilder::getStopperStiffnessFromUser(double unit){
    double kStopper;
    std::cout<<"\nEnter the Bumpstop Stiffness [N/mm]: ";
    std::cin >> kStopper;
    return kStopper*unit;
}

double ModelBuilder::getTireStiffnessFromUser(double unit){
    double kTire;
    std::cout<<"\nEnter the Tire Stifness [N/mm]: ";
    std::cin >> kTire;
    return kTire*unit;
}

double ModelBuilder::getTireDampingFromUser(){
    double cTire;
    std::cout<<"\nEnter the Tire Damping [Ns/m]: ";
    std::cin >> cTire;
    return cTire;
}

void ModelBuilder::getParams(){
   
   std::cout<<" -----------------------------------------------  START ----------------------------------------------- \n\n";
   std::cout<<"Do you wish to define vehicle parameters? Type 'y' to yes, or 'n' to keep the previously defined parameters.\n";
   std::cout<<"Selection: ";
   std::cin>>repeatparam;
   if(repeatparam=='y'){
   //std::cout<<"\n------------------------------------------------------------------------------------------------------------\n";
   std::cout<<"---------------------------------------------- VEHICLE PARAMETERS ------------------------------------------ \n";;
   std::cout<<"\nType '1' to define all vehicle parameters in the console.\n" ;
   std::cout<<"Type '2' to change single selected parameters.\n";
   //std::cout<<"Type '3' to use all vehicle parameters defined in the code. \n";
   std::cout<<"Type '3' to keep the previous vehicle parameters \n"<< std::endl;
   std::cout<<"Selection: ";
   std::cin>>paramtype; 
   std::cout<<"--------------------------------------------------------\n";


   if(paramtype==1){

        std::cout<<"\n------------------------ OPTION 1 SELECTED: ALL VEHICLE PARAMETERS DEFINITION -----------------------\n";   

        //Define the vehicle quarter sprung mass
        m_s = getSprungMassFromUser();
        car_.setSprungMass(m_s);
        
        //Define the suspension stiffness
        k_w=getStiffnessFromUser();
        spring_->setStiffness(k_w);

        //Define the Suspension Damping Ratio;
        DR = getDampingRatioFromUser();
        damper_->setDampingCoef(DR);

        //Define the suspension travel until bumpstop
        travel_limit=getTravelLimitFromUser();
        car_.setMaxTravel(travel_limit);

        // Define bumpstop stiffness
        k_stopper=getStopperStiffnessFromUser();
        car_.setKBumpstop(k_stopper);
        
        //Define the vehicle quarter sprung mass
        m_u=getUnsprungMassFromUser();
        car_.setUnsprungMass(m_u);

        //Define the Tire Vertical Stiffness
        k_t=getTireStiffnessFromUser();
        car_.setKT(k_t);

        //Define the Tire Damping
        c_t=getTireDampingFromUser();
        car_.setCT(c_t);

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
            std::cout<<"(5) Bumpstop Stiffness [N/m]\n";
            std::cout<<"(6) Unsprung Mass [Kg]\n";
            std::cout<<"(7) Tire Vertical Stiffness [N/m]\n";

            std::cout<<"Selection: ";
            std::cin>>param;
            std::cout<<"-----------------------------------------------\n";

            if(param==1){
                m_s = getSprungMassFromUser();
                car_.setSprungMass(m_s);
            }

            else if(param==2){
                k_w=getStiffnessFromUser();
                spring_->setStiffness(k_w);
            }

            else if(param==3){
                DR = getDampingRatioFromUser();
                damper_->setDampingCoef(DR);
            }

            else if(param==4){
                travel_limit=getTravelLimitFromUser();
                car_.setMaxTravel(travel_limit);
            }

            else if(param==5){
                k_stopper=getStopperStiffnessFromUser();
                car_.setKBumpstop(k_stopper);
            }

            else if(param==6){
                m_u=getUnsprungMassFromUser();
                car_.setUnsprungMass(m_u);
            }
                                   
            else if(param==7){
                k_t=getTireStiffnessFromUser();
                car_.setKT(k_t);
            }
                    
            std::cout<<"\nWant to change other parameter? ('y' or 'n'): ";
            std::cin>> repeat1;
            std::cout<<"--------------------------------------------------\n";
        }
            
    }

   /*else if(paramtype==3){
        m_s = 120; 
        k_w = 7680; 
        DR = 0.50; 
        m_u = 30; 
        k_t = 90000; 
        c_t = 0; 
        k_stopper = 900000; 
        travel_limit = 0.80; */
         
    }

    car_.setSpring(spring_);
    car_.setDamper(damper_);
}

Spring* ModelBuilder::getSpring() const{
    return spring_;
}

Damper* ModelBuilder::getDamper() const{
    return damper_;
}

/*void ModelBuilder::setCarProperties(){
    car_.setSpring(spring_);
    car_.setDamper(damper_);
}*/



/*ModelBuilder::~ModelBuilder() {
    delete spring_;
}*/
