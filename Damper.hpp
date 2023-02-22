#ifndef DAMPERS_HPP
#define DAMPERS_HPP

class Damper {
public:
    virtual double getDampingRatio(double velocity = 0) = 0;
    virtual double getDampingCoefficient(double velocity=0) = 0; 
    virtual void setDampingCoefficient(double dampingCoefficient) = 0;
    virtual void setDampingRatio(double dampingRatio) = 0;
    virtual void setTriggerDistance(double triggerDistance){};

};

class LinearDamper : public Damper {
private:
    double dampingRatio;
    double dampingCoefficient;
public:
    LinearDamper()=default;
    LinearDamper(double dampingRatio);
    double getDampingRatio(double velocity = 0);
    virtual double getDampingCoefficient(double velocity=0){return dampingCoefficient;};
    virtual void setDampingCoefficient(double dampingCoefficient){this-> dampingCoefficient = dampingCoefficient;}
    void setDampingRatio(double dampingRatio);
};

class LinearContactDamper : public Damper{
private:
    double dampingRatio;
    double dampingCoefficient;
    double triggerDistance;
public:
    LinearContactDamper() = default;
    LinearContactDamper(double dampingRatio);
    double getDampingRatio(double displacement = 0);
    double getDampingCoefficient(double displacement = 0);
    void setDampingRatio(double dampingRatio=0);
    void setDampingCoefficient(double dampingCoefficient);
    void setTriggerDistance(double triggerDistance) {this->triggerDistance = triggerDistance;}
};

/*class TireDamper : public Damper {
private:
    double dampingRatio;
    double dampingCoefficient;
public:
    TireDamper()=default;
    TireDamper(double dampingRatio);
    double getDampingRatio(double velocity);
    virtual double getDampingCoefficient(double velocity) = 0; 
    virtual void setDampingCoefficient(double dampingCoefficient) = 0;
    void setDampingRatio(double dampingRatio);
};*/

class NonLinearDamper : public Damper {
private:
    double dampingRatio;
    double dampingCoefficient;
public:
    NonLinearDamper()=default;
    NonLinearDamper(double dampingRatio);
    double getDampingRatio(double velocity);
    virtual double getDampingCoefficient(double velocity){return dampingCoefficient;};
    virtual void setDampingCoefficient(double dampingCoefficient){this-> dampingCoefficient = dampingCoefficient;}
    void setDampingRatio(double dampingRatio);
};

#endif