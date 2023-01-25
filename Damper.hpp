#ifndef DAMPERS_HPP
#define DAMPERS_HPP

class Damper {
public:
    
    virtual double getDampingCoef() = 0;
    virtual void setDampingCoef(double coefficient) = 0;
};

class LinearDamper : public Damper {
private:
    double coefficient;
public:
    LinearDamper()=default;
    LinearDamper(double coefficient);
    double getDampingCoef();
    void setDampingCoef(double coefficient);
};

class NonLinearDamper : public Damper {
private:
    double coefficient;
public:
    NonLinearDamper()=default;
    NonLinearDamper(double coefficient);
    double getDampingCoef();
    void setDampingCoef(double coefficient);
};

#endif