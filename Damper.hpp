#ifndef DAMPERS_HPP
#define DAMPERS_HPP

class Damper {
public:
    virtual double getDampingRatio(double velocity) = 0;
    virtual void setDampingRatio(double dampingRatio) = 0;
};

class LinearDamper : public Damper {
private:
    double dampingRatio;
public:
    LinearDamper()=default;
    LinearDamper(double dampingRatio);
    double getDampingRatio(double velocity);
    void setDampingRatio(double dampingRatio);
};

class NonLinearDamper : public Damper {
private:
    double dampingRatio;
public:
    NonLinearDamper()=default;
    NonLinearDamper(double dampingRatio);
    double getDampingRatio(double velocity);
    void setDampingRatio(double dampingRatio);
};

#endif