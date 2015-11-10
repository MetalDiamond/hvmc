#ifndef SOLVER_H
#define SOLVER_H

#include "hvmc_physics.h"

class Contrainte{
protected:
    std::vector<int> points;
public:
    virtual float C(int id)=0;
    virtual vec3 deltaP();
};


class Solver
{
public:
    Solver();
};

#endif // SOLVER_H
