#ifndef SOLVER_H
#define SOLVER_H

#include "hvmc_physics.h"

class Contrainte{
public:
    virtual float C(float x, float y, float r)=0;
    virtual vec3 deltaP();
};


class Solver
{
public:
    Solver();
};

#endif // SOLVER_H
