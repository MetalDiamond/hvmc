#ifndef SOLVER_H
#define SOLVER_H

#include "hvmc_physics.h"

class Contrainte{
protected:
    friend class Solver;
    std::vector<int> points;
public:
    virtual float C(int id)=0;
    virtual vec3 getlambda(int p)=0;
    virtual vec3 getgradient(int p)=0;
    virtual ~Contrainte();
};


class Solver
{
private:

    std::vector<Contrainte * > contraintes;

public:
    void resolve(std::vector<RigidBody*> rigidBodies, int nbiteration);
    void pushConstraint(Contrainte * contrainte);
    void clear();
};

#endif // SOLVER_H
