#ifndef SOLVER_H
#define SOLVER_H

#include "hvmc_physics.h"

class Constraint
{
protected:
    std::vector<int> bodies;
public:
    virtual float C() = 0;
    virtual vec3 getGradient(RigidBody* body);
    virtual float getLambda() {return 1;}
    const std::vector<int>& getBodies() {return bodies;}
};

class SphereToSphereConstraint : public Constraint
{
public:
    SphereToSphereConstraint(RigidBody* sphere1, RigidBody* sphere2);
    virtual float C();
};

class Solver
{
private:

    std::vector<Constraint * > contraintes;

public:
    void resolve(std::vector<RigidBody*> rigidBodies, int nbiteration);
    void pushConstraint(Constraint * Contrainte);
    void clear();
};

#endif // SOLVER_H
