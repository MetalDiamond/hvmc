#ifndef SOLVER_H
#define SOLVER_H
#include <vector>
#include "hvmc_math.h"

class RigidBody;

class Constraint
{
protected:
    std::vector<int> bodies;
    std::vector<RigidBody *> * system;
public:
    void setSystem(std::vector<RigidBody *>& system);
    virtual float C() = 0;
    virtual vec3 getGradient(RigidBody* body);
    virtual float getLambda(RigidBody * body);
    const std::vector<int>& getBodies() {return bodies;}
};

class ArbitraryBox : public Constraint{
public:
    ArbitraryBox(int offset){
        bodies.push_back(offset);
    }
    float C();
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
    void resolve(std::vector<RigidBody *>&  rigidBodies, int nbiteration, float deltaT);
    void pushConstraint(Constraint * Contrainte);
    void clear();
};

#endif // SOLVER_H
