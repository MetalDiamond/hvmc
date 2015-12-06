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

class Solver
{
private:

    std::vector<Constraint * > contraintes;

public:
    void resolve(std::vector<RigidBody *>&  rigidBodies, int nbiteration, float deltaT);
    void pushConstraint(Constraint * Contrainte);
    void clear();
};

// contraintes :

class SphereToSphereConstraint : public Constraint
{
public:
    SphereToSphereConstraint(int sphere1, int sphere2);
    float C();
};

class BoxToBoxConstraint : public Constraint
{
public:
    BoxToBoxConstraint(int box1, int box2);
    float C();
};

class BoxToSphereConstraint : public Constraint
{
    RigidBody* b;
    RigidBody* s;
public:
    BoxToSphereConstraint(int box, int sphere);
    float C();
};

#endif // SOLVER_H
