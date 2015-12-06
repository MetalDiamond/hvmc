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

class Solver
{
private:

    std::vector<Constraint * > contraintes;

public:
    void resolve(std::vector<RigidBody*> rigidBodies, int nbiteration);
    void pushConstraint(Constraint * Contrainte);
    void clear();
};

// contraintes :

class SphereToSphereConstraint : public Constraint
{
    RigidBody* s1;
    RigidBody* s2;
public:
    SphereToSphereConstraint(RigidBody* sphere1, RigidBody* sphere2);
    virtual float C();
};

class BoxToBoxConstraint : public Constraint
{
    RigidBody* b1;
    RigidBody* b2;
public:
    BoxToBoxConstraint(RigidBody* box1, RigidBody* box2);
    virtual float C();
};

class BoxToSphereConstraint : public Constraint
{
    RigidBody* b;
    RigidBody* s;
public:
    BoxToSphereConstraint(RigidBody* box, RigidBody* sphere);
    virtual float C();
};

#endif // SOLVER_H
