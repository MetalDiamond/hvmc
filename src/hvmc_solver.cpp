#include "hvmc_solver.h"

#include <iostream>

void Solver::pushConstraint(Constraint *contrainte){
    contraintes.push_back(contrainte);
}

void Solver::clear(){
    for(Constraint * contrainte : contraintes){
        delete contrainte;
    }
    contraintes.clear();
}


void Solver::resolve(std::vector<RigidBody *> rigidBodies, int nbiteration){
    for(int i=0;i<nbiteration;i++)
    for(Constraint * contrainte : contraintes){
        std::vector<vec3> mespos;
        for(int i=0;i<contrainte->getBodies().size();i++){
            RigidBody *rigid = rigidBodies[i];
            mespos.push_back(-1*contrainte->getLambda()*rigid->im*contrainte->getGradient(rigid));
        }
        for(int i=0;i<contrainte->getBodies().size();i++){
            RigidBody *rigid = rigidBodies[i];
            rigid->position.x=mespos[i].x;
            rigid->position.y=mespos[i].y;
            rigid->rotation=mespos[i].z;
        }
    }
}

vec3 Constraint::getGradient(RigidBody* body)
{
    const float epsilon = 0.001f;
    vec3 gradient;
    float baseC = C();

    body->position.x += epsilon;
    gradient.x = C() - baseC;
    body->position.x -= epsilon;

    body->position.y += epsilon;
    gradient.y = C() - baseC;
    body->position.y -= epsilon;

    body->rotation += epsilon;
    gradient.z = C() - baseC;
    body->rotation -= epsilon;

    return gradient / epsilon;
}

SphereToSphereConstraint::SphereToSphereConstraint(RigidBody* sphere1, RigidBody* sphere2)
{
    // TODO
}

float SphereToSphereConstraint::C()
{
    return 0.5;
}
