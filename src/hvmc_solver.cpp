#include "hvmc_solver.h"

#include <iostream>

void Solver::pushConstraint(Contrainte *contrainte){
    contraintes.push_back(contrainte);
}

void Solver::clear(){
    for(Contrainte * contrainte : contraintes){
        delete contrainte;
    }
    contraintes.clear();
}


void Solver::resolve(std::vector<RigidBody *> rigidBodies, int nbiteration){
    for(int i=0;i<nbiteration;i++)
    for(Contrainte * contrainte : contraintes){
        std::vector<vec3> mespos;
        for(int i=0;i<contrainte->points.size();i++){
            RigidBody *rigid = rigidBodies[i];
            mespos.push_back(-1*contrainte->getlambda(i)*rigid->im*contrainte->getgradient(i));
        }
        for(int i=0;i<contrainte->points.size();i++){
            RigidBody *rigid = rigidBodies[i];
            rigid->position.x=mespos[i].x;
            rigid->position.y=mespos[i].y;
            rigid->rotation=mespos[i].z;
        }
    }
}
