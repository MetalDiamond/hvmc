#include "hvmc_solver.h"
#include "hvmc_physics.h"
#include "hvmc_world.h"

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


void Solver::resolve(std::vector<RigidBody *>& rigidBodies, int nbiteration, float dt){

    for(int i=0;i<nbiteration;i++)
    for(Constraint * contrainte : contraintes)
    {
        contrainte->setSystem(rigidBodies);
        std::vector<vec3> displacement;
        const std::vector<int> & bodies = contrainte->getBodies();

        for(int offset : bodies)
        {
            RigidBody *rigid = rigidBodies[offset];
            vec3 gradient = contrainte->getGradient(rigid);

            if(Length(gradient)>0.01)//évite les / en 0 ou simili
                displacement.push_back(contrainte->getLambda(rigid)*rigid->im*gradient);
            else
                displacement.push_back(0*gradient);
        }
        for(int j = 0; j<bodies.size();j++)
        {
            RigidBody *rigid = rigidBodies[bodies[j]];
            vec3 newdst = displacement[j];

            //mise à jour de la vélocité en fonction du déplacement imposé
            //fait n'importe quoi si le déplacement est trop grand...
            rigid->velocity.x+=newdst.x/dt;
            rigid->velocity.y+=newdst.y/dt;
            rigid->angularVelocity+=newdst.z/dt;

            //mise à jour de la position
            rigid->position.x+=newdst.x;
            rigid->position.y+=newdst.y;
            rigid->rotation+=newdst.z;
        }
    }
}

void Constraint::setSystem(std::vector<RigidBody *> &psystem){
    system=&psystem;
}

float ArbitraryBox::C(){
    RigidBody * particle = system[0][bodies[0]];
    //very much realistic
    float bord = 200;
    float bottom = 400;
    float right = 600;

    float to_return = 0;

    vec2 pos = World::PhysicsToGraphicsPos(particle->position);

    if(pos.x<bord)  to_return=pos.x-bord;
    if(pos.x>right) to_return=right-pos.x;
    if(pos.y<bord)  to_return=pos.y-bord;
    if(pos.y>bottom)to_return=bottom-pos.y;

    return to_return;
    return (float)rand()/(float)RAND_MAX/1000.f;
}

vec3 Constraint::getGradient(RigidBody* body)
{
    const float epsilon = 0.1f;
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

float Constraint::getLambda(RigidBody *body){
    return -1*C()/std::pow(Length(getGradient(body)),2);
}

SphereToSphereConstraint::SphereToSphereConstraint(RigidBody* sphere1, RigidBody* sphere2)
{
    // TODO
}

float SphereToSphereConstraint::C()
{
    return 0.5;
}
