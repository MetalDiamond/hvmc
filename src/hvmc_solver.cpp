#include "hvmc_solver.h"

#include <iostream>

// petit espace à laisser entre les objets pour éviter les intersections intempestives
#define SAFE_GAP 0.01f

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

// ---- CONTRAINTE SPHERE-SPHERE ----

SphereToSphereConstraint::SphereToSphereConstraint(RigidBody* sphere1, RigidBody* sphere2) :
    s1(sphere1), s2(sphere2) {}

float SphereToSphereConstraint::C()
{
    float dist = s1->collider.radius + s2->collider.radius + SAFE_GAP;
    return LengthSquared(s1->position - s2->position) - dist*dist;
}

// ---- CONTRAINTE BOX-BOX ----

BoxToBoxConstraint::BoxToBoxConstraint(RigidBody* box1, RigidBody* box2) :
    b1(box1), b2(box2) {}

float BoxToBoxConstraint::C()
{
    vec2 diff = b1->position - b2->position;
    vec2 dims = (b1->collider.dims + b2->collider.dims)/2 + SAFE_GAP;
    float h = fabs(diff.x) - dims.x;
    float v = fabs(diff.y) - dims.y;
    if(h<0 || v<0)
        return fmax(h, v);
    else
        return fmin(h, v);
}

// ---- CONTRAINTE BOX-SPHERE ----

BoxToSphereConstraint::BoxToSphereConstraint(RigidBody* box, RigidBody* sphere) :
    b(box), s(sphere) {}

float BoxToSphereConstraint::C()
{
    vec2 nearest;
    vec2 clamp_box = b->collider.dims/2 + SAFE_GAP;
    vec2 sphere_in_box_world = s->position - b->position;
    nearest = Max(Min(sphere_in_box_world, clamp_box), -clamp_box);
    float dist = LengthSquared(sphere_in_box_world-nearest);
    float radius = s->collider.radius*s->collider.radius;
    return dist - radius;
}
