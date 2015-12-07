#include "hvmc_solver.h"
#include "hvmc_physics.h"
#include "hvmc_world.h"

#include <iostream>

// petit espace à laisser entre les objets pour éviter les intersections intempestives
#define SAFE_GAP 0.f

void Solver::pushConstraint(Constraint *contrainte)
{
    contraintes.push_back(contrainte);
}

void Solver::clear(){
    for(Constraint * contrainte : contraintes)
        delete contrainte;
    contraintes.clear();
}


void Solver::resolve(std::vector<RigidBody *>& rigidBodies, int nbiteration, float dt)
{

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
                displacement.push_back(contrainte->getLambda(rigid)*rigid->im*gradient/bodies.size());
            else
                displacement.push_back(0*gradient);
        }
        for(int j = 0; j<bodies.size();j++)
        {
            RigidBody *rigid = rigidBodies[bodies[j]];
            vec3 newdst = displacement[j];

            //mise à jour de la vélocité en fonction du déplacement imposé
            //fait n'importe quoi si le déplacement est trop grand...
            // c'est les impulsions calculées par fabien qui doivent faire ça
            /*rigid->velocity.x+=newdst.x/dt;
            rigid->velocity.y+=newdst.y/dt;
            rigid->angularVelocity+=newdst.z/dt;*/

            //mise à jour de la position
            rigid->position.x+=newdst.x;
            rigid->position.y+=newdst.y;
            rigid->rotation+=newdst.z;
        }
    }
}

void Constraint::setSystem(std::vector<RigidBody *> &psystem)
{
    system=&psystem;
}

float ArbitraryBox::C()
{
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

float Constraint::getLambda(RigidBody *body)
{
    return -1*C()/std::pow(Length(getGradient(body)),2);
}

// ---- CONTRAINTE SPHERE-SPHERE ----

SphereToSphereConstraint::SphereToSphereConstraint(int sphere1, int sphere2)
{
    bodies.push_back(sphere1);
    bodies.push_back(sphere2);
}

float SphereToSphereConstraint::C()
{
    RigidBody* s1 = system[0][bodies[0]];
    RigidBody* s2 = system[0][bodies[1]];
    float dist = s1->collider.radius + s2->collider.radius + SAFE_GAP;
    return LengthSquared(s1->position - s2->position) - dist*dist;
}

// ---- CONTRAINTE BOX-BOX ----

BoxToBoxConstraint::BoxToBoxConstraint(int box1, int box2)
{
    bodies.push_back(box1);
    bodies.push_back(box2);
}

float BoxToBoxConstraint::C()
{
    RigidBody* b1 = system[0][bodies[0]];
    RigidBody* b2 = system[0][bodies[1]];

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

BoxToSphereConstraint::BoxToSphereConstraint(int box, int sphere)
{
    bodies.push_back(box);
    bodies.push_back(sphere);
}

float BoxToSphereConstraint::C()
{

    RigidBody* b = system[0][bodies[0]];
    RigidBody* s = system[0][bodies[1]];

    vec2 nearest;
    vec2 clamp_box = b->collider.dims/2 + SAFE_GAP;
    vec2 sphere_in_box_world = s->position - b->position;
    nearest = Max(Min(sphere_in_box_world, clamp_box), -clamp_box);
    float dist = LengthSquared(sphere_in_box_world-nearest);
    float radius = s->collider.radius*s->collider.radius;
    return dist - radius;
}
