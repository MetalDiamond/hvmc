#include "hvmc_physics.h"
#include "hvmc_world.h"
#include <iostream>

void RigidBody::Update( f32 dt )
{
}

void RigidBody::ApplyForce( vec2 const& f )
{
    forces += f;
}

void RigidBody::ApplyImpulse( vec2 const& impulse, vec2 const& contactVector )
{
    if(collider.type == RIGID_BODY_BOX)
        velocity += impulse/im;
    else
    {
        velocity += impulse/im;//*intensity;
        angularVelocity -= 0.3*Length(impulse)*Cross(contactVector-position, impulse);
    }
}

void RigidBody::IntegrateVelocities(f32 dt) {
    position += velocity * dt;
    rotation += angularVelocity * dt;
}

void RigidBody::IntegrateForces(f32 dt) {
    velocity += forces * dt;
    forces = {0.0, 0.0};
    angularVelocity += torque * dt;
    torque = 0.0;
}

void RigidBody::SetKinematic()
{
    I = iI = m = im = 0.f;
}

bool PhysicsSystem::Init()
{
    gravity = vec2{ 0.f, -9.81f };
    return true;
}

void PhysicsSystem::Cleanup()
{
    rigidBodies.clear();
}

RigidBody* PhysicsSystem::AddSphere( vec2 const& pos, f32 radius )
{
    RigidBody* body = new RigidBody;
    
    body->forces = { 0.f, 0.f };
    body->im = 1.f; // 1 kg
    body->iI = 1.f;
    body->position = pos;
    body->velocity = { 0.f, 0.f };

    body->collider.type = RIGID_BODY_SPHERE;
    body->collider.radius = radius;

    rigidBodies.push_back( body );
    return body;
}

RigidBody* PhysicsSystem::AddBox( vec2 const& pos, vec2 const& dims )
{
    RigidBody* body = new RigidBody; 
    
    body->forces = { 0.f, 0.f };
    body->im = 1.f; // 1 kg
    body->position = pos;
    body->velocity = { 0.f, 0.f };
    
    body->collider.type = RIGID_BODY_BOX;
    body->collider.dims = dims;

    rigidBodies.push_back( body );
    return body;
}

RigidBody* PhysicsSystem::AddWall( vec2 const& pos, vec2 const& dims )
{
    RigidBody* body = new RigidBody;

    body->im = 0.f;
    body->position = pos;

    body->collider.type = RIGID_BODY_BOX;
    body->collider.dims = dims;

    rigidBodies.push_back( body );
    return body;
}

void PhysicsSystem::Update( f32 dt )
{
    for(RigidBody* body : rigidBodies)
    {
        body->ApplyForce(body->im * gravity);
    }

    //Gestion des bords
    for (unsigned int i=0; i<rigidBodies.size(); i++) {

        //DEBUG reset pos
        RigidBody *a = rigidBodies[i];

        vec2 pos = World::PhysicsToGraphicsPos(a->position);

        // loop

        pos.x -= 800 * floor(pos.x / 800);
        pos.y -= 600 * floor(pos.y / 600);

        a->position = World::GraphicsToPhysicsPos(pos);
    }


    for (unsigned int i=0; i<rigidBodies.size()-1; ++i)
    {
        for(unsigned int j=i+1; j<rigidBodies.size(); ++j)
        {
            RigidBody *a = rigidBodies[i];
            RigidBody *b = rigidBodies[j];
            if(a->im > 0 || b->im > 0) // if at least one of the two bodies is not kinematic
            {
                CollisionInfo colInfo;

                if (Collisions::Collide(a, b, colInfo))
                {


                    if (colInfo.type == SPHERE_TO_SPHERE) {
                        system.pushConstraint(new SphereToSphereConstraint(i, j));

                        /*deltaAB = a->velocity - b->velocity;
                        deltaBA = b->velocity - a->velocity;*/

                        // Calcul d'angle sortant selon l'angle incident
                        vec2 impA = (a->velocity - 2.0 * Dot(colInfo.normal, a->velocity) * colInfo.normal) * 0.8;
                        vec2 impB = (b->velocity - 2.0 * Dot(colInfo.normal, b->velocity) * colInfo.normal) * 0.8;

                        a->velocity = impA;
                        b->velocity = impB;

                        /*a->ApplyImpulse(colInfo.normal * Length(a->velocity), colInfo.intersection);
                        b->ApplyImpulse(colInfo.normal * Length(b->velocity), colInfo.intersection);*/

                    } else if (colInfo.type == SPHERE_TO_BOX) {
                        if(b->collider.type == RIGID_BODY_SPHERE)
                            system.pushConstraint(new BoxToSphereConstraint(i, j));
                        else
                            system.pushConstraint(new BoxToSphereConstraint(j, i));

                        //TODO
                        // Problème de stabilité (:lawl: coincé dans les murs)
                        // Fonctionne seulement avec les murs kinématics
                        // Abstraction pour que a soit tjrs la sphere et b la box
                        if (colInfo.boxSideCol == SIDE_EDGE)
                            b->velocity.x = b->velocity.x * -0.8;
                        else
                            b->velocity.y = b->velocity.y * -0.8;

                    } else if (colInfo.type == BOX_TO_BOX) {
                        system.pushConstraint(new BoxToBoxConstraint(i, j));
                    }

                    float ea = Length(a->velocity) + fabs(a->angularVelocity);
                    float eb = Length(b->velocity) + fabs(b->angularVelocity);
                    float ratio = Dot(colInfo.normal, Normalize(a->velocity - b->velocity));
                    //a->ApplyImpulse(colInfo.normal*(ea+eb)*ratio, colInfo.intersection);
                }
            }
        }
    }

    system.resolve(rigidBodies,5,dt);
    system.clear();

    for(RigidBody* body : rigidBodies)
    {
        body->IntegrateForces(dt);
        body->IntegrateVelocities(dt);
    }
}

