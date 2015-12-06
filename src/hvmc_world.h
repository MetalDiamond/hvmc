#ifndef HVMC_WORLD_H
#define HVMC_WORLD_H

#include <vector>
#include "hvmc_math.h"

struct SDL_Renderer;
struct RigidBody;
struct GraphicsComponent;
struct PhysicsSystem;
struct GraphicsSystem;

struct Entity
{
    RigidBody* physics;
    GraphicsComponent* graphics;
};

struct World
{

    bool Init( SDL_Renderer* renderer );
    void SetupScene();
    
    void AddBall( vec2 const& pos );
    void AddBox( vec2 const& pos );
    void ThrowBall( vec2 const& pos );
    void ThrowUp( vec2 const& pos );
    
    void Update( f32 dt );
    void Render();
    void Cleanup();


public:
    static vec2 GraphicsToPhysicsPos(const vec2 &v);
    static vec2 PhysicsToGraphicsPos(const vec2 &v);
    static f32 GraphicsToPhysicsRadius(f32 r);
    static vec2 GraphicsToPhysicsDim(const vec2 &v);
private:
    PhysicsSystem* physics = nullptr; 
    GraphicsSystem* graphics = nullptr;

    std::vector<Entity> entities;
};

#endif

