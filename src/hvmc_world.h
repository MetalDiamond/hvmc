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

private:
    PhysicsSystem* physics = nullptr; 
    GraphicsSystem* graphics = nullptr;

    std::vector<Entity> entities;
};

#endif

