#include "hvmc_world.h"

#include <assert.h>

#include "hvmc_graphics.h"
#include "hvmc_physics.h"
#include <SDL2/SDL.h>

namespace
{

//static const vec2 graphicsWorldSize = { 1280.f, 720.f };
//static const vec2 physicsWorldSize = { 50.f, 50.f * 9.f / 16.f };

static const vec2 graphicsWorldSize = { 800.f, 600.f };
static const vec2 physicsWorldSize = { 50.f, 50.f * 3.f / 4.f };

static const f32 physicsToGraphicsW = graphicsWorldSize.x / physicsWorldSize.x;
static const f32 physicsToGraphicsH = graphicsWorldSize.y / physicsWorldSize.y;
static const f32 graphicsToPhysicsW = physicsWorldSize.x / graphicsWorldSize.x;
static const f32 graphicsToPhysicsH = physicsWorldSize.y / graphicsWorldSize.y;

}

vec2 World::GraphicsToPhysicsPos( vec2 const& v )
{
    vec2 result = { v.x * graphicsToPhysicsW, physicsWorldSize.y - v.y * graphicsToPhysicsH };
    return result;
}

vec2 World::PhysicsToGraphicsPos( vec2 const& v )
{
    vec2 result = { v.x * physicsToGraphicsW, graphicsWorldSize.y - v.y * physicsToGraphicsH };
    return result;
}

f32 World::GraphicsToPhysicsRadius( f32 r )
{
    f32 result = r * graphicsToPhysicsW;
    return result;
}

vec2 World::GraphicsToPhysicsDim( vec2 const& v )
{
    vec2 result = { v.x * graphicsToPhysicsW, v.y * graphicsToPhysicsH };
    return result;
}

bool World::Init( SDL_Renderer* renderer )
{
    physics = new PhysicsSystem;
    if ( !physics->Init() )
    {
        return false;
    }

    graphics = new GraphicsSystem;
    if ( !graphics->Init( renderer ) )
    {
        return false;
    }

    return true;
}

void World::Update( f32 dt )
{
    physics->Update( dt );
}

void World::Render()
{
    for ( auto& ent : entities )
    {
        vec2 pos = PhysicsToGraphicsPos( ent.physics->position );
        pos.x -= ( f32 )ent.graphics->rect->w * 0.5f;
        pos.y -= ( f32 )ent.graphics->rect->h * 0.5f;
        ent.graphics->rect->x = pos.x;
        ent.graphics->rect->y = pos.y;
        ent.graphics->rotation = ent.physics->rotation;
    }

    graphics->Render();
}

void World::SetupScene()
{
    // Create walls
    {
        Entity entity;

        vec2 gpos{ 0.f, 0.f };
        vec2 gdim{ 800.f, 16.f };
        vec2 ppos = GraphicsToPhysicsPos( gpos );
        vec2 pdim = GraphicsToPhysicsDim( gdim );

        ppos = ppos + vec2{ pdim.x * 0.5f, -pdim.y * 0.5f };

        entity.graphics = graphics->AddWall( gpos, gdim );
        entity.physics = physics->AddWall( ppos, pdim );

        entities.push_back( entity );
    }

    {
        Entity entity;

        vec2 gpos{ 0.f, 584.f };
        vec2 gdim{ 800.f, 16.f };
        vec2 ppos = GraphicsToPhysicsPos( gpos );
        vec2 pdim = GraphicsToPhysicsDim( gdim );

        ppos = ppos + vec2{ pdim.x * 0.5f, -pdim.y * 0.5f };

        entity.graphics = graphics->AddWall( gpos, gdim );
        entity.physics = physics->AddWall( ppos, pdim );

        entities.push_back( entity );
    }

    {
        Entity entity;

        vec2 gpos{ 0.0f, 0.0f };
        vec2 gdim{ 16.f, 800.f };
        vec2 ppos = GraphicsToPhysicsPos( gpos );
        vec2 pdim = GraphicsToPhysicsDim( gdim );

        ppos = ppos + vec2{ pdim.x * 0.5f, -pdim.y * 0.5f };

        entity.graphics = graphics->AddWall( gpos, gdim );
        entity.physics = physics->AddWall( ppos, pdim );

        entities.push_back( entity );
    }

    {
        Entity entity;

        vec2 gpos{ 784.f, 0.f };
        vec2 gdim{ 16.f, 800.f };
        vec2 ppos = GraphicsToPhysicsPos( gpos ); 
        vec2 pdim = GraphicsToPhysicsDim( gdim );

        ppos = ppos + vec2{ pdim.x * 0.5f, -pdim.y * 0.5f };

        entity.graphics = graphics->AddWall( gpos, gdim );
        entity.physics = physics->AddWall( ppos, pdim );
        
        entities.push_back( entity );
    }
}

void World::AddBall( vec2 const& pos )
{
    Entity entity;

    f32 grad = 32.f;
    f32 prad = GraphicsToPhysicsRadius( grad );
    vec2 gpos{ pos.x - grad, pos.y - grad };
    vec2 ppos = GraphicsToPhysicsPos( gpos ) + vec2{ prad, -prad };
    
    entity.graphics = graphics->AddSphere( gpos, grad );
    entity.physics = physics->AddSphere( ppos, prad );

    entities.push_back( entity );
}

void World::AddBox( vec2 const& pos )
{
    Entity entity;

    vec2 gdim = { 32.f, 32.f };
    vec2 gpos = pos - gdim * 0.5f;
    vec2 pdim = GraphicsToPhysicsDim( gdim );
    vec2 ppos = GraphicsToPhysicsPos( gpos ) + vec2{ pdim.x * 0.5f, -pdim.y * 0.5f };
    
    entity.graphics = graphics->AddBox( gpos, gdim );
    entity.physics = physics->AddBox( ppos, pdim );

    entities.push_back( entity );
}

void World::ThrowBall( vec2 const& pos )
{
    Entity entity;

    f32 grad = 32.f;
    f32 prad = GraphicsToPhysicsRadius( grad );
    vec2 gpos{ pos.x - grad, pos.y - grad };
    vec2 ppos = GraphicsToPhysicsPos( gpos ) + vec2{ prad, -prad };

    entity.graphics = graphics->AddSphere( gpos, grad );
    entity.physics = physics->AddSphere( ppos, prad );
    entity.physics->ApplyImpulse( { 25.f, 10.f }, { 50.f, 10.f } );

    entities.push_back( entity );
}

void World::ThrowUp( vec2 const& pos )
{
    for(Entity &e : entities)
    {
        if(e.physics->collider.type == RIGID_BODY_SPHERE)
        {
            vec2 mousePos = GraphicsToPhysicsPos(pos);
            float radius = e.physics->collider.radius;
            if(LengthSquared(e.physics->position - mousePos) < radius*radius)
                e.physics->ApplyImpulse({0.f, 10.f}, mousePos);
        }
    }
}

void World::Cleanup()
{
    if ( physics )
    {
        physics->Cleanup();
    }
    delete physics;
    physics = nullptr;

    if ( graphics )
    {
        graphics->Cleanup();
    }
    delete graphics;
    graphics = nullptr;
}

