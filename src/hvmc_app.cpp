#include "hvmc_app.h"

#include <assert.h>
#include <SDL2/SDL.h>

bool HVMC_App::Init()
{
    if ( SDL_Init( SDL_INIT_VIDEO ) != 0 )
    {
        fprintf( stderr, "SDL_Init Error : %s\n", SDL_GetError() );
        return false;
    }

    window = SDL_CreateWindow( "2D Physics Engine", 100, 100, 800, 600, SDL_WINDOW_SHOWN );
    if ( nullptr == window )
    {
        fprintf( stderr, "SDL_CreateWindow Error : %s\n", SDL_GetError() );
        return false;
    }

    renderer = SDL_CreateRenderer( window, -1, SDL_RENDERER_ACCELERATED |
                                                 SDL_RENDERER_PRESENTVSYNC );
    if ( nullptr == renderer )
    {
        fprintf( stderr, "SDL_CreateRenderer Error : %s\n", SDL_GetError() );
        return false;
    }

    if ( !world.Init( renderer ) )
    {
        fprintf( stderr, "Error in World::Init()\n" );
        return false;
    }

    return true;
}

void HVMC_App::SetupScene()
{
    SDL_SetRenderDrawColor( renderer, 67, 176, 42, 255 );

    world.SetupScene();
}

void HVMC_App::GameLoop()
{
    running = true;
    
    while ( running )
    {
        ProcessEvents();
        Update( 1.0 / 60.0 );
        Render();
    }
}

void HVMC_App::Cleanup()
{
    if ( renderer )
    {
        SDL_DestroyRenderer( renderer );
    }
    
    if ( window )
    {
        SDL_DestroyWindow( window );
    }
    
    SDL_Quit();
}

void HVMC_App::ProcessEvents()
{
    SDL_Event event;
    while ( SDL_PollEvent( &event ) )
    {
        if ( ( event.type == SDL_QUIT ) ||
             ( event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_q ) ||
             ( event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE ) )
        {
            running = false;
            return;
        }
    }

    if ( event.type == SDL_MOUSEBUTTONDOWN )
    {
        vec2 pos{ (f32)event.button.x, (f32)event.button.y };
        if ( event.button.button == SDL_BUTTON_LEFT )
        {
            SDL_Keymod modifier = SDL_GetModState();
            
            if ( modifier & KMOD_LCTRL )
            {
                world.ThrowBall( pos );
            }
            else
            {
                world.AddBall( pos );
            }
        }

        if ( event.button.button == SDL_BUTTON_RIGHT )
        {
            world.AddBox( pos );
        }
    }
}

void HVMC_App::Update( f32 dt )
{
    world.Update( dt );
}

void HVMC_App::Render()
{
    SDL_RenderClear( renderer );
    world.Render();
    SDL_RenderPresent( renderer );
}

