package main

import "core:strings"
import "core:fmt"
import rl "vendor:raylib"

Vector2 :: [2]f32

Particle :: struct {
    force: Vector2,
    velocity: Vector2,
    position: Vector2,
    mass: f32
}

UP :: Vector2{0,-1}
DOWN :: Vector2{0,1}
LEFT :: Vector2{-1,0}
RIGHT :: Vector2{1,0}
STRENGTH :: 100
DAMPING :: 0.99

HandleInputs :: proc(particle: ^Particle) {
    if rl.IsKeyDown(rl.KeyboardKey.W) do particle.force += UP * STRENGTH
    if rl.IsKeyDown(rl.KeyboardKey.S) do particle.force += DOWN * STRENGTH
    if rl.IsKeyDown(rl.KeyboardKey.A) do particle.force += LEFT * STRENGTH
    if rl.IsKeyDown(rl.KeyboardKey.D) do particle.force += RIGHT * STRENGTH
}

Integrate :: proc(particle: ^Particle, deltaTime: f32) {
    particle.velocity += particle.force / particle.mass * deltaTime
    particle.position += particle.velocity * deltaTime
}

Draw :: proc(particle: Particle) {
    rl.DrawCircleV(particle.position, 5, rl.WHITE)
    
    text := fmt.tprintf("force=(%.2f, %.2f)\nvelocity=(%.2f, %.2f)\nposition=(%.2f, %.2f)\nmass=%.2f",
        particle.force.x, particle.force.y,
        particle.velocity.x, particle.velocity.y, 
        particle.position.x, particle.position.y,
        particle.mass)
    rl.DrawText(
        strings.clone_to_cstring(text),
        i32(particle.position.x) + 5,
        i32(particle.position.y) + 5,
        10,
        rl.WHITE,
    )
}

main :: proc() {
    rl.InitWindow(800, 600, "Example")
    rl.SetTargetFPS(60)

    particle := Particle{
        position = {400, 300},
        mass = 2
    }

    for !rl.WindowShouldClose() {
        deltaTime := rl.GetFrameTime()

        particle.force = {}

        HandleInputs(&particle)
        Integrate(&particle, deltaTime)

        particle.velocity *= DAMPING

        rl.BeginDrawing()
        rl.ClearBackground(rl.BLACK)
        Draw(particle)
        rl.EndDrawing()
    }

    rl.CloseWindow()
}