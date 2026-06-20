package main

RigidBody :: struct {
    force: Vector3,
    velocity: Vector3,
    isStatic: bool
}

ApplyPhysics :: proc(models: []Model, deltaTime: f32) {
    for &model in models {
        if model.rigidBody.isStatic do continue

        ApplyGravity(&model, models, deltaTime)
        IntegrateLinearForce(&model, deltaTime)
    }
}

ApplyGravity :: proc(model: ^Model, models: []Model, deltaTime: f32) {
    model.rigidBody.velocity += GRAVITY * deltaTime
}

IntegrateLinearForce :: proc(model: ^Model, deltaTime: f32) {
    model.rigidBody.velocity += model.rigidBody.force * deltaTime
    model.rigidBody.force = {}
    model.rigidBody.velocity *= LINEAR_DRAG
    model.translation += model.rigidBody.velocity * deltaTime
}