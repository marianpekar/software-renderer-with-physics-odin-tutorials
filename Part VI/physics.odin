package main

RigidBody :: struct {
    force: Vector3,
    velocity: Vector3,
    isStatic: bool,
    bounciness: f32,
    friction: f32,
    mass: f32,
    massInverse: f32
}

AddForceAtPoint :: proc(model: ^Model, force: Vector3) {
    rb, has_rb := &model.rigidBody.(RigidBody)
    if !has_rb do return

    rb.force += force
}

ApplyPhysics :: proc(models: []Model, deltaTime: f32) {
    for &model in models {
        rb, has_rb := model.rigidBody.?
        if !has_rb || rb.isStatic do continue

        ApplyGravity(&model, deltaTime)
        IntegrateLinearForce(&model, deltaTime)
    }
}

ApplyGravity :: proc(model: ^Model, deltaTime: f32) {
    rb := &model.rigidBody.(RigidBody)
    rb.velocity += GRAVITY * deltaTime
}

IntegrateLinearForce :: proc(model: ^Model, deltaTime: f32) {
    rb := &model.rigidBody.(RigidBody)
    rb.velocity += rb.force * deltaTime
    rb.force = {}
    rb.velocity *= LINEAR_DRAG
    model.translation += rb.velocity * deltaTime
}