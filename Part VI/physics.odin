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

        ApplyGravity(&model, models, deltaTime)
        IntegrateLinearForce(&model, deltaTime)
    }
}

ApplyGravity :: proc(model: ^Model, models: []Model, deltaTime: f32) {
    rb := &model.rigidBody.(RigidBody)
    rb.velocity += GRAVITY * deltaTime

    for &other in models {
        if &other == model do continue

        model.translation.y -= GROUND_PROBE_DIST
        probe := GetCollisionResult(model, &other)
        model.translation.y += GROUND_PROBE_DIST

        if probe.hit {
            ApplyFriction(model, other)
        }
    }
}

ApplyFriction :: proc(model: ^Model, other: Model) {
    rbo, has_rbo := other.rigidBody.?
    avgFriction := (model.rigidBody.?.friction + rbo.friction) * 0.5 if has_rbo else (model.rigidBody.?.friction + 1.0) * 0.5

    rb := &model.rigidBody.(RigidBody)
    rb.force.x *= avgFriction
    rb.force.z *= avgFriction
}

IntegrateLinearForce :: proc(model: ^Model, deltaTime: f32) {
    rb := &model.rigidBody.(RigidBody)
    rb.velocity += rb.force * deltaTime
    rb.force = {}
    rb.velocity *= LINEAR_DRAG
    model.translation += rb.velocity * deltaTime
}