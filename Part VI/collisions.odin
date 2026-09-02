
package main

BoxCollider :: Vector3

CollisionResult :: struct {
    hit: bool,
    normal: Vector3,
    depth: f32
}

ResolveCollisions :: proc(models: []Model) {
    for i in 0..<len(models) {
        for j in i + 1..<len(models) {
            a := &models[i]
            b := &models[j]
            
            rba, has_rba := a.rigidBody.?
            rbb, has_rbb := b.rigidBody.?
            if (!has_rba || rba.isStatic) && (!has_rbb || rbb.isStatic) || 
               (a.collider.x * a.collider.y * a.collider.z < 1e-6 || b.collider.x * b.collider.y * b.collider.z < 1e-6) {
                    continue
                }

            result := GetCollisionResult(a, b)

            if result.hit {
                Correct(a, b, result)
                Push(a, result.normal)
            }
        }
    }

    Correct :: proc(a, b: ^Model, result: CollisionResult) {
        correction := result.normal * max(result.depth, 0.0)

        rba, has_rba := a.rigidBody.?
        rbb, has_rbb := b.rigidBody.?

        if (has_rba && !a.rigidBody.?.isStatic) && (has_rbb && !b.rigidBody.?.isStatic) {
            a.translation -= correction * 0.5
            b.translation += correction * 0.5
        } else if !has_rba || !a.rigidBody.?.isStatic {
            a.translation -= correction
        } else if !has_rbb || !b.rigidBody.?.isStatic {
            b.translation += correction
        }
    }

    Push :: proc(model: ^Model, normal: Vector3) {
        rb, has_rb := &model.rigidBody.(RigidBody)
        if !has_rb || rb.isStatic do return
        
        rb.velocity -= Vector3DotProduct(rb.velocity, normal) * normal * rb.bounciness
    }
}

GetCollisionResult :: proc(a, b: ^Model) -> CollisionResult {
    axesA := GetAxesFromRotationMatrix(a.rotationMatrix)
    axesB := GetAxesFromRotationMatrix(b.rotationMatrix)

    axes: [15]Vector3
    axes[0] = axesA[0]
    axes[1] = axesA[1]
    axes[2] = axesA[2]
    axes[3] = axesB[0]
    axes[4] = axesB[1]
    axes[5] = axesB[2]

    idx := 6
    for i in 0..<3 {
        for j in 0..<3 {
            axes[idx] = Vector3CrossProduct(axesA[i], axesB[j])
            idx += 1
        }
    }

    direction := b.translation - a.translation
    
    minDepth: f32 = max(f32)
    minNormal: Vector3

    for axis in axes {
        if Vector3Length(axis) < 1e-6 do continue

        normalizedAxis := Vector3Normalize(axis)

        radiusA := ProjectRadius(a.collider * a.scale, axesA, normalizedAxis)
        radiusB := ProjectRadius(b.collider * b.scale, axesB, normalizedAxis)

        projection := abs(Vector3DotProduct(direction, normalizedAxis))
        overlap    := radiusA + radiusB - projection

        if overlap <= 0 do return CollisionResult{hit = false}

        if overlap < minDepth {
            minDepth  = overlap
            minNormal = normalizedAxis
        }
    }

    return CollisionResult{
        hit = true, 
        normal = -minNormal if Vector3DotProduct(direction, minNormal) < 0 else minNormal, 
        depth = minDepth
    }

    ProjectRadius :: proc(collider: BoxCollider, axes: [3]Vector3, axis: Vector3) -> f32 {
    return collider.x * abs(Vector3DotProduct(axes[0], axis)) +
           collider.y * abs(Vector3DotProduct(axes[1], axis)) +
           collider.z * abs(Vector3DotProduct(axes[2], axis))
    }
}