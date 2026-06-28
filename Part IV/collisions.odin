
package main

BoxCollider :: struct {
    size: Vector3
}

CollisionResult :: struct {
    hit: bool,
    normal: Vector3,
    depth: f32,
    contactPoint: Vector3
}

ResolveCollisions :: proc(models: []Model) {
    for i in 0..<len(models) {
        for j in i + 1..<len(models) {
            a := &models[i]
            b := &models[j]

            if a.rigidBody.isStatic && b.rigidBody.isStatic do continue

            result := GetCollisionResult(a, b)

            if result.hit {
                Correct(a, b, result)
                Push(a, result.normal)
            }
        }
    }

    Correct :: proc(a, b: ^Model, result: CollisionResult) {
        correction := result.normal * max(result.depth, 0.0)

        if !a.rigidBody.isStatic && !b.rigidBody.isStatic {
            a.translation -= correction * 0.5
            b.translation += correction * 0.5
        } else if !a.rigidBody.isStatic {
            a.translation -= correction
        } else if !b.rigidBody.isStatic {
            b.translation += correction
        }
    }

    Push :: proc(model: ^Model, normal: Vector3) {
        if model.rigidBody.isStatic do return
        
        model.rigidBody.velocity -= Vector3DotProduct(model.rigidBody.velocity, normal) * normal
    }
}

GetCollisionResult :: proc(a, b: ^Model) -> CollisionResult {
    axesA := GetAxesFromRotationMatrix(a.rotationMatrix)
    axesB := GetAxesFromRotationMatrix(b.rotationMatrix)

    colSizeA := a.collider.size * a.scale
    colSizeB := b.collider.size * b.scale

    centerDiff := b.translation - a.translation

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

    minDepth: f32 = max(f32)
    minNormal: Vector3

    for axis in axes {
        if Vector3Length(axis) < 1e-6 do continue

        normalizedAxis := Vector3Normalize(axis)

        radiusA := ProjectRadius(colSizeA, axesA, normalizedAxis)
        radiusB := ProjectRadius(colSizeB, axesB, normalizedAxis)

        centerProj := abs(Vector3DotProduct(centerDiff, normalizedAxis))
        overlap    := radiusA + radiusB - centerProj

        if overlap <= 0 do return CollisionResult{hit = false}

        if overlap < minDepth {
            minDepth  = overlap
            minNormal = normalizedAxis
        }
    }

    if Vector3DotProduct(centerDiff, minNormal) < 0 {
        minNormal = -minNormal
    }

    return CollisionResult{
        hit = true, 
        normal = minNormal, 
        depth = minDepth
    }

    ProjectRadius :: proc(colSize: Vector3, axes: [3]Vector3, axis: Vector3) -> f32 {
    return colSize.x * abs(Vector3DotProduct(axes[0], axis)) +
           colSize.y * abs(Vector3DotProduct(axes[1], axis)) +
           colSize.z * abs(Vector3DotProduct(axes[2], axis))
    }
}