package main

import "core:math"

Ray :: struct {
    hit: bool,
    model: ^Model,
    direction: Vector3
}

CastRayFromWorldPosition :: proc(origin: Vector3, direction: Vector3, models: []Model, maxLenght: f32 = max(f32), ignore: ^Model = nil) -> Ray {
    ray: Ray
    ray.direction = direction
    closestDist := max(f32)

    for &model in models {
        if &model == ignore do continue
            
        center := model.translation
        delta := center - origin

        axes := GetAxesFromRotationMatrix(model.rotationMatrix)
        size := model.collider * model.scale

        tMin :=  f32(0)
        tMax :=  maxLenght
        hit := true

        for i in 0..<3 {
            axis := axes[i]
            e := Vector3DotProduct(axis, delta)
            f := Vector3DotProduct(axis, ray.direction)
            
            if abs(f) < 1e-6 {
                if e < -size[i] || e > size[i] {
                    hit = false
                    break
                }
                continue
            }

            t1 := (e + size[i]) / f
            t2 := (e - size[i]) / f

            if t1 > t2 {
                t1, t2 = t2, t1 
            }

            tMin = max(tMin, t1)
            tMax = min(tMax, t2)

            if tMin > tMax {
                hit = false
                break
            }
        }

        if hit && tMin < closestDist {
            closestDist = tMin
            ray.hit = true
            ray.direction = direction
            ray.model = &model
        }
    }

    return ray
}

CastRayFromScreenPosition :: proc(screenX, screenY: f32, camera: Camera, projType: ProjectionType, models: []Model) -> Ray {
    ndcX := (screenX / f32(SCREEN_WIDTH)) * 2.0 - 1.0
    ndcY := (screenY / f32(SCREEN_HEIGHT)) * 2.0 - 1.0

    rayOrigin := GetRayOrigin(ndcX, ndcY, camera, projType)
    rayDirection := GetRayDirection(ndcX, ndcY, camera, projType)
    ray := CastRayFromWorldPosition(rayOrigin, rayDirection, models)

    return ray

    GetRayOrigin :: proc(ndcX, ndcY: f32, camera: Camera, projType: ProjectionType) -> Vector3 {
        if projType == .Perspective do return camera.position

        aspect := f32(SCREEN_WIDTH) / f32(SCREEN_HEIGHT)
        return camera.position + camera.right * (ndcX * aspect) + camera.up * (-ndcY)
    }

    GetRayDirection :: proc(ndcX, ndcY: f32, camera: Camera, projType: ProjectionType) -> Vector3 {
        if projType == .Orthographic do return camera.forward

        aspect := f32(SCREEN_WIDTH) / f32(SCREEN_HEIGHT)
        tanHalfFov := math.tan_f32(FOV * 0.5 * DEG_TO_RAD)

        return Vector3Normalize (
            camera.forward +
            camera.right * (ndcX * aspect * tanHalfFov) +
            camera.up * (-ndcY * tanHalfFov)
        )
    }
}