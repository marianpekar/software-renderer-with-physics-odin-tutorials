package main

import "core:math"

Ray :: struct {
    hit: bool,
    model: ^Model,
    direction: Vector3
}

CastRay :: proc(screenX, screenY: f32, camera: Camera, projType: ProjectionType, models: []Model) -> Ray {
    ndcX := (screenX / f32(SCREEN_WIDTH)) * 2.0 - 1.0
    ndcY := (screenY / f32(SCREEN_HEIGHT)) * 2.0 - 1.0

    rayOrigin := GetRayOrigin(ndcX, ndcY, camera, projType)
    rayDir := GetRayDirection(ndcX, ndcY, camera, projType)

    ray: Ray
    closestDist := max(f32)

    for &model in models {
        center := model.translation
        delta := center - rayOrigin

        axes := GetAxesFromRotationMatrix(model.rotationMatrix)
        size := model.collider * model.scale

        tMin :=  min(f32)
        tMax :=  max(f32)
        hit := true

        for i in 0..<3 {
            axis := axes[i]
            e := Vector3DotProduct(axis, delta)
            f := Vector3DotProduct(axis, rayDir)

            t1 := (e + size[i]) / f
            t2 := (e - size[i]) / f

            if t1 > t2 {
                t1, t2 = t2, t1 
            }

            tMin = max(tMin, t1)
            tMax = min(tMax, t2)

            if tMin > tMax || tMax < 0 {
                hit = false
                continue
            }
        }

        if hit && tMin < closestDist {
            closestDist = tMin
            ray.hit = true
            ray.model = &model
            ray.direction = rayDir
        }
    }

    return ray

    GetRayOrigin :: proc(ndcX, ndcY: f32, camera: Camera, projType: ProjectionType) -> Vector3 {
        if projType == .Orthographic {
            aspect := f32(SCREEN_WIDTH) / f32(SCREEN_HEIGHT)
            return camera.position + camera.right * (ndcX * aspect) + camera.up * (-ndcY)
        }
        
        return camera.position
    }

    GetRayDirection :: proc(ndcX, ndcY: f32, camera: Camera, projType: ProjectionType) -> Vector3 {
        if projType == .Orthographic {
            return camera.forward
        }

        aspect := f32(SCREEN_WIDTH) / f32(SCREEN_HEIGHT)
        tanHalfFov := math.tan_f32(FOV * 0.5 * DEG_TO_RAD)

        return Vector3Normalize (
            camera.forward +
            camera.right * (ndcX * aspect * tanHalfFov) +
            camera.up * (-ndcY * tanHalfFov)
        )
    }
}