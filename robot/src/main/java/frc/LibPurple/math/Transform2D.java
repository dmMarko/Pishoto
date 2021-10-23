/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.math;

/**
 * Add your docs here.
 */
public class Transform2D {

    public final Vector2D translation;
    public final Rotation2D rotation;

    public Transform2D(Vector2D translation, Rotation2D rotation){
        this.translation = translation;
        this.rotation = rotation;
    }

    private static Vector2D intersectionInternal(Transform2D a, Transform2D b) {
        final double t = ((a.translation.x - b.translation.x) * b.rotation.tan + b.translation.y - a.translation.y) /
                (a.rotation.sin - a.rotation.cos * b.rotation.tan);
        return a.translation.add(Vector2D.fromAngle(a.rotation).scale(t));
    }

    public Transform2D transformBy(Transform2D other) {
        return new Transform2D(translation.add(other.translation.rotateBy(rotation)), rotation.rotateBy(other.rotation));
    }

    public Transform2D inverse() {
        Rotation2D inverseRotation = rotation.inverse();
        return new Transform2D(translation.inverse().rotateBy(inverseRotation), inverseRotation);
    }

    public Vector2D intersection(Transform2D other) {
        if (rotation.isParallel(other.rotation)) {
            return new Vector2D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }

        if (Math.abs(rotation.cos) < Math.abs(other.rotation.cos)) {
            return intersectionInternal(this, other);
        } else {
            return intersectionInternal(other, this);
        }
    }

    public Transform2D interpolate(Transform2D other, double t) {
        return new Transform2D(translation.interpolate(other.translation, t),
                rotation.interpolate(other.rotation, t));
    }
}

