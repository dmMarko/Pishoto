/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.math;

import frc.LibPurple.swerve.Utils.Interpolable;

/**
 * Add your docs here.
 */
public class Rotation2D implements Interpolable<Rotation2D> {

    public static final Rotation2D ZERO = new Rotation2D(1, 0, false);

    public final double cos;
    public final double sin;
    public final double tan;    

    public Rotation2D(double x, double y, boolean normalize){
        if(normalize){
            double length = Math.sqrt(x*x + y*y);

            if(length > RandomMath.EPSILON){
                x /= length;
                y /= length;
            } else{
                x = 1;
                y = 0;
            }
        }
        this.cos = x;
        this.sin = y;

        if(RandomMath.epsilonEquals(cos, 0.0)){
            if(sin >= 0){
                this.tan = Double.POSITIVE_INFINITY;
            } else{
                this.tan = Double.NEGATIVE_INFINITY;
            }
        } else{
            this.tan = sin / cos;
        }
    }

    public static Rotation2D fromDegrees(double angle){
        return fromRadians(Math.toRadians(angle));
    }

    public static Rotation2D fromRadians(double angle) {
        return new Rotation2D(Math.cos(angle), Math.sin(angle), false);
    }

    public double toRadians() {
        double angle = Math.atan2(sin, cos);

        if (angle < 0) {
            angle += 2 * Math.PI;
        }

        return angle;
    }

    public double toRadiansWithout2Pi(){
        double angle = Math.atan2(sin, cos);

        return angle;
    }

    public double toDegrees(){
        return Math.toDegrees(toRadians());
    }

    public Rotation2D rotateBy(Rotation2D other){
        return new Rotation2D(cos * other.cos - sin * other.sin,
                cos * other.sin + sin * other.cos, true);
    }

    public Rotation2D normal(){
        return new Rotation2D(-sin, cos, false);
    }

    public Rotation2D inverse(){
        return new Rotation2D(-cos, sin, false);
    }

    public boolean isParallel(Rotation2D other) {
        return RandomMath.epsilonEquals(Vector2D.fromAngle(this).cross(Vector2D.fromAngle(other)), 0.0);
    }

    public Rotation2D interpolate(Rotation2D other, double t) {
        if (t <= 0.0) {
            return this;
        } else if (t >= 1.0) {
            return other;
        }

        double from = toRadians();
        double to = other.toRadians();

        double diff = Math.abs(from - to);
        if (diff > Math.PI) {
            if (from < to) {
                from += 2 * Math.PI;
            } else {
                to += 2 * Math.PI;
            }
        }

        return Rotation2D.fromRadians(from + ((to - from) * t));
    }

    public boolean equals(Rotation2D other, double maxError) {
        return RandomMath.epsilonEquals(cos, other.cos, Math.abs(Math.cos(maxError))) &&
                RandomMath.epsilonEquals(sin, other.sin, Math.abs(Math.sin(maxError)));
    }

}
