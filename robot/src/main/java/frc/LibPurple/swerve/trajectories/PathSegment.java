/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.swerve.trajectories;

import frc.LibPurple.math.Rotation2D;
import frc.LibPurple.math.Vector2D;

/**
 * Add your docs here.
 */
public abstract class PathSegment {
    private final Vector2D start;
    private final Vector2D end;

    public PathSegment(Vector2D start, Vector2D end) {
        this.start = start;
        this.end = end;
    }

    public abstract PathSegment[] subdivide();

    public abstract PathSegment mirror();

    /**
     * Gets the curvature of the segment.
     *
     * @return the curvature of the segment
     */
    public abstract double getCurvature();

    public frc.LibPurple.math.Vector2D getStart() {
        return start;
    }

    public Vector2D getEnd() {
        return end;
    }

    public Vector2D getPositionAtDistance(double distance) {
        return getPositionAtPercentage(distance / getLength());
    }

    public abstract frc.LibPurple.math.Vector2D getPositionAtPercentage(double percentage);

    public Rotation2D getHeadingAtDistance(double distance) {
        return getHeadingAtPercentage(distance / getLength());
    }

    public abstract frc.LibPurple.math.Rotation2D getHeadingAtPercentage(double percentage);

    public abstract double getLength();
}
