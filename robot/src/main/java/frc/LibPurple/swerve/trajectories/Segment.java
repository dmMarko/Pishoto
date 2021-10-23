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
public class Segment {
    // public final int pathSegmentIndex;
    // public final double time;
    public final Vector2D translation;
    public final Rotation2D heading;
    public final Rotation2D rotation;
    // public final double position, velocity, acceleration;
    public final double velocity, acceleration;
    // public final double maxVelocity;
    // public final double maxAcceleration;

    public Segment(Vector2D translation, Rotation2D heading, Rotation2D rotation, double velocity, double acceleration) {
        // this.pathSegmentIndex = pathSegmentIndex;
        // this.time = time;
        this.translation = translation;
        this.heading = heading;
        this.rotation = rotation;
        // this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
        // this.maxVelocity = maxVelocity;
        // this.maxAcceleration = maxAcceleration;
    }
}
