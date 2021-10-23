/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.swerve.trajectories;

/**
 * Add your docs here.
 */
public interface ITrajectoryConstraint {
    /**
     * Get the maximum velocity this constraint allows for this segment.
     *
     * @param segment the segment
     * @return the maximum velocity
     */
    double getMaxVelocity(PathSegment segment);

    /**
     * Get the maximum acceleration this constraint allows for this segment at a velocity.
     *
     * @param segment  the segment
     * @param velocity the velocity
     * @return the maximum acceleration
     */
    double getMaxAcceleration(PathSegment segment, double velocity);
}