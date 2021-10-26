/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.swerve.trajectories;


import frc.LibPurple.math.RandomMath;

/**
 * Add your docs here.
 */
public class FeedForwardConstrants implements ITrajectoryConstraint{
    private final double targetFeedforward;

    private final double kV;
    private final double kA;
    private final double kS;

    public FeedForwardConstrants(double targetFeedforward, double kV, double kA, double kS) {
        if (targetFeedforward < 0.0) {
            throw new IllegalArgumentException("Target feedforward must be positive");
        }
        if (kV < 0.0) {
            throw new IllegalArgumentException("kV must be positive");
        }
        if (kA < 0.0) {
            throw new IllegalArgumentException("kA must be positive");
        }
        if (kS < 0.0) {
            throw new IllegalArgumentException("kS must be positive");
        }



        this.targetFeedforward = targetFeedforward;
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
    }

    @Override
    public double getMaxVelocity(PathSegment segment) {
        return (targetFeedforward - kS) / kV;
    }

    @Override
    public double getMaxAcceleration(PathSegment segment, double velocity) {
        double accel = (targetFeedforward - kV * velocity - kS) / kA;
        if (RandomMath.epsilonEquals(accel, 0.0)) {
            return 0.0;
        }

        return accel;
    }
}
