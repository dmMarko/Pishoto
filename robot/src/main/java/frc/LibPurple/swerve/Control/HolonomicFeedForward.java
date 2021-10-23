/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.swerve.Control;

import frc.LibPurple.math.Vector2D;

/**
 * Add your docs here.
 */
public class HolonomicFeedForward {
    private final DrivetrainFeedForwardConstants forwardConstants;
    private final DrivetrainFeedForwardConstants strafeConstants;

    public HolonomicFeedForward(DrivetrainFeedForwardConstants forwardConstants,
                                DrivetrainFeedForwardConstants strafeConstants) {
        this.forwardConstants = forwardConstants;
        this.strafeConstants = strafeConstants;
    }

    public HolonomicFeedForward(DrivetrainFeedForwardConstants translationConstants) {
        this(translationConstants, translationConstants);
    }

    public Vector2D calculateFeedforward(Vector2D velocity, Vector2D acceleration) {
        // We don't use `DrivetrainFeedforwardConstants.calculateFeedforward` because we want to apply kS (the static
        // constant) proportionally based on the rest of the feedforwards.

        double forwardFeedforward = forwardConstants.getVelocityConstant() * velocity.x;
        forwardFeedforward += forwardConstants.getAccelerationConstant() * acceleration.x;

        double strafeFeedforward = strafeConstants.getVelocityConstant() * velocity.y;
        strafeFeedforward += strafeConstants.getAccelerationConstant() * acceleration.y;

        Vector2D feedforwardVector = new Vector2D(forwardFeedforward, strafeFeedforward);

        // Apply the kS constant proportionally to the forward and strafe feedforwards based on their relative
        // magnitudes
        if(feedforwardVector.x == 0 && feedforwardVector.y == 0)
        {
            return new Vector2D(0, 0);
        }
        // Vector2D feedforwardUnitVector = feedforwardVector.normal();
        // forwardFeedforward += Math.copySign(feedforwardUnitVector.x * forwardConstants.getStaticConstant(),
        //         forwardFeedforward);
        // strafeFeedforward += Math.copySign(feedforwardUnitVector.y * strafeConstants.getStaticConstant(),
        //         strafeFeedforward);

        return new Vector2D(forwardFeedforward, strafeFeedforward);
    }
}
