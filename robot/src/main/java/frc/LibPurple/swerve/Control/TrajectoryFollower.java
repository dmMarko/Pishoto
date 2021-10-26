/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.LibPurple.swerve.Control;

import java.util.Optional;

import frc.LibPurple.math.RigidTransform2D;
import frc.LibPurple.math.Vector2D;
import frc.LibPurple.swerve.trajectories.TrajectoryS;



/**
 * Add your docs here.
 */
public abstract class TrajectoryFollower<DriveSignalType> {
    /**
     * The trajectory that is currently being followed. Null if no trajectory is
     * being followed.
     * <p>
     * Protected by {@link #trajectoryLock}
     */
    private TrajectoryS currentTrajectory = null;

    /**
     * The time that the current trajectory started to be followed. NaN if the
     * trajectory has not been started yet.
     * <p>
     * Protected by {@link #trajectoryLock}
     */
    private double startTime = Double.NaN;

    /**
     * Calculates the drive signal required to follow the trajectory.
     *
     * @param currentPose        the current pose of the robot
     * @param velocity           the translational velocity of the robot
     * @param rotationalVelocity the rotational velocity of the robot
     * @param trajectory         the trajectory to follow
     * @param time               the amount of time that has elapsed since the
     *                           current trajectory started to be followed
     * @param dt                 the amount of time that has elapsed since the
     *                           update loop was last ran
     * @return the signal required to follow the trajectory
     */
    protected abstract DriveSignalType calculateDriveSignal(RigidTransform2D currentPose, Vector2D velocity,
                                                            double rotationalVelocity, TrajectoryS trajectory,
                                                            double time, double dt);

    /**
     * Gets if the follower is done following the path.
     *
     * @return true if the path is done
     */
    protected abstract boolean isFinished();

    /**
     * Resets the follower's internal state. This is called when a new trajectory is started.
     */
    protected abstract void reset();

    /**
     * Cancels the currently running trajectory.
     */
    public synchronized final void cancel() {
            currentTrajectory = null;
    }

    public synchronized final void follow(TrajectoryS trajectory) {
            currentTrajectory = trajectory;
            startTime = Double.NaN;
    }

    /**
     * Gets the current trajectory that is being followed if any.
     *
     * @return the current trajectory being followed
     */
    public  synchronized final Optional<TrajectoryS> getCurrentTrajectory() {
            return Optional.ofNullable(currentTrajectory);
    }

    /**
     * Updates the follower and returns the calculated drive signal that should be applied to the robot in order to
     * follow the current trajectory.
     *
     * @param currentPose        the current pose of the robot
     * @param velocity           the translational velocity of the robot
     * @param rotationalVelocity the rotational velocity of the robot
     * @param time               the current time
     * @param dt                 the time since update was last called
     * @return the drive signal required to follow the current path if any
     */
    public synchronized final Optional<DriveSignalType> update(RigidTransform2D currentPose, Vector2D velocity,
                                                  double rotationalVelocity, double time, double dt) {
        TrajectoryS trajectory;
        double timeSinceStart;

            // Return empty if no trajectory is being followed
            if (currentTrajectory == null) {
                return Optional.empty();
            }

            // If the trajectory has not been started, update the start time and reset the follower state
            if (Double.isNaN(startTime)) {
                startTime = time;
                reset();
            } else if (isFinished()) {
                currentTrajectory = null;
                return Optional.empty();
            }

            trajectory = currentTrajectory;
            timeSinceStart = time - startTime;

        DriveSignalType signal = calculateDriveSignal(currentPose, velocity, rotationalVelocity, trajectory,
                timeSinceStart, dt);

        return Optional.of(signal);
    }
}
