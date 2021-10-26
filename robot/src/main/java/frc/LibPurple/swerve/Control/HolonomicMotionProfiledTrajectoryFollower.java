package frc.LibPurple.swerve.Control;

import frc.LibPurple.control.PIDController;
import frc.LibPurple.control.PIDvalue;
import frc.LibPurple.math.HolonomicDriveSignal;
import frc.LibPurple.math.RigidTransform2D;
import frc.LibPurple.math.Vector2D;
import frc.LibPurple.swerve.trajectories.TrajectoryS;



public class HolonomicMotionProfiledTrajectoryFollower extends TrajectoryFollower<HolonomicDriveSignal> {
    private PIDController forwardController;
    private PIDController strafeController;
    private PIDController rotationController;

    private HolonomicFeedForward feedforward;

    private TrajectoryS.Segment lastSegment = null;

    private boolean finished = false;

    public HolonomicMotionProfiledTrajectoryFollower(PIDvalue translationConstants, PIDvalue rotationConstants,
                                                     HolonomicFeedForward feedforward) {
        this.forwardController = new PIDController(translationConstants);
        this.strafeController = new PIDController(translationConstants);
        this.rotationController = new PIDController(rotationConstants);
        this.rotationController.setContinuous(true);
        this.rotationController.setInputRange(0.0, 2.0 * Math.PI);

        this.feedforward = feedforward;
    }


    public TrajectoryS.Segment getLastSegment() {
        return lastSegment;
    }

    @Override
    protected boolean isFinished() {
        return finished;
    }

    @Override
    protected void reset() {
        forwardController.reset();
        strafeController.reset();
        rotationController.reset();

        finished = false;
    }

    @Override
    protected HolonomicDriveSignal calculateDriveSignal(RigidTransform2D currentPose,
            Vector2D velocity, double rotationalVelocity, TrajectoryS trajectory,
            double time, double dt) {
                if (time > trajectory.getDuration()) {
                    finished = true;
                    return new HolonomicDriveSignal(Vector2D.ZERO, 0.0, false);
                }
        
                lastSegment = trajectory.calculateSegment(time);
        
                Vector2D segmentVelocity = Vector2D.fromAngle(lastSegment.heading).scale(lastSegment.velocity);
                Vector2D segmentAcceleration = Vector2D.fromAngle(lastSegment.heading).scale(lastSegment.acceleration);
        
                Vector2D feedforwardVector = feedforward.calculateFeedforward(segmentVelocity, segmentAcceleration);
        
                forwardController.setSetpoint(lastSegment.translation.x);
                strafeController.setSetpoint(lastSegment.translation.y);
                rotationController.setSetpoint(lastSegment.rotation.toRadians());
        
                return new HolonomicDriveSignal(
                        new Vector2D(
                                forwardController.calculate(currentPose.translation.x, dt) + feedforwardVector.x,
                                strafeController.calculate(currentPose.translation.y, dt) + feedforwardVector.y
                        ),
                        rotationController.calculate(currentPose.rotation.toRadians(), dt),
                        true
                );
    }
}