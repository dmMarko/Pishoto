// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.LibPurple.swerve.controlTrajectory;

// /**
//  * Add your docs here.
//  */
// public class OldMotionProfile extends TrajectoryController {
//     private PIDController forwardController;
//     private PIDController strafeController;
//     // private PIDController rotationController;
//     private RotationPIDController rotationController;

//     private HolonomicFeedForward feedforward;

//     public MotionProfile(SwerveDrivetrain drivetrain, PIDvalue forwardValue, PIDvalue starfeValue, PIDvalue rotationValue, HolonomicFeedForward feedForward) {
//         super(drivetrain);
//         this.forwardController = new PIDController(forwardValue);
//         this.strafeController = new PIDController(starfeValue);
//         this.rotationController = new RotationPIDController(rotationValue);

//         this.feedforward = feedForward;
//     }



//     public void calculate(double passedTime, double dt) {
//     now = Timer.getFPGATimestamp();
//     dt = now - lastTime;
// 		passedTime = now - startTime;
//     lastTime = now;
//     setpointSegment = trajectory.calculate(passedTime);

//     Vector2D segmentVelocity = Vector2D.fromAngle(setpointSegment.heading).scale(setpointSegment.velocity);
//     Vector2D segmentAcceleration = Vector2D.fromAngle(setpointSegment.heading).scale(setpointSegment.acceleration);

//     Vector2D feedforwardVector = feedforward.calculateFeedforward(segmentVelocity, segmentAcceleration);

//     forwardController.setSetpoint(setpointSegment.translation.x);
//     strafeController.setSetpoint(setpointSegment.translation.y);
//     // rotationController.setSetpoint(setpointSegment.rotation.toRadiansWithout2Pi());
//     rotationController.setSetpoint(setpointSegment.rotation.toRadians());
//     // rotationController.setSetpoint(0);

//     double power = forwardController.calculate(Robot.swerve.getKinematicPosition().x, dt) + feedforwardVector.x;
//     double angle = Math.toRadians(Robot.swerve.navX.getYaw());
//     // double angle = (Math.toRadians(Robot.swerve.navX.getYaw()) + 2 * Math.PI) % (2 * Math.PI);
//     // if (rotationController.getSetpoint() > Math.PI && Utils.inRange(angle, 0, Math.PI / 180)){
//     //     angle = 2 * Math.PI;
//     // }
//     double rot = rotationController.calculate(angle, dt);
//     SmartDashboard.putNumber("rot angle", angle);
//     SmartDashboard.putNumber("rot pid", rot);
//     // double angle = Math.toRadians(Robot.swerve.navX.getYaw());
//     Robot.swerve.holonomicDrive(
//         new Vector2D(power,
//             strafeController.calculate(Robot.swerve.getKinematicPosition().y, dt) + feedforwardVector.y
//             ),
//             rot,
//             true
//     );
//     Robot.swerve.updateKinematics(passedTime);
//     }

//     @Override
//     public double getSetpointVelocity() {
//         return 0;
//     }

//     public void resetControllers(){
//         forwardController.reset();
//         strafeController.reset();
//         rotationController.reset();
//     }

//     @Override
//     public void setTrajectory(Trajectory trajectory) {
//         super.setTrajectory(trajectory);
//         resetControllers();
//     }

//     @Override
//     public void disable(){
//         super.disable();
//         resetControllers();
//     }
// }
