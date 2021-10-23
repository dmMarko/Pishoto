// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.LibPurple.swerve.controlTrajectory;

// import edu.wpi.first.wpilibj.Timer;
// import frc.LibPurple.control.PIDController;
// import frc.LibPurple.control.PIDvalue;
// import frc.LibPurple.math.Matrix;
// import frc.LibPurple.math.Vector2D;
// import frc.LibPurple.swerve.Control.HolonomicFeedForward;
// import frc.LibPurple.swerve.Swerve.SwerveDrivetrain;
// import frc.LibPurple.swerve.trajectories.Segment;
// import frc.LibPurple.utils.Utils;
// import frc.robot.Constants;
// import frc.robot.Robot;

// /**
//  * Add your docs here.
//  */
// public class PurePursuitController extends TrajectoryController {

//     private PIDController rotationController;

//     private HolonomicFeedForward feedforward;

//     public PurePursuitController(SwerveDrivetrain drivetrain, PIDvalue rotationValue, HolonomicFeedForward feedForward){
//         super(drivetrain);

//         this.rotationController = new PIDController(rotationValue);

//         this.feedforward = feedForward;
//     }

//     @Override
//     public void calculate(double passedTime, double dt) {
//         now = Timer.getFPGATimestamp();
//         dt = now - lastTime;
//         passedTime = now - startTime;
//         lastTime = now;
//         setpointSegment = trajectory.calculate(passedTime);

//         Vector2D segmentVelocity = Vector2D.fromAngle(setpointSegment.heading).scale(setpointSegment.velocity);
//         Vector2D segmentAcceleration = Vector2D.fromAngle(setpointSegment.heading).scale(setpointSegment.acceleration);

//         Vector2D feedforwardVector = feedforward.calculateFeedforward(segmentVelocity, segmentAcceleration);

//         Vector2D currentPos = drivetrain.getKinematicPosition();
//         Segment goalSetpoint = trajectory.calculate(passedTime + Constants.lookheadPurePursuit);

//         double xgoal = goalSetpoint.translation.y - currentPos.x;
//         double ygoal = goalSetpoint.translation.x - currentPos.y;

//         double l = Math.sqrt(xgoal * xgoal + ygoal * ygoal);

//         double angle = Math.asin(xgoal / l) / 2;

//         Vector2D translation = new Vector2D(l * Math.cos(angle * 2) + feedforwardVector.x, -l * Math.sin(angle * 2) + feedforwardVector.y);

//         rotationController.setSetpoint(setpointSegment.rotation.toRadians());
//         double absError = setpointSegment.rotation.toRadians() - Math.toRadians(drivetrain.navX.getYaw());
//         double error = 0;
//         if(absError < Math.PI){
//             error = absError;
//         }
//         else{
//             error = absError - 2 * Math.PI;
//         }

//         drivetrain.holonomicDrive(translation, rotationController.calculate(setpointSegment.rotation.toRadians() - error, dt), true);
//         drivetrain.updateKinematics(passedTime);
//     }

//     @Override
//     public double getSetpointVelocity() {
//         return this.velocity;
//     }
// }
