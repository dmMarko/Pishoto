// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.LibPurple.swerve.controlTrajectory;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.LibPurple.control.PIDController;
// import frc.LibPurple.control.PIDvalue;
// import frc.LibPurple.control.RotationPIDController;
// import frc.LibPurple.math.Vector2D;
// import frc.LibPurple.swerve.Control.HolonomicFeedForward;
// import frc.LibPurple.swerve.Swerve.SwerveDrivetrain;
// import frc.LibPurple.swerve.Utils.Utils;
// import frc.LibPurple.swerve.trajectories.Trajectory;
// import frc.robot.RobotContainer;


// /**
//  * Add your docs here.
//  */
// public class MotionProfile extends TrajectoryController {
//     private PIDController forwardController;
//     private PIDController strafeController;
//     private PIDController rotationController;
//     // private RotationPIDController rotationController;

//     private HolonomicFeedForward feedforward;

//     public MotionProfile(SwerveDrivetrain drivetrain, PIDvalue forwardValue, PIDvalue starfeValue, PIDvalue rotationValue, HolonomicFeedForward feedForward) {
//         super(drivetrain);
//         this.forwardController = new PIDController(forwardValue);
//         this.strafeController = new PIDController(starfeValue);
//         this.rotationController = new PIDController(rotationValue);

//         this.feedforward = feedForward;
//     }



//     public void calculate(double passedTime, double dt) {
//         now = Timer.getFPGATimestamp();
//         dt = now - lastTime;
//         passedTime = now - startTime;
//         lastTime = now;
//         setpointSegment = trajectory.calculate(passedTime);

//         Vector2D segmentVelocity = Vector2D.fromAngle(setpointSegment.heading).scale(setpointSegment.velocity);
//         Vector2D segmentAcceleration = Vector2D.fromAngle(setpointSegment.heading).scale(setpointSegment.acceleration);

//         Vector2D feedforwardVector = feedforward.calculateFeedforward(segmentVelocity, segmentAcceleration);

//         forwardController.setSetpoint(setpointSegment.translation.x);
//         strafeController.setSetpoint(setpointSegment.translation.y);
//         // strafeController.setSetpoint(0);
//         // rotationController.setSetpoint(setpointSegment.rotation.toRadiansWithout2Pi());
//         rotationController.setSetpoint(setpointSegment.rotation.toRadians());
//         // rotationController.setSetpoint(0);

//         double absError = setpointSegment.rotation.toRadians() - Math.toRadians(drivetrain.navX.getYaw());
//         double error = 0;
//         if(absError < Math.PI){
//             error = absError;
//         }
//         else{
//             error = absError - 2 * Math.PI;
//         }
//         // SmartDashboard.putNumber("error", error);
//         // double power = rotationController.calculate(Math.toRadians(drivetrain.navX.getYaw()), dt);
//         double power = rotationController.calculate(setpointSegment.rotation.toRadians() - error, dt);
//         SmartDashboard.putNumber("rot power", power);

//         drivetrain.holonomicDrive(
//             new Vector2D(forwardController.calculate(drivetrain.getKinematicPosition().x, dt) + feedforwardVector.x,
//             // new Vector2D(feedforwardVector.x,
//                 strafeController.calculate(drivetrain.getKinematicPosition().y, dt) + feedforwardVector.y
//                 ),
//                 // rotationController.calculate(setpointSegment.rotation.toRadians() - error, dt),
//                 power,
//                 // 0,
//                 true
//         );
//         Utils.print("update motion");
//         drivetrain.updateKinematics(passedTime);
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
//         drivetrain.resetPID();
//     }
// }
