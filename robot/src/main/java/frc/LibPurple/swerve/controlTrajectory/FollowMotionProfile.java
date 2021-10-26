// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.LibPurple.swerve.controlTrajectory;

// import java.io.FileWriter;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.LibPurple.control.PIDController;
// import frc.LibPurple.swerve.Control.HolonomicFeedForward;
// import frc.LibPurple.swerve.Swerve.SwerveDrivetrain;
// import frc.LibPurple.swerve.trajectories.Segment;
// import frc.LibPurple.swerve.trajectories.Trajectory;
// import frc.LibPurple.utils.Utils;
// import frc.robot.Constants;
// import frc.LibPurple.math.Vector2D;

// public class FollowMotionProfile extends CommandBase {
//   private SwerveDrivetrain drivetrain;

//   private final PIDController forwardController = new PIDController(Constants.forwardValueMotionProfile);
//   private final PIDController strafeController = new PIDController(Constants.strafeValueMotionProfile);
//   private final PIDController rotationController = new PIDController(Constants.rotationValueMotionProfile);

//   private final HolonomicFeedForward feedforward = new HolonomicFeedForward(Constants.forwardFeedForwardHolonomicConstants);
//   private double startTime;
//   private double lastTime;
//   private double now;
//   private double passedTime;

//   private final Trajectory trajectory;

//   private final boolean debug;
//   private FileWriter handle;
  
//   public FollowMotionProfile(SwerveDrivetrain drivetrain, final Trajectory trajectory, final boolean debug) {
//     // Use requires() here to declare subsystem dependencies
//     // eg. requires(chassis);
//     addRequirements(drivetrain);
//     this.drivetrain = drivetrain;
//     this.trajectory = trajectory;
//     passedTime = 0;
//     now = 0;
//     lastTime = 0;
//     this.debug = debug;
//     if(this.debug){
//       this.handle = Utils.initializeCSVFile("/graphs/SBeizer");
//       Utils.addCSVLine(handle, new String[] {"Time", "Tarjectory X", "Real X", "Tarjectory Y", "Real Y", "Tarjectory Rotation", "Real Rotation"});
//     }
//   }

//   // Called just before this Command runs the first time
//   @Override
//   public void initialize() {
//     Utils.print("start");
//     startTime = Timer.getFPGATimestamp();
//     lastTime = startTime;
//     withTimeout(trajectory.getTotalTime());
//   }

//   // Called repeatedly when this Command is scheduled to run
//   @Override
//   public void execute() {
//     now = Timer.getFPGATimestamp();
//     // dt = now - startTime - passedTime;
//     final double dt = now - lastTime;
//     passedTime = now - startTime;
//     lastTime = now;
//     // if(passedTime >= trajectory.getTotalTime()){
//     // Utils.print("avi");
//     // Robot.swerve.holonomicDrive(new Vector2D(0, 0), 0, true);
//     // }
//     final Segment setpoint = trajectory.calculate(passedTime);

//     final Vector2D segmentVelocity = Vector2D.fromAngle(setpoint.heading).scale(setpoint.velocity);
//     final Vector2D segmentAcceleration = Vector2D.fromAngle(setpoint.heading).scale(setpoint.acceleration);

//     final Vector2D feedforwardVector = feedforward.calculateFeedforward(segmentVelocity, segmentAcceleration);

//     forwardController.setSetpoint(setpoint.translation.x);
//     strafeController.setSetpoint(setpoint.translation.y);
//     rotationController.setSetpoint(setpoint.rotation.toRadians());

//     // return new HolonomicDriveSignal(
//     // new Vector2(
//     // forwardController.calculate(currentPose.translation.x, dt) +
//     // feedforwardVector.x,
//     // strafeController.calculate(currentPose.translation.y, dt) +
//     // feedforwardVector.y
//     // ),
//     // rotationController.calculate(currentPose.rotation.toRadians(), dt),
//     // true
//     // );
//     final double power = forwardController.calculate(drivetrain.getKinematicPosition().x, dt) + feedforwardVector.x;
//     if (debug) {
//       final double[] params = { passedTime, setpoint.translation.x, drivetrain.getKinematicPosition().x,
//           setpoint.translation.y, drivetrain.getKinematicPosition().y, setpoint.heading.toDegrees(),
//           SwerveDrivetrain.navX.getYaw() };
//       Utils.addCSVLine(handle, params);
//     }
//     drivetrain.holonomicDrive(
//         new Vector2D(power,
//             strafeController.calculate(drivetrain.getKinematicPosition().y, dt) + feedforwardVector.y),
//         rotationController.calculate(Math.toRadians(SwerveDrivetrain.navX.getYaw()), dt), true);
//   }

//   // Make this return true when this Command no longer needs to run execute()
//   @Override
//   public boolean isFinished() {
//     // return isTimedOut();
//     return false;
//   }

//   // Called once after isFinished returns true
//   @Override
//   public void end(boolean interrupted) {
//   }
// }
