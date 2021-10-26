// package frc.LibPurple.swerve.SDriveCommands;

// import frc.LibPurple.swerve.Swerve.SwerveDrivetrain;
// import frc.LibPurple.swerve.Swerve.SwerveDrivetrain.ControllerType;
// import frc.LibPurple.swerve.trajectories.StraightTrajectorySMP;
// import frc.robot.Constants;

// public class SDriveStraight extends SDriveCommand {

//   double distance;
//   double angle;
//   double rotation;
//   boolean endless;

//   double maxV;
//   double maxA;

//   public SDriveStraight(SwerveDrivetrain driveSystem, double distance, double angle, double rotation, ControllerType controllerType, boolean debug){
//     super(driveSystem, controllerType, debug);
//     this.distance = distance;
//     this.rotation = rotation;
//     this.angle = angle;
//     this.maxV = Constants.maxXV;
//     this.maxA = Constants.maxXA;
//     filenameDebug = "SDriveSraight";
//   }

//   public SDriveStraight(SwerveDrivetrain driveSystem, double distance, double angle, double rotation, ControllerType controllerType, boolean endless, boolean debug) {
//     this(driveSystem, distance, angle, rotation, controllerType, debug);
//     this.endless = endless;
//   }
    
//   public SDriveStraight(SwerveDrivetrain driveSystem, double distance, double angle, double rotation, double maxA, double maxV, ControllerType controllerType, boolean endless, boolean debug) {
//     this(driveSystem, distance, angle, rotation, controllerType, endless, debug);
//     this.maxV = maxV;
//     this.maxA = maxA;
//   }

//   @Override
//   public void setTrajectory() {
//     // TODO Auto-generated method stub
//     controller.setTrajectory(new StraightTrajectorySMP(distance, angle, rotation, maxA));
//     controller.setDirection(distance < 0);
//   }
// }
// // /*----------------------------------------------------------------------------*/
// // /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// // /* Open Source Software - may be modified and shared by FRC teams. The code   */
// // /* must be accompanied by the FIRST BSD license file in the root directory of */
// // /* the project.                                                               */
// // /*----------------------------------------------------------------------------*/

// // package frc.LibPurple.swerve.SDriveCommands;

// // import java.io.FileWriter;
// // import edu.wpi.first.wpilibj2.command.CommandBase;
// // import frc.LibPurple.swerve.Swerve.SwerveDrivetrain;
// // import frc.LibPurple.swerve.Swerve.SwerveDrivetrain.ControllerType;
// // import frc.LibPurple.swerve.Utils.Utils;
// // import frc.LibPurple.swerve.controlTrajectory.TrajectoryController;
// // import frc.LibPurple.swerve.trajectories.StraightTrajectorySMP;
// // import frc.robot.Constants;


// // public class SDriveStraight extends CommandBase {
// //   private double distance;
// //   private double angle;
// //   private boolean debug;

// //   SwerveDrivetrain driveSystem;
// //   TrajectoryController controller;
// //   ControllerType controllerType;

// //   boolean endless;

// // 	double maxA;
// // 	double maxV;

// //   FileWriter handle;
// //   FileWriter debugHandle;
  
// //   double rotation;

// // // 	double tolerance;

// //   public SDriveStraight(SwerveDrivetrain drivetrain, double distance, double angle, double rotation, boolean debug) {
// //     // Use requires() here to declare subsystem dependencies
// //     // eg. requires(chassis);
// //     addRequirements(drivetrain);
// //     this.driveSystem = drivetrain;
// //     this.distance = distance;
// //     this.rotation = rotation;
// //     this.angle = angle;
// //     this.debug = debug;
// //     this.maxV = Constants.maxXV;
// //     this.maxA = Constants.maxXA;
// //   }

// //   public SDriveStraight(SwerveDrivetrain drivetrain, double distance, double angle, double rotation, ControllerType controllerType, boolean endless, boolean debug) {
// //     this(drivetrain, distance, angle, rotation, debug);
// //     this.endless = endless;
// //     this.controllerType = controllerType;
// //   }

// //   public SDriveStraight(SwerveDrivetrain driveSystem, double distance, double angle, double rotation, double maxA, double maxV, ControllerType controllerType, boolean endless, boolean debug) {
// //     this(driveSystem, distance, angle, rotation, controllerType, endless, debug);
// // 		this.maxV = maxV;
// // 		this.maxA = maxA;
// //   }

// //   // Called just before this Command runs the first time
// //   @Override
// //   public void initialize() {
// //     Utils.print("start");
// //     Constants.autoActive = true;
// //     driveSystem.resetEncoders();
// //     if (debug)
// // 		{
// //       Utils.print("halo");
// // 			this.handle = Utils.initializeCSVFile("/graphs/SDriveSraight");
// //       Utils.addCSVLine(this.handle, new String[] {"Time", "Trajectory X", "Real X", "Trajectory Y", "Real Y" });
// //       this.debugHandle = Utils.initializeDebugFile("/graphs/");
// //       Utils.addCSVLine(this.debugHandle, new String[] {"Time", "Trajectory X", "Real X", "Trajectory Y", "Real Y" });
// // 		}
// //     driveSystem.setController(controllerType);
// //     this.controller = driveSystem.getController();
// //     this.controller.setTrajectory(new StraightTrajectorySMP(distance, angle, rotation, maxA));
// //     controller.setDirection(distance < 0);
// //     this.controller.enable();
// //     Utils.print("" + controller.getTrajectory().getTotalTime());
// //     // withTimeout(controller.getTrajectory().getTotalTime());
// //   }

// //   // Called repeatedly when this Command is scheduled to run
// //   @Override
// //   public void execute() {
// //     if (debug) {
// //       double[] params = {controller.getPassedTime(), controller.getSetpointSegment().translation.x, driveSystem.getKinematicPosition().x,
// //         controller.getSetpointSegment().translation.y, driveSystem.getKinematicPosition().y, controller.getSetpointSegment().heading.toDegrees(), driveSystem.navX.getYaw()};
// //       Utils.addCSVLine(this.handle, params);
// //       Utils.addCSVLine(debugHandle, params);
// //     }
// //   }

// //   // Make this return true when this Command no longer needs to run execute()
// //   @Override
// //   public boolean isFinished() {
// //     // return isTimedOut();
// //     return controller.isTimeUp();
// //     // return false;
// //   }

// //   // Called once after isFinished returns true
// //   @Override
// //   public void end(boolean interrupted) {
// //     this.controller.disable();
// //     Constants.autoActive = false;
// //   }
// // }
