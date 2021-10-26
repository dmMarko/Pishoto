// package frc.LibPurple.swerve.SDriveCommands;

// import frc.LibPurple.swerve.Swerve.SwerveDrivetrain;
// import frc.LibPurple.swerve.Swerve.SwerveDrivetrain.ControllerType;
// import frc.LibPurple.swerve.trajectories.TrajectoryFile;

// public class SDrive2DCommand extends SDriveCommand {

//   String filename;

//   boolean reversed;

//   public SDrive2DCommand(SwerveDrivetrain driveSystem, String filename, boolean reversed, ControllerType controllerType, boolean debug){
//     super(driveSystem, controllerType, debug);
//     this.filename = filename;
//     this.reversed = reversed;
//     filenameDebug = "SBeizer";
//   }

//   @Override
//   public void setTrajectory() {
//     // TODO Auto-generated method stub
//     controller.setTrajectory(new TrajectoryFile(filename, reversed));
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

// // import edu.wpi.first.wpilibj.Timer;
// // import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// // import edu.wpi.first.wpilibj2.command.CommandBase;
// // import frc.LibPurple.control.PIDvalue;
// // import frc.LibPurple.math.Vector2D;
// // import frc.LibPurple.swerve.Swerve.SwerveDrivetrain;
// // import frc.LibPurple.swerve.Swerve.SwerveDrivetrain.ControllerType;
// // import frc.LibPurple.swerve.Utils.Utils;
// // import frc.LibPurple.swerve.controlTrajectory.TrajectoryController;
// // import frc.LibPurple.swerve.trajectories.TrajectoryFile;
// // import frc.robot.Constants;
// // import frc.robot.Robot;
// // import frc.robot.RobotContainer;


// // public class SDrive2DCommand extends CommandBase {
// //   // private final Supplier<Trajectory> trajectorySupplier;

// //   SwerveDrivetrain driveSystem;
// // 	TrajectoryController controller;
// // 	PIDvalue rightVelocityPIDValue;
// // 	PIDvalue leftVelocityPIDValue;
// // 	boolean reversed;
// // 	boolean debug;
// // 	String fileName;

// // 	ControllerType controllerType;

// //   FileWriter handle;
// //   FileWriter debugHandle;

// //   String text = "";


// //   public SDrive2DCommand(SwerveDrivetrain driveSystem, String fileName, boolean reversed, ControllerType controllerType, boolean debug) {
// //     // Use requires() here to declare subsystem dependencies
// //     // eg. requires(chassis);
// //     addRequirements(driveSystem);
// //     this.debug = debug;
// // 		this.driveSystem = driveSystem;
// // 		this.fileName = fileName;
// // 		this.controllerType = controllerType;
// //     this.reversed = reversed;
// //   }

// //   // public SDrive2DCommand(Supplier<Trajectory> trajectorySupplier) {
// //   //   this.trajectorySupplier = trajectorySupplier;

// //   //   requires(Robot.swerve);
// // // }

// //   // Called just before this Command runs the first time
// //   @Override
// //   public void initialize() {
// //     // trajectory = trajectorySupplier.get();
// //     // Robot.swerve.resetKinematics(Vector2D.ZERO, Timer.getFPGATimestamp());
// //     // Robot.swerve.getFollower().follow(trajectory);
	
// // 		// this.handle = Robot.handle;
// // 		// this.driveSystem.setController(controllerType);
// // 		// this.controller = driveSystem.getController();
// // 		// controller.setTrajectory(new TrajectoryFile(this.fileName, reversed));
// // 		// controller.setTrajectory(Robot.trajectoryFile);
// // 		// controller.setDirection(reversed);
// // 		// driveSystem.resetLocalizataion();
// //     // Robot.driveSystem.setInvertedDirection(reversed);

// //     // for(int i = 0; i < driveSystem.getSwerveModules().length; i++){
// //       // driveSystem.getSwerveModules()[i].resetKinematics();
// //     // }
// //     driveSystem.resetEncoders();
// //     driveSystem.resetKinematics(Timer.getFPGATimestamp());

// //     this.driveSystem.setController(this.controllerType);
// //     this.controller = driveSystem.getController();
// //     this.controller.resetLocalization();
// //     this.controller.setTrajectory(new TrajectoryFile(this.fileName, this.reversed));
// //     // this.controller.setDirection(this.reversed);
// //     this.controller.enable();
// //     Constants.autoActive = true;
// //     withTimeout(controller.getTrajectory().getTotalTime());
// //   }

// //   // Called repeatedly when this Command is scheduled to run
// //   @Override
// //   public void execute() {
// //     // double[] params = { controller.getPassedTime(), controller.getSetpoint()[0],
// // 		// 		driveSystem.getX(), controller.getSetpoint()[1],
// // 		// 		driveSystem.getY(), controller.getSetpoint()[2], driveSystem.getAngleRad(),
// // 		// 		controller.getSetpointVelocity(),
// // 		// 		(driveSystem.getLeftEncoder().getRate() + driveSystem.getRightEncoder().getRate()) / 2.0 };
// // 		// double[] params = { controller.getPassedTime(), controller.getSetpoint()[0], controller.getSetpoint()[1]};
// // 		// double[] params = {controller.getPassedTime(), controller.getLeftSpeed(), Robot.driveSystem.getLeftEncoder().getRate(), controller.getPassedTime(), controller.getRightSpeed(), Robot.driveSystem.getLeftEncoder().getRate()};
// // 		// if(debug)
// //     //   Utils.addCSVLine(this.handle, params);
// //     if(debug){
// //       double[] params = {controller.getPassedTime(), controller.getSetpointSegment().translation.x, driveSystem.getKinematicPosition().x,
// //           controller.getSetpointSegment().translation.y, driveSystem.getKinematicPosition().y,
// //           controller.getSetpointSegment().heading.toDegrees(), driveSystem.navX.getYaw(), Math.toDegrees(driveSystem.getSwerveModules()[0].getAngle()),
// //           Math.toDegrees(driveSystem.getSwerveModules()[0].getError()), Math.toDegrees(driveSystem.getSwerveModules()[0].getTargetAngle()),
// //           Math.toDegrees(driveSystem.getSwerveModules()[1].getAngle()), Math.toDegrees(driveSystem.getSwerveModules()[1].getError()),
// //           Math.toDegrees(driveSystem.getSwerveModules()[1].getTargetAngle()), Math.toDegrees(driveSystem.getSwerveModules()[2].getAngle()),
// //           Math.toDegrees(driveSystem.getSwerveModules()[2].getError()), Math.toDegrees(driveSystem.getSwerveModules()[2].getTargetAngle()),
// //           Math.toDegrees(driveSystem.getSwerveModules()[3].getAngle()), Math.toDegrees(driveSystem.getSwerveModules()[3].getError()),
// //           Math.toDegrees(driveSystem.getSwerveModules()[3].getTargetAngle())};
// //       // Utils.addCSVLine(handle, params);
// //       // Utils.addCSVLine(debugHandle, params);
// //       for(int i = 0; i < params.length - 1; i++){
// //         text += params[i] + ", ";
// //       }
// //       text += params[params.length - 1];
// //       text += "\n";
// //     }

// //     // driveSystem.updateKinematics(Timer.getFPGATimestamp());
// //   }

// //   // Make this return true when this Command no longer needs to run execute()
// //   @Override
// //   public boolean isFinished() {
// //     // return true;
// //     return controller.isTimeUp();
// //     // return false;
// //   }

// //   // Called once after isFinished returns true
// //   @Override
// //   public void end(boolean interrupted) {
// //     if(debug){
// //       this.handle = Utils.initializeCSVFile("/graphs/SBeizerV2");
// //       Utils.addCSVLine(this.handle, new String[] {"Time", "Trajectory X", "Real X", "Trajectory Y", "Real Y" });
// //       // handle.write(text);  
// //       this.debugHandle = Utils.initializeDebugFile("/graphs/");
// //       Utils.addCSVLine(this.debugHandle, new String[] {"Time", "Trajectory X", "Real X", "Trajectory Y", "Real Y" , "Trajectory heading", "Real Heading"});
// //     }
// //     // Utils.closeCSVFile(this.handle);
// //     controller.disable();
// //     driveSystem.setZeroPower();
// //     Constants.autoActive = false;
// //     Utils.print("  " + Math.toDegrees(driveSystem.getSwerveModules()[0].getTargetAngle()));
// //     Utils.print("angle " + Math.toDegrees(driveSystem.getSwerveModules()[0].getAngle()));
// //     Utils.print("error " + Math.toDegrees(driveSystem.getSwerveModules()[0].getError()));
// //     Utils.print("2D Done!");
// //     Utils.print("" + Math.toDegrees(controller.getTrajectory().getLastSegment().rotation.toRadiansWithout2Pi()));
// //   }
// // }
