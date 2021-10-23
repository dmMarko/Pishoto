// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.LibPurple.swerve.SDriveCommands;

// import java.io.FileWriter;
// import java.io.IOException;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.LibPurple.math.Vector2D;
// import frc.LibPurple.swerve.Swerve.SwerveDrivetrain;
// import frc.LibPurple.swerve.Swerve.SwerveModule;
// import frc.LibPurple.swerve.Swerve.SwerveDrivetrain.ControllerType;
// import frc.LibPurple.swerve.controlTrajectory.TrajectoryController;
// import frc.LibPurple.swerve.Utils.Utils;
// import frc.robot.Constants;

// public abstract class SDriveCommand extends CommandBase {
//   /**
//    * Creates a new SDriveCommand.
//    */
//   protected SwerveDrivetrain driveSystem;
//   protected TrajectoryController controller;
//   protected ControllerType controllerType;

//   protected boolean debug;
//   protected String filenameDebug;
//   protected FileWriter handle;
//   protected FileWriter debugHandle;
//   protected String debugText = "";

//   public SDriveCommand(SwerveDrivetrain driveSystem, ControllerType controllerType, boolean debug) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(driveSystem);
//     this.driveSystem = driveSystem;
//     this.controllerType = controllerType;
//     this.debug = debug;
//   }

//   public abstract void setTrajectory();

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     driveSystem.resetKinematics(Timer.getFPGATimestamp());
//     driveSystem.setController(controllerType);
//     controller = driveSystem.getController();
//     controller.resetLocalization();
//     setTrajectory();
//     Utils.print("SDriveCommand time=" + controller.getTrajectory().getTotalTime());
//     Constants.autoActive = true;
//     controller.enable();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if(debug){
//       double[] params = {controller.getPassedTime(), controller.getSetpointSegment().translation.x, driveSystem.getKinematicPosition().x,
//           controller.getSetpointSegment().translation.y, driveSystem.getKinematicPosition().y,
//           controller.getSetpointSegment().rotation.toDegrees(), driveSystem.navX.getYaw(), controller.getSetpointSegment().velocity, driveSystem.getKinematicVelocity().length};
//       for(int i = 0; i < params.length - 1; i++){
//         debugText += params[i] + "\t,";
//       }
//       debugText += params[params.length - 1];
//       debugText += "\n";
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     if(debug){
//       handle = Utils.initializeCSVFile("/graphs/" + filenameDebug);
//       Utils.addCSVLine(handle, new String[] {"Time", "Trajectory X", "Real X", "Trajectory Y", "Real Y" });
//       debugHandle = Utils.initializeDebugFile("/graphs/");
//       Utils.addCSVLine(debugHandle, new String[] {"Time", "Trajectory X", "Real X", "Trajectory Y", "Real Y" , "Trajectory heading", "Real Heading"});
//       try{
//         handle.write(debugText);
//         debugHandle.write(debugText);
//         handle.close();
//         debugHandle.close();
//       } catch (IOException e){
//         e.printStackTrace();
//       }
//     }
//     Constants.autoActive = false;
//     controller.disable();
//     Utils.print("SDriveCommand Done");
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     // return false;
//     return controller.isTimeUp();
//   }
// }
