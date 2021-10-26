// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.LibPurple.swerve.SDriveCommands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.LibPurple.math.Vector2D;
// import frc.LibPurple.swerve.Swerve.SwerveDrivetrain;
// import frc.LibPurple.utils.Utils;
// import frc.robot.Constants;
// import frc.robot.Robot;
// import frc.robot.RobotContainer;


// public class HolonomicDriveCommand extends CommandBase {

//   private SwerveDrivetrain drivetrain;

//   private double lastForward1 = 0.0;
//   private double lastForward2 = 0.0;
//   private double lastForward3 = 0.0;
//   private double lastForward4 = 0.0;
//   private double lastStrafe1 = 0.0;
//   private double lastStrafe2 = 0.0;
//   private double lastStrafe3 = 0.0;
//   private double lastStrafe4 = 0.0;
//   private double lastRotation1 = 0.0;
//   private double lastRotation2 = 0.0;
//   private double lastRotation3 = 0.0;
//   private double lastRotation4 = 0.0;

//   public HolonomicDriveCommand(SwerveDrivetrain drivetrain) {
//     this.drivetrain = drivetrain;
//     addRequirements(this.drivetrain);
//   }

//   // Called just before this Command runs the first time
//   @Override
//   public void initialize() {
//   }

//   // // Called repeatedly when this Command is scheduled to run
//   @Override
//   public void execute() {
//     double forward = -RobotContainer.driverStick.getY();
//     // double forward = 0;
//     double strafe = RobotContainer.driverStick.getRawAxis(0);
//     double rotation = RobotContainer.driverStick.xGet() / 1.8;
    
//     forward = (forward + lastForward1 + lastForward2 + lastForward3 + lastForward4) / 5;
//     strafe = (strafe + lastStrafe1 + lastStrafe2 + lastStrafe3 + lastStrafe4) / 5;
//     rotation = (rotation + lastRotation1 + lastRotation2 + lastRotation3 + lastRotation4) / 5;
//     forward = Utils.deadband(forward, 0.05);
//     strafe = Utils.deadband(strafe, 0.05);
//     rotation = Utils.deadband(rotation, 0.01);

//     Vector2D translation = new Vector2D(forward, strafe);

//     drivetrain.holonomicDrive(translation, rotation, Constants.robotOriented);
//     // drivetrain.getSwerveModules()[0].setAngleOutput(0.1);
    
//     double xForward = lastForward1;
//     double yForward = lastForward2;
//     double zForward = lastForward3;
//     lastForward1 = forward;
//     lastForward2 = xForward;
//     lastForward3 = yForward;
//     lastForward4 = zForward;

//     double xStrafe = lastStrafe1;
//     double yStrafe = lastStrafe2;
//     double zStrafe = lastStrafe3;
//     lastStrafe1 = strafe;
//     lastStrafe2 = xStrafe;
//     lastStrafe3 = yStrafe;
//     lastStrafe4 = zStrafe;

//     double xRotation = lastRotation1;
//     double yRotation = lastRotation2;
//     double zRotation = lastRotation3;
//     lastRotation1 = rotation;
//     lastRotation2 = xRotation;
//     lastRotation3 = yRotation;
//     lastRotation4 = zRotation;

//   }

//   // Make this return true when this Command no longer needs to run execute()
//   @Override
//   public boolean isFinished() {
//     return false;
//   }

//   // Called once after isFinished returns true
//   @Override
//   public void end(boolean interrupted) {
//   }
// }
