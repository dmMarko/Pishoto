// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 first. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the first BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.LibPurple.DriveCommands;

// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.Joystick.AxisType;
// import edu.wpi.first.wpilibj.command.Command;
// import frc.LibPurple.systems.DriveSystem3075;

// public class ArcadeDriveCommand extends Command {
//   DriveSystem3075 driveSystem;
//   Joystick stick;

//   private double leftValue;
//   private double rightValue;

//   public ArcadeDriveCommand(DriveSystem3075 driveSystem, Joystick stick) {
//     requires(driveSystem);
//     this.driveSystem = driveSystem;
//     this.stick = stick;
//     // Use requires() here to declare subsystem dependencies
//     // eg. requires(chassis);
//   }

//   // Called just before this Command runs the first time
//   @Override
//   protected void initialize() {
//   }

//   // Called repeatedly when this Command is scheduled to run
//   @Override
//   protected void execute() {
//     double throttle = stick.getRawAxis(AxisType.kY.value);
//     double turn = stick.getRawAxis(AxisType.kX.value);

//     leftValue = throttle + turn;
//     rightValue = throttle - turn;

//     driveSystem.set(rightValue, leftValue);
//   }

//   // Make this return true when this Command no longer needs to run execute()
//   @Override
//   protected boolean isFinished() {
//     return false;
//   }

//   // Called once after isFinished returns true
//   @Override
//   protected void end() {
//   }

//   // Called when another command which requires one or more of the same
//   // subsystems is scheduled to run
//   @Override
//   protected void interrupted() {
//   }
// }
