// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 first. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the first BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.LibPurple.DriveCommands;

// import edu.wpi.first.wpilibj.command.Command;
// import frc.LibPurple.sensors.ConsoleJoystick;
// import frc.LibPurple.systems.DriveSystem3075;
// import frc.LibPurple.utils.Utils;
// public class XBoxArcadeCommand extends Command {
//   DriveSystem3075 driveSystem;
// 	ConsoleJoystick stick;

// 	private double leftValue;
// 	private double rightValue;
// 	private double last = 0;
// 	private double last2 = 0;
// 	private double last3 = 0;
// 	private double last4 = 0;
//   public XBoxArcadeCommand(DriveSystem3075 driveSystem, ConsoleJoystick stick)
//   {
//     requires(driveSystem);
// 		this.driveSystem = driveSystem;
// 		this.stick = stick;
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
//     double throttle = stick.yGet();
// 		double turn = Math.pow(stick.xGet(), 2) * Math.signum(stick.xGet());

// 		// double throttle = stick.getDrivingY();
// 		// double turn = stick.getRawAxis(0);
// 		throttle = Utils.deadband(throttle, 0.01);
// 		// turn = Utils.deadband(turn, 0.3);

// 		// throttle = Utils.accellimit(throttle, last, 0.8);
// 		throttle = (throttle + last + last2 + last3 + last4) / 5;
// 		leftValue = (throttle + turn);
// 		rightValue = (throttle - turn);

// 		driveSystem.set(rightValue, leftValue);

// 		double x = last;
// 		double y = last2;
// 		double z = last3;
// 		last = throttle;
// 		last2 = x;
// 		last3 = y;
// 		last4 = z;
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
