// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Drive;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// // import frc.LibPurple.control.PID254;
// import frc.robot.RobotContainer;

// public class DriveJoystick extends CommandBase {
//   /** Creates a new DriveJoystick. */
//   public DriveJoystick() {
//     // Use addRequirements() here to declare subsystem dependencies.
//     // addRequirements(RobotContainer.drive);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double r2 = RobotContainer.driver.getR2();
//     double l2 = RobotContainer.driver.getL2();
//     double power = RobotContainer.driver.yGet();
    
//     double rightPower;
//     double leftPower;

//     double ratio = l2 > r2 ? r2 / l2 : l2 / r2;
    
//     if (ratio > 0) {
//       rightPower = power;
//       leftPower = power * ratio;
//     } else {
//       leftPower = power;
//       rightPower = power * ratio;
//     }

//     RobotContainer.drive.setPower(leftPower, rightPower);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     RobotContainer.drive.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
