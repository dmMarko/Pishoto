// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.LibPurple.control.PID254;
import frc.LibPurple.utils.Utils;
import frc.robot.RobotContainer;

public class DriveJoystick extends CommandBase {
  /** Creates a new DriveJoystick. */
  public DriveJoystick(Subsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double angle = RobotContainer.driver.xGet();
    double power = Utils.deadband(-RobotContainer.driver.getRawAxis(1), 0.05);
    
    
    double rightPower;
    double leftPower;

    rightPower = power + angle;
    leftPower = power - angle;
    // rightPower = power;
    // leftPower = power * ratio;
    
    SmartDashboard.putNumber("leftPower", leftPower);
    SmartDashboard.putNumber("rightPower", rightPower);
    SmartDashboard.putNumber("ratio", angle);
    SmartDashboard.putNumber("leftEncoder", RobotContainer.drive.getLeftPos());
    SmartDashboard.putNumber("rightEncoder", RobotContainer.drive.getRightPos());
    RobotContainer.drive.setPower(leftPower, rightPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
