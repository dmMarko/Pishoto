/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;

public class SetArmAngle extends CommandBase {
  private double armAngle;

  public SetArmAngle(double armAngle, Subsystem requirements) {
    this.armAngle = armAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(requirements);
  }

  public SetArmAngle(double armAngle) {
    this.armAngle = armAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(RobotContainer.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    RobotContainer.arm.setPositionByAngle(this.armAngle);
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return false;
    // return Utils.inRange(RobotContainer.arm.getPos(), this.armPos, 5);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.arm.setPower(0);
  }
}
