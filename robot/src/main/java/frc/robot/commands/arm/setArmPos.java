/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.LibPurple.utils.Utils;
import frc.robot.Constants;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetArmPos extends CommandBase {
  private double armPos;

  public SetArmPos(double armPos, Subsystem requirements) {
    if(armPos > Constants.ARM_MIN_INPUT || armPos < Constants.ARM_MAX_INPUT){
      this.armPos = armPos;
    } else {
      this.armPos = 0;
    }
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(requirements);
  }

  public SetArmPos(double armPos){
    if(armPos > Constants.ARM_MIN_INPUT || armPos < Constants.ARM_MAX_INPUT){
      this.armPos = armPos;
    } else {
      this.armPos = RobotContainer.arm.getPos();
    }
    addRequirements(RobotContainer.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    RobotContainer.arm.setPosition(this.armPos);
    // double power = -RobotContainer.driver.getY();
    // SmartDashboard.putNumber("ArmPower", power);
    // RobotContainer.arm.setPower(power);
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
