/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.SetArmAngle;
import frc.robot.commands.intake.Helpers.RaiseArmFromFloor;

public class CollectDiskFromFloor extends CommandBase {
  /**
   * Creates a new CollectDiskFromFloor.
   */
  public CollectDiskFromFloor() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake, RobotContainer.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intake.setHolderPistonValue(Value.kReverse);
    RobotContainer.intake.setFloorPistonValue(Value.kReverse);
    CommandScheduler.getInstance().schedule(new SetArmAngle(Constants.ARM_FLOOR_ANGLE));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.intake.spinLowerWheel(Constants.COLLECT_DISK_FROM_FLOOR_SPIN_POWER);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.setFloorPistonValue(Value.kForward);
    CommandScheduler.getInstance().schedule(new WaitCommand(1));
    RobotContainer.intake.setPushHolderPistonValue(Value.kForward);
    RobotContainer.intake.setHolderPistonValue(Value.kForward);
    CommandScheduler.getInstance().schedule(new RaiseArmFromFloor());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}