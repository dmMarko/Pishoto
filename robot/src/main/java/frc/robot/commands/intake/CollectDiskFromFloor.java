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
import frc.LibPurple.swerve.Utils.Utils;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Elevator.setElevatorPosition;
import frc.robot.commands.arm.SetArmAngle;
import frc.robot.commands.intake.Helpers.RaiseArmFromFloor;
import frc.robot.commands.intake.Helpers.setPiston;
import frc.robot.commands.intake.Helpers.setPiston.Piston;

public class CollectDiskFromFloor extends CommandBase {
  /**
   * Creates a new CollectDiskFromFloor.
   */
  public CollectDiskFromFloor() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake, RobotContainer.arm, RobotContainer.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Utils.print("pablo tambal");
    CommandScheduler.getInstance().schedule(new setPiston(false, Piston.PushHolder));
    CommandScheduler.getInstance().schedule(new setPiston(false, Piston.Holder));
    CommandScheduler.getInstance().schedule(new setPiston(false, Piston.Floor));
    CommandScheduler.getInstance().schedule(new SetArmAngle(Constants.ARM_FLOOR_ANGLE));
    CommandScheduler.getInstance().schedule(new setElevatorPosition(0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.intake.setPowerLowerWheel(Constants.COLLECT_DISC_FROM_FLOOR_SPIN_POWER);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Utils.print("boaz very noaz");
    RobotContainer.intake.setPowerLowerWheel(0);

    CommandScheduler.getInstance().schedule(new RaiseArmFromFloor().andThen(new WaitCommand(2)).andThen(new setPiston(true, Piston.PushHolder)).andThen(new setPiston(true, Piston.Holder)).andThen(new WaitCommand(1)).andThen(new setPiston(false, Piston.Floor)));
    // Utils.print("pablo very tambal");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}