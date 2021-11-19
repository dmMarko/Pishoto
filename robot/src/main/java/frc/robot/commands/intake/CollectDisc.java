/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Elevator.setElevatorPosition;
import frc.robot.commands.arm.SetArmAngle;
import frc.robot.commands.intake.Helpers.setPiston;
import frc.robot.commands.intake.Helpers.setPiston.Piston;

public class CollectDisc extends CommandBase {
  /**
   * Creates a new TakeDisc.
   */
  public CollectDisc() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake, RobotContainer.arm, RobotContainer.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(new setPiston(false, Piston.Holder));
    CommandScheduler.getInstance().schedule(new setPiston(true, Piston.PushHolder));
    CommandScheduler.getInstance().schedule(new setElevatorPosition(Constants.TAKE_DISC_HIGHT));
    CommandScheduler.getInstance().schedule(new SetArmAngle(0));
  }

  @Override
  public boolean isFinished() {
      return false;
  }

  @Override
  public void end(boolean interrupted) {
    //   
    CommandScheduler.getInstance().schedule(new setPiston(true, Piston.Holder));

    
  }
}
