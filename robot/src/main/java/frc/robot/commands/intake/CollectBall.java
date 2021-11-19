package frc.robot.commands.intake;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.intake.Helpers.setPiston;
import frc.robot.commands.intake.Helpers.setPiston.Piston;

public class CollectBall extends CommandBase {
  /**
   * Creates a new CollectBall.
   */
  public CollectBall() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(new setPiston(false, Piston.Holder).andThen(new setPiston(false,Piston.PushHolder)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.intake.setPowerUpperWheel(Constants.COLLECT_BALL_TOP_SPIN_POWER);
    RobotContainer.intake.setPowerLowerWheel(Constants.COLLECT_BALL_BOTTOM_SPIN_POWER);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.setPowerLowerWheel(0);
    RobotContainer.intake.setPowerUpperWheel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
