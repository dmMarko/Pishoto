package frc.robot.commands.intake;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class CollectBall extends CommandBase {
  /**
   * Creates a new CollectBall.
   */
  public CollectBall(Subsystem... requirements) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(requirements);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.intake.spinLowerWheel(Constants.COLLECT_BALL_BOTTOM_SPIN_POWER);
    RobotContainer.intake.spinUpperWheel(Constants.COLLECT_BALL_TOP_SPIN_POWER);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
