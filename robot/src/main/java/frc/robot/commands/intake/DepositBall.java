/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DepositBall extends CommandBase {
  double time;
  /**
   * Creates a new DepositBall.
   */
  public DepositBall() {
    time = Timer.getFPGATimestamp();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.intake.setPowerLowerWheel(-2*Constants.COLLECT_BALL_BOTTOM_SPIN_POWER);
    RobotContainer.intake.setPowerUpperWheel(-2*Constants.COLLECT_BALL_TOP_SPIN_POWER);
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
    // return Timer.getFPGATimestamp() - time > 50;
    return false;
  }
}
