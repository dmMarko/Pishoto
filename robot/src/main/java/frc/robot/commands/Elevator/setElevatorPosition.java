/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.LibPurple.utils.Utils;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class setElevatorPosition extends CommandBase {
  private double elevatorPos;
  /**
   * Creates a new setElevatorPosition.
   */
  public setElevatorPosition(double elevatorPos) {
    this.elevatorPos = elevatorPos;
    
    addRequirements(RobotContainer.elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public setElevatorPosition(double elevatorPos, Subsystem... requirements) {
    this.elevatorPos = elevatorPos;
    addRequirements(requirements);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.elevator.setPosition(elevatorPos);
    // double power = RobotContainer.driver.getY();
    // RobotContainer.elevator.setPower(power);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.elevator.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
