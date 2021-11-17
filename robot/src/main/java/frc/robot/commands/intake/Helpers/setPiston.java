/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake.Helpers;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class setPiston extends InstantCommand {
  public enum Piston{
    Floor,
    Holder,
    PushHolder
  }

  private Piston piston;
  Value value;
  public setPiston(boolean forward, Piston piston){
    this.value = forward ? Value.kForward : Value.kReverse;
    this.piston = piston;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (piston == Piston.Floor){
      RobotContainer.intake.setFloorPistonValue(value);
    } else if (piston == Piston.Holder) {
      RobotContainer.intake.setHolderPistonValue(value);
    } else if (piston == Piston.PushHolder) {
      RobotContainer.intake.setPushHolderPistonValue(value);
    }
  }
}
