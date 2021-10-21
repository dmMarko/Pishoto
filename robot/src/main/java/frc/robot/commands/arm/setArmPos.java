package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class setArmPos extends InstantCommand {

  double armPos;
    public setArmPos(double armPos) {
        this.armPos=armPos;
        addRequirements(RobotContainer.arm);
    
    }
    @Override
    public void initialize() {
      RobotContainer.arm.setArmPos(armPos);
    }
}