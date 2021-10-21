import frc.robot.RobotContainer;

public class setArmPerc extends InstantCommand {

double armPos;
    public setArmPerc(double armPos) {
        this.armPos=armPos;
        addRequirements(RobotContainer.arm());
    
    }
    @Override
    public void initialize() {
      RobotContainer.arm.setArmPos(armPos);
    }
}