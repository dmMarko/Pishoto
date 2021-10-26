package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.LibPurple.control.PID254;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class setArmPos extends CommandBase {
  private double armPos;
  private PID254 pid;

  public setArmPos(double armPos) {
    this.armPos=armPos;
    this.pid = new PID254(Constants.ARM_PID.kP, Constants.ARM_PID.kI, Constants.ARM_PID.kD);
    this.pid.setSetpoint(armPos);
    this.pid.setInputRange(Constants.armMinInput, Constants.armMaxInput);
    this.pid.setOutputRange(Constants.armMinOutput, Constants.armMaxOutput);
    
    addRequirements(RobotContainer.arm);
  }

    @Override
    public void execute() {
      RobotContainer.arm.setPosition(this.pid.calculate(RobotContainer.arm.getPos()));
    }

    @Override
    public boolean isFinished() {
      // TODO Auto-generated method stub
      return this.pid.onTarget(Constants.armTolerance);
    }

    @Override
    public void end(boolean interrupted) {
      RobotContainer.arm.setPower(0);
    }
}