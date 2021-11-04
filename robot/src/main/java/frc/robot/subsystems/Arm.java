/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.commands.arm.SetArmPos;

public class Arm extends SubsystemBase {
  private WPI_TalonSRX motor;
  /**
   * Creates a new Arm.
   */

  public Arm() {
    motor = new WPI_TalonSRX(RobotMap.ARM_MOTOR_CHANNLE);
    motor.setSelectedSensorPosition(0);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.setInverted(false);
    configPID();
    
    // setDefaultCommand(new SetArmPos(0, this));
    // CommandScheduler.getInstance().setDefaultCommand(this, new SetArmPos(0, this));
    // motor.setSelectedSensorPosition(sensorPos)
  }
  
  public void setPower(double power){
    motor.set(ControlMode.PercentOutput, power);
  }

  public void configPID(){
    motor.config_kP(0, Constants.ARM_PID_KP);
    motor.config_kI(0, Constants.ARM_PID_KI);
    motor.config_kD(0, Constants.ARM_PID_KD);
  }

  public void setPosition(double pos) {
    motor.set(ControlMode.Position, pos);
  }

  public void setPositionByAngle(double angle){
    setPosition(angle*-37.73333333);
  }

  public double getPos(){
    return motor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("armPos", getPos());
    SmartDashboard.putNumber("armAngle", getPos()/-37.73333333);
  }
}
