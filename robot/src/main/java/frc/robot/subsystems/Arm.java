/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase {
  private WPI_TalonSRX motor;
  private AnalogInput encoder;
  /**
   * Creates a new Arm.
   */

  public Arm() {
    encoder=new AnalogInput(RobotMap.ARM_ENCODER_CHANNLE);
    motor = new WPI_TalonSRX(RobotMap.ARM_MOTOR_CHANNLE);
  }
  public void setArmPos(double armPos){
    motor.set(ControlMode.PercentOutput,armPos);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
