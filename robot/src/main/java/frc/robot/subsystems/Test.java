/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Test extends SubsystemBase {
  WPI_TalonSRX motor;
  /**
   * Creates a new Test.
   */
  public Test() {
    motor = new WPI_TalonSRX(RobotMap.TEST_MOTOR_DEVICE_NUM);
  }

  public void setPower(double power) {
    motor.set(ControlMode.PercentOutput, power);
  }
  public void setPosition(double pos) {
    motor.set(ControlMode.Position, pos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
