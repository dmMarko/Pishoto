/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
  private WPI_TalonSRX upperMotor;
  private WPI_TalonSRX lowerMotor;

  private DoubleSolenoid holderPiston;
  private DoubleSolenoid pushHolderPiston;
  private DoubleSolenoid floorPiston;

  /**
   * Creates a new Intake.
   */
  public Intake() {
    upperMotor = new WPI_TalonSRX(RobotMap.INTAKE_UPPER_MOTOR_DEVICE_NUM);
    lowerMotor = new WPI_TalonSRX(RobotMap.INTAKE_LOWER_MOTOR_DEVICE_NUM);

    holderPiston = new DoubleSolenoid(RobotMap.INTAKE_HOLDER_PISTON_FORWARD_CHANNLE,
        RobotMap.INTAKE_HOLDER_PISTON_REVERSE_CHANNLE);
    pushHolderPiston = new DoubleSolenoid(RobotMap.INTAKE_PUSH_HOLDER_PISTON_FORWARD_CHANNLE,
        RobotMap.INTAKE_PUSH_HOLDER_PISTON_REVERSE_CHANNLE);
    floorPiston = new DoubleSolenoid(RobotMap.INTAKE_FLOOR_PISTON_FORWARD_CHANNLE,
        RobotMap.INTAKE_FLOOR_PISTON_REVERSE_CHANNLE);
  }

  public void setHolderPistonValue(Value value) {
    holderPiston.set(value);
  }

  public void setPushHolderPistonValue(Value value) {
    pushHolderPiston.set(value);
  }

  public void setFloorPistonValue(Value value) {
    floorPiston.set(value);
  }

  public void spinUpperWheel(double power) {
    upperMotor.set(ControlMode.PercentOutput, power);
  }

  public void spinLowerWheel(double power) {
    lowerMotor.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
