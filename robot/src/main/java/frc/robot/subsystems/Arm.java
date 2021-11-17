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

    motor.configPeakOutputForward(Constants.ARM_MAX_POWER_FORWARD);
    motor.configPeakOutputReverse(Constants.ARM_MAX_POWER_REVERSE);
    configPID();
  }

  public void setPower(double power) {
    motor.set(ControlMode.PercentOutput, power);
  }

  public void configPID() {
    motor.config_kP(0, Constants.ARM_PID.kP);
    motor.config_kI(0, Constants.ARM_PID.kI);
    motor.config_kD(0, Constants.ARM_PID.kD);

  }

  public void setPosition(double pos) {
    if (pos > Constants.ARM_MAX_POS)
      motor.set(ControlMode.Position, Constants.ARM_MAX_POS);
    if (pos > Constants.ARM_MIN_POS)
      motor.set(ControlMode.Position, Constants.ARM_MIN_POS);
    motor.set(ControlMode.Position, pos);
  }

  public void setPositionByAngle(double angle) {
    setPosition(angle * (37 + 11 / 15));
  }

  public double getPos() {
    return motor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Position", getPos());
    SmartDashboard.putNumber("Arm Angle", getPos() / (37 + 7 / 15));
  }
}
