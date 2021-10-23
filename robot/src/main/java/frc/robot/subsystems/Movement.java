/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.LibPurple.control.PIDvalue;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.commands.Drive.DriveJoystick;

public class Movement extends SubsystemBase {
  private WPI_TalonSRX rightMaster;
  private WPI_TalonSRX rightSlave1;
  private WPI_TalonSRX rightSlave2;
  private WPI_TalonSRX leftMaster;
  private WPI_TalonSRX leftSlave1;
  private WPI_TalonSRX leftSlave2;
  /**
   * Creates a new Movement.
   */
  public Movement() {
    // right side
    rightMaster = new WPI_TalonSRX(RobotMap.MOVEMENT_RIGHT_MASTER_MOTOR_DEVICE_NUM);
    rightSlave1 = new WPI_TalonSRX(RobotMap.MOVEMENT_RIGHT_SLAVE1_MOTOR_DEVICE_NUM);
    rightSlave2 = new WPI_TalonSRX(RobotMap.MOVEMENT_RIGHT_SLAVE2_MOTOR_DEVICE_NUM);

    // left side
    leftMaster = new WPI_TalonSRX(RobotMap.MOVEMENT_LEFT_MASTER_MOTOR_DEVICE_NUM);
    leftSlave1 = new WPI_TalonSRX(RobotMap.MOVEMENT_LEFT_SLAVE1_MOTOR_DEVICE_NUM);
    leftSlave2 = new WPI_TalonSRX(RobotMap.MOVEMENT_LEFT_SLAVE2_MOTOR_DEVICE_NUM);

    rightSlave1.set(ControlMode.Follower, rightMaster.getDeviceID());
    rightSlave2.set(ControlMode.Follower, rightMaster.getDeviceID());

    leftSlave1.set(ControlMode.Follower, leftMaster.getDeviceID());
    leftSlave2.set(ControlMode.Follower, leftMaster.getDeviceID());

    this.configPID(Constants.leftSidePID, Constants.rightSidePID);

    CommandScheduler.getInstance().setDefaultCommand(RobotContainer.drive, new DriveJoystick());
  }


  public void setPower(double leftPower, double rightPower) {
    this.rightMaster.set(ControlMode.PercentOutput, leftPower);
    this.leftMaster.set(ControlMode.PercentOutput, rightPower);
  }

  public void stop() {
    this.setPower(0, 0);
  }

  public void configPID(PIDvalue leftSide, PIDvalue rightSide) {
    this.leftMaster.config_kP(0, leftSide.kP);
    this.leftMaster.config_kI(0, leftSide.kI);
    this.leftMaster.config_kD(0, leftSide.kD);

    this.rightMaster.config_kP(0, rightSide.kP);
    this.rightMaster.config_kI(0, rightSide.kI);
    this.rightMaster.config_kD(0, rightSide.kD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
