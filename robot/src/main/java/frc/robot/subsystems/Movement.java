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
    rightMaster = new WPI_TalonSRX(RobotMap.MOVEMENT_RIGHT_MASTER_MOTOR_DEVICE_NUM);
    rightSlave1 = new WPI_TalonSRX(RobotMap.MOVEMENT_RIGHT_SLAVE1_MOTOR_DEVICE_NUM);
    rightSlave2= new WPI_TalonSRX(RobotMap.MOVEMENT_RIGHT_SLAVE2_MOTOR_DEVICE_NUM);
    leftMaster = new WPI_TalonSRX(RobotMap.MOVEMENT_LEFT_MASTER_MOTOR_DEVICE_NUM);
    leftSlave1 = new WPI_TalonSRX(RobotMap.MOVEMENT_LEFT_SLAVE1_MOTOR_DEVICE_NUM);
    leftSlave2 = new WPI_TalonSRX(RobotMap.MOVEMENT_LEFT_SLAVE2_MOTOR_DEVICE_NUM);

    rightSlave1.set(ControlMode.Follower, rightMaster.getDeviceID());
    rightSlave2.set(ControlMode.Follower, rightMaster.getDeviceID());
    leftSlave1.set(ControlMode.Follower, leftMaster.getDeviceID());
    leftSlave2.set(ControlMode.Follower, leftMaster.getDeviceID());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
