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
import frc.LibPurple.utils.Utils;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {
  private WPI_TalonSRX masterMotor;
  private WPI_TalonSRX slaveMotor;

  /**
   * Creates a new Elevator.
   */
  public Elevator() {
    masterMotor = new WPI_TalonSRX(RobotMap.ELEVATOR_MASTER_MOTOR_DEVICE_NUM);
    slaveMotor = new WPI_TalonSRX(RobotMap.ELEVATOR_SLAVE_MOTOR_DEVICE_NUM);

    masterMotor.setNeutralMode(NeutralMode.Brake);
    slaveMotor.setNeutralMode(NeutralMode.Brake);

    masterMotor.configPeakOutputForward(Constants.ELEVATOR_MAX_POWER_FORWARD);
    masterMotor.configPeakOutputReverse(Constants.ELEVATOR_MAX_POWER_REVERSE);

    slaveMotor.configPeakOutputForward(Constants.ELEVATOR_MAX_POWER_FORWARD);
    slaveMotor.configPeakOutputReverse(Constants.ELEVATOR_MAX_POWER_REVERSE);

    slaveMotor.set(ControlMode.Follower, masterMotor.getDeviceID());

    masterMotor.setSensorPhase(true);
    resetPostion();

    // masterMotor.setInverted(true);
    configPID();
  }

  public void configPID(){
    masterMotor.config_kP(0, Constants.ELEVATOR_PID.kP);
    masterMotor.config_kI(0, Constants.ELEVATOR_PID.kI);
    masterMotor.config_kD(0, Constants.ELEVATOR_PID.kD); 
    masterMotor.config_kF(0, Constants.ELEVATOR_PID.kF); 
  }

  public void setPower(double power){
    if (getPos() < Constants.ELEVATOR_MAX_POS) { 
      masterMotor.set(ControlMode.PercentOutput, 0);
      return;
    }
    masterMotor.set(ControlMode.PercentOutput, power);
    SmartDashboard.putNumber("Elevator master output", masterMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Elevator slave output", slaveMotor.getMotorOutputPercent());
  }

  public void setPosition(double pos) {
    if (pos < Constants.ELEVATOR_MAX_POS) {
      masterMotor.set(ControlMode.Position, Constants.ELEVATOR_MAX_POS);
      return;
    }

    masterMotor.set(ControlMode.Position, pos);
  }

  private void setPositionEncoder(int postion) {
    this.masterMotor.setSelectedSensorPosition(postion);
  }

  public void resetPostion() {
    this.setPositionEncoder(0);
  }

  public double getPos(){
    return masterMotor.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", getPos());
    // This method will be called once per scheduler run
  }
}
