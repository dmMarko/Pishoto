// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotMap;

// public class Elevator extends SubsystemBase {
//   private WPI_TalonSRX masterMotor;
//   private WPI_TalonSRX slaveMotor;

//   /**
//    * Creates a new Elevator.
//    */
//   public Elevator() {
//     masterMotor = new WPI_TalonSRX(RobotMap.ELEVATOR_MASTER_MOTOR_DEVICE_NUM);
//     slaveMotor = new WPI_TalonSRX(RobotMap.ELEVATOR_SLAVE_MOTOR_DEVICE_NUM);

//     slaveMotor.set(ControlMode.Follower, masterMotor.getDeviceID());
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
