// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.LibPurple.swerve.Swerve;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.sensors.CANCoder;

// import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.LibPurple.control.PID254;
// import frc.LibPurple.control.PIDController;
// import frc.LibPurple.control.PIDvalue;
// import frc.LibPurple.math.Vector2D;
// import frc.LibPurple.sensors.Dashboard3075;
// import frc.LibPurple.utils.Utils;
// import frc.robot.Constants;

// /**
//  * Add your docs here.
//  */
// public class SwerveModule3075V2 extends SwerveModule {
//     WPI_TalonFX angleMotor, driveMotor;
//     AnalogInput angleEncoder;
//     boolean motorPhase, invertEncoder;
//     private final double angleOffset, distancePerPulse;
//     private double inverted = 1.0;
//     // PIDController angleController;
//     PID254 angleController;

//     double error = 0.0;

//     public SwerveModule3075V2(Vector2D modulePos, double angleOffset, WPI_TalonFX angleMotor,
//      WPI_TalonFX driveMotor, AnalogInput angleEncoder, boolean motorPhase, boolean invertEncoder, double distancePerPulse, PIDvalue pidvalue){
//         super(modulePos);
//         this.angleOffset = angleOffset;
//         this.motorPhase = motorPhase;
//         this.invertEncoder = invertEncoder;
//         this.distancePerPulse = distancePerPulse;

//         this.driveMotor = driveMotor;
//         this.angleMotor = angleMotor;
//         this.angleEncoder = angleEncoder;

//         driveMotor.setInverted(motorPhase);
//         driveMotor.configPeakOutputForward(1);
//         driveMotor.configPeakOutputReverse(-1);
//         driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
//         driveMotor.setNeutralMode(NeutralMode.Brake);
        
//         angleMotor.configPeakOutputForward(1);
//         angleMotor.configPeakOutputReverse(-1);
//         angleMotor.setNeutralMode(NeutralMode.Brake);
//         // angleMotor.setInverted(true);
//         // angleEncoder.setPositionToAbsolute(10);
//         // angleEncoder.configMagnetOffset(angleOffset);
//         // angleEncoder.configSensorDirection(invertEncoder);
//         // setAnglePID(Constants.anglePIDValue);
//         setDrivePID(Constants.drivePIDValue);

//         // angleController = new PIDController(pidvalue);
//         angleController = new PID254(pidvalue.kP, pidvalue.kI, pidvalue.kD);
        
//         angleController.setInputRange(0, 2.0 * Math.PI);
//         angleController.setContinuous(true);
//         angleController.setOutputRange(-0.6, 0.6);

//         if(invertEncoder){
//             inverted = -inverted;
//         }

//     }

//     public void setAnglePID(PIDvalue value){
//         angleMotor.config_kP(0, value.kP, 10);
//         angleMotor.config_kI(0, value.kI, 10);
//         angleMotor.config_kD(0, value.kD, 10);
//     }

//     public void setDrivePID(PIDvalue value){
//         driveMotor.config_kP(0, value.kP, 10);
//         driveMotor.config_kI(0, value.kI, 10);
//         driveMotor.config_kD(0, value.kD, 10);
//         driveMotor.config_kF(0, value.kF, 10);
//     }

//     public int degreesToEncUnits(double degrees){
// 		return (int) (degrees/360.0*4096);
// 	}
	
// 	public double encUnitsToDegrees(double encUnits){
//         return encUnits/4096*360.0;
//     }

//     public double encUnitsToRadians(double encUnits){
//         return encUnitsToDegrees(encUnits) * Math.PI / 180.0;
//     }

//     public int degreesToEncUnitsFalcon(double degrees){
// 		return (int) (degrees/360.0*2048);
// 	}
	
// 	public double encUnitsToDegreesFalcon(double encUnits){
// 		return encUnits/2048*360.0;
//     }

//     @Override
//     public double getDistancePerPulse() {
//         return distancePerPulse;
//     }

//     @Override
//     public double getAngle() {
//         // return Math.toRadians(angleEncoder.getAbsolutePosition());
//         // return Constants.getAngleModule(getModuleName()) + Math.toRadians(angleOffset);
//         double angle = -(1.0 - angleEncoder.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI + Math.toRadians(angleOffset);
//         angle %= 2.0 * Math.PI;
//         if (angle > 0.0) {
//             angle -= 2.0 * Math.PI;
//         }

//         return angle;
//     }

//     @Override
//     public double getDistance() {
//         return getPos() / distancePerPulse;
//     }

//     public double getVelocity(){
//         return driveMotor.getSelectedSensorVelocity();
//     }

//     public int getPos(){
//         return driveMotor.getSelectedSensorPosition();
//     }

//     public double getPosInverted(){
//         return getPos() * inverted;
//     }

//     public double getError(){
//         return error;
//     }

//     @Override
//     public void setTargetAngle(double angle) {
//         // angleController.setSetpoint((angle + 2 * Math.PI) % (2 * Math.PI));
//         // angleController.setSetpoint(angle % (2 * Math.PI));
//         angleController.setSetpoint(angle);
//         // angleController.setSetpoint(Math.PI - Math.PI / 4);
//         // angleController.setSetpoint(2 * Math.PI);
//         // angleController.setSetpoint(Math.PI / 2);
//         // angleController.setSetpoint(0);
//     }

//     @Override
//     public void setAngleOutput(double output){
//         SmartDashboard.putNumber("output angle", output);
//         angleMotor.set(ControlMode.PercentOutput, output);
//     }

//     @Override
//     public void setDriveOutput(double output) {
//         driveMotor.set(output);
//     }

//     @Override
//     public void setVelocity(double velocity) {
//         driveMotor.set(ControlMode.Velocity, velocity);
//     }

//     @Override
//     public double getSetDrive() {
//         return driveMotor.get();
//     }

//     @Override
//     public double getSetAngle() {
//         return angleMotor.get();
//     }

//     @Override
//     public void updateState(double dt){
//         super.updateState(dt);
//         // SmartDashboard.putNumber("angle" + getModuleNumber(), angleEncoder.getAbsolutePosition());
//         double absError = angleController.getSetpoint() - getAngle();
//         double error = 0;
//         if(absError < Math.PI){
//             error = absError;
//         }
//         else{
//             error = absError - 2 * Math.PI;
//         }
//         double power = angleController.calculate(angleController.getSetpoint() - error, dt);
//         // double power = angleController.calculate(getAngle(), dt);
//         SmartDashboard.putNumber("error" + getModuleName(), Math.toDegrees(error));
//         SmartDashboard.putNumber("set " + getModuleName(), Math.toDegrees(angleController.getSetpoint()));
//         setAngleOutput(power);
//         SmartDashboard.putNumber("power " + getModuleName(), angleMotor.getMotorOutputPercent());
//         // driveMotor.set(0);
//         // angleMotor.set(0.07);
//     }

//     @Override
//     public void updateStateAuto(double dt) {
//         super.updateStateAuto(dt);
//         angleMotor.set(angleController.calculate(getAngle(), dt));
//     }

//     @Override
//     public void resetKinematics(){
//         super.resetKinematics();
//         reset();
//     }

//     public void reset(){
//         driveMotor.setSelectedSensorPosition(0);
//     }

//     public double getAngleSetpoint(){
//         return angleMotor.getMotorOutputVoltage();
//     }

//     @Override
//     public void resetPID(){
//         angleController.reset();
//     }

// }
